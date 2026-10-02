//! Contraction hierarchies (CH) for fast point-to-point routing.
//!
//! Preprocessing contracts nodes one at a time, least important first. When
//! node `v` is removed, every pair of neighbours `u → v → x` whose only
//! shortest connection runs through `v` gets a *shortcut* arc `u → x`
//! carrying the combined cost; a bounded "witness" search decides whether some
//! other path is at least as short. Queries then run a bidirectional Dijkstra
//! that only ever climbs towards more important nodes, which settles a few
//! hundred nodes instead of a large share of the city.
//!
//! Shortcuts remember the two arcs they replace, so a query result unpacks
//! into the original petgraph edges and route geometry works unchanged.
//! Results are exact: same optimal cost as Dijkstra on the same edge costs.

use petgraph::graph::{EdgeIndex, NodeIndex};

use std::collections::HashMap;

use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::search::{seed, usable, workspace, HeapEntry, SearchIndex, SearchPath, Workspace, NONE};

/// Witness searches give up after settling this many nodes. Giving up only
/// ever adds a redundant shortcut, never a wrong one. Priority estimates can
/// be much rougher than the real contraction: on the Munich walking graph,
/// (300, 3) prepares 2.2x faster than (500, 50) for ~20% slower queries.
const CONTRACTION_SETTLE_LIMIT: usize = 300;
const SIMULATION_SETTLE_LIMIT: usize = 3;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
enum ArcKind {
    /// An original graph edge (petgraph edge index).
    Edge(u32),
    /// A shortcut standing for arc `.0` followed by arc `.1`.
    Shortcut(u32, u32),
}

/// One direction of the search graph, grouped by owning node.
#[derive(Clone, Serialize, Deserialize)]
struct UpwardArcs {
    offsets: Vec<u32>,
    /// The higher-ranked node at the other end of each arc.
    heads: Vec<u32>,
    weights: Vec<f64>,
    arcs: Vec<u32>,
}

impl UpwardArcs {
    fn build(node_count: usize, mut items: Vec<(u32, u32, f64, u32)>) -> Self {
        items.sort_unstable_by_key(|&(owner, head, _, arc)| (owner, head, arc));
        let mut offsets = vec![0u32; node_count + 1];
        for &(owner, ..) in &items {
            offsets[owner as usize + 1] += 1;
        }
        for i in 0..node_count {
            offsets[i + 1] += offsets[i];
        }
        Self {
            offsets,
            heads: items.iter().map(|i| i.1).collect(),
            weights: items.iter().map(|i| i.2).collect(),
            arcs: items.iter().map(|i| i.3).collect(),
        }
    }

    #[inline]
    fn range(&self, node: u32) -> std::ops::Range<usize> {
        self.offsets[node as usize] as usize..self.offsets[node as usize + 1] as usize
    }
}

/// A prepared contraction hierarchy for one cost field.
#[derive(Clone, Serialize, Deserialize)]
pub(crate) struct ContractionHierarchy {
    node_count: usize,
    /// Arcs `u → v` with `rank(v) > rank(u)`, owned by `u` (forward search).
    up: UpwardArcs,
    /// Arcs `v → u` with `rank(v) > rank(u)`, owned by `u` (backward search).
    down: UpwardArcs,
    kinds: Vec<ArcKind>,
    /// Length in metres of the road each arc stands for.
    lengths: Vec<f64>,
}

/// A search seed: `(node, cost, distance in metres)`.
pub(crate) type Seed = (u32, f64, f64);

/// Mutable state while contracting.
struct Builder {
    /// Remaining (uncontracted) neighbours: `(node, arc)`.
    out: Vec<Vec<(u32, u32)>>,
    inc: Vec<Vec<(u32, u32)>>,
    from: Vec<u32>,
    to: Vec<u32>,
    weight: Vec<f64>,
    kinds: Vec<ArcKind>,
    /// Replaced by a cheaper parallel shortcut; excluded from the result.
    superseded: Vec<bool>,
    /// Original edges an arc stands for (1 for an edge, more for shortcuts).
    hops: Vec<u32>,
    contracted: Vec<bool>,
    /// Height of the hierarchy already built below each node.
    depth: Vec<u32>,
}

struct Shortcut {
    from: u32,
    to: u32,
    weight: f64,
    first: u32,
    second: u32,
}

impl Builder {
    fn new(index: &SearchIndex, costs: &[f64]) -> Self {
        let n = index.node_count();
        let mut builder = Builder {
            out: vec![Vec::new(); n],
            inc: vec![Vec::new(); n],
            from: Vec::new(),
            to: Vec::new(),
            weight: Vec::new(),
            kinds: Vec::new(),
            superseded: Vec::new(),
            hops: Vec::new(),
            contracted: vec![false; n],
            depth: vec![0; n],
        };
        // Keep the cheapest of any parallel edges (first one on ties).
        let mut arcs: Vec<(u32, f64, u32)> = Vec::new();
        for u in 0..n as u32 {
            arcs.clear();
            for slot in index.out.range(u) {
                let (v, w) = (index.out.neighbors[slot], costs[slot]);
                if v != u && usable(w) {
                    arcs.push((v, w, index.out.edges[slot]));
                }
            }
            arcs.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
            arcs.dedup_by_key(|a| a.0);
            for &(v, w, edge) in &arcs {
                builder.add_arc(u, v, w, ArcKind::Edge(edge));
            }
        }
        builder
    }

    fn add_arc(&mut self, from: u32, to: u32, weight: f64, kind: ArcKind) -> u32 {
        let arc = self.from.len() as u32;
        self.from.push(from);
        self.to.push(to);
        self.weight.push(weight);
        self.hops.push(match kind {
            ArcKind::Edge(_) => 1,
            ArcKind::Shortcut(a, b) => self.hops[a as usize] + self.hops[b as usize],
        });
        self.kinds.push(kind);
        self.superseded.push(false);
        self.out[from as usize].push((to, arc));
        self.inc[to as usize].push((from, arc));
        arc
    }

    /// Shortcuts needed if `v` were contracted now.
    fn shortcuts_for(&self, v: u32, ws: &mut Workspace, settle_limit: usize) -> Vec<Shortcut> {
        let mut shortcuts = Vec::new();
        // Mark v's out-neighbours so witness searches can stop once all are settled.
        ws.clear_marks();
        let mut target_count = 0;
        for &(x, _) in &self.out[v as usize] {
            target_count += usize::from(ws.mark(x));
        }
        for &(u, uv) in &self.inc[v as usize] {
            let w_uv = self.weight[uv as usize];
            let max_via = self.out[v as usize]
                .iter()
                .filter(|&&(x, _)| x != u)
                .map(|&(_, vx)| w_uv + self.weight[vx as usize])
                .fold(f64::NEG_INFINITY, f64::max);
            if max_via == f64::NEG_INFINITY {
                continue;
            }
            let targets = target_count - usize::from(ws.is_marked(u));
            self.witness_search(u, v, max_via, targets, settle_limit, ws);
            for &(x, vx) in &self.out[v as usize] {
                let via = w_uv + self.weight[vx as usize];
                if x != u && ws.dist(x) > via {
                    shortcuts.push(Shortcut {
                        from: u,
                        to: x,
                        weight: via,
                        first: uv,
                        second: vx,
                    });
                }
            }
            ws.reset();
        }
        shortcuts
    }

    /// Bounded Dijkstra from `source` that avoids `skip`. Stops past
    /// `max_cost`, after `settle_limit` nodes, or once `targets` marked nodes
    /// have been settled.
    fn witness_search(
        &self,
        source: u32,
        skip: u32,
        max_cost: f64,
        mut targets: usize,
        settle_limit: usize,
        ws: &mut Workspace,
    ) {
        ws.relax(source, 0.0, NONE, NONE);
        ws.heap.push(HeapEntry {
            key: 0.0,
            cost: 0.0,
            node: source,
        });
        let mut settled = 0;
        while let Some(HeapEntry { cost, node, .. }) = ws.heap.pop() {
            if cost > ws.dist(node) || !ws.settle(node) {
                continue;
            }
            settled += 1;
            if cost > max_cost || settled > settle_limit {
                break;
            }
            if node != source && ws.is_marked(node) {
                targets -= 1;
                if targets == 0 {
                    break;
                }
            }
            for &(next, arc) in &self.out[node as usize] {
                if next == skip {
                    continue;
                }
                let next_cost = cost + self.weight[arc as usize];
                if next_cost <= max_cost && ws.relax(next, next_cost, node, arc) {
                    ws.heap.push(HeapEntry {
                        key: next_cost,
                        cost: next_cost,
                        node: next,
                    });
                }
            }
        }
    }

    /// Contraction order heuristic (the one OSRM uses): prefer nodes whose
    /// removal adds few shortcuts relative to the arcs it removes, counting
    /// both arcs and the original edges they stand for, and keep the
    /// hierarchy shallow by penalising depth. Lower goes first.
    fn priority(&self, v: u32, settle_limit: usize, ws: &mut Workspace) -> i64 {
        let shortcuts = self.shortcuts_for(v, ws, settle_limit);
        let arcs = self.inc[v as usize].iter().chain(&self.out[v as usize]);
        let removed = self.inc[v as usize].len() + self.out[v as usize].len();
        let removed_hops: u32 = arcs.map(|&(_, arc)| self.hops[arc as usize]).sum();
        let added_hops: u32 = shortcuts
            .iter()
            .map(|s| self.hops[s.first as usize] + self.hops[s.second as usize])
            .sum();
        let edge_quotient = shortcuts.len() as f64 / removed.max(1) as f64;
        let hop_quotient = added_hops as f64 / removed_hops.max(1) as f64;
        let score = 2.0 * edge_quotient + hop_quotient + self.depth[v as usize] as f64;
        (score * 1024.0) as i64
    }

    /// Whether `v` beats every remaining neighbour on priority (ties broken by
    /// a hash of the node id), so it can be contracted alongside the other
    /// local minima: no two of them are adjacent.
    fn is_local_minimum(&self, v: u32, priority: &[i64]) -> bool {
        let key = |n: u32| (priority[n as usize], spread(n));
        let mine = key(v);
        self.out[v as usize]
            .iter()
            .chain(&self.inc[v as usize])
            .all(|&(n, _)| mine < key(n))
    }

    /// Remove `v`, insert its shortcuts, and return its former neighbours.
    fn contract(&mut self, v: u32, shortcuts: Vec<Shortcut>) -> Vec<u32> {
        let vi = v as usize;
        self.contracted[vi] = true;
        let mut neighbors: Vec<u32> = Vec::new();
        for (u, _) in std::mem::take(&mut self.inc[vi]) {
            self.out[u as usize].retain(|&(t, _)| t != v);
            neighbors.push(u);
        }
        for (x, _) in std::mem::take(&mut self.out[vi]) {
            self.inc[x as usize].retain(|&(f, _)| f != v);
            neighbors.push(x);
        }
        neighbors.sort_unstable();
        neighbors.dedup();
        let depth = self.depth[vi] + 1;
        for &n in &neighbors {
            let d = &mut self.depth[n as usize];
            *d = (*d).max(depth);
        }

        for s in shortcuts {
            let existing = self.out[s.from as usize]
                .iter()
                .position(|&(t, _)| t == s.to);
            if let Some(pos) = existing {
                let old = self.out[s.from as usize][pos].1;
                if s.weight >= self.weight[old as usize] {
                    continue;
                }
                self.superseded[old as usize] = true;
                self.out[s.from as usize].swap_remove(pos);
                let inc = &mut self.inc[s.to as usize];
                if let Some(p) = inc.iter().position(|&(_, a)| a == old) {
                    inc.swap_remove(p);
                }
            }
            self.add_arc(s.from, s.to, s.weight, ArcKind::Shortcut(s.first, s.second));
        }
        neighbors
    }
}

/// Cheap integer hash used to break priority ties without a spatial bias.
#[inline]
fn spread(n: u32) -> u32 {
    ((n as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) >> 32) as u32
}

impl ContractionHierarchy {
    /// Number of search states the hierarchy was built for.
    pub(crate) fn node_count(&self) -> usize {
        self.node_count
    }

    /// Contract the graph described by `index` with per-slot forward costs;
    /// `edge_length` gives each graph edge's length for matrix distances.
    ///
    /// Works in rounds: every node whose priority is a local minimum among
    /// its neighbours is contracted in the same round, with the witness
    /// searches run in parallel. Those nodes are pairwise non-adjacent, so
    /// their shortcuts are independent; applying them in a fixed order keeps
    /// the result identical whatever the thread count.
    pub(crate) fn build(
        index: &SearchIndex,
        costs: &[f64],
        edge_length: impl Fn(u32) -> f64,
    ) -> Self {
        let n = index.node_count();
        let mut builder = Builder::new(index, costs);

        let mut priority: Vec<i64> = (0..n as u32)
            .into_par_iter()
            .map(|v| builder.priority(v, SIMULATION_SETTLE_LIMIT, &mut workspace(n)))
            .collect();

        let mut rank = vec![0u32; n];
        let mut next_rank = 0u32;
        let mut remaining: Vec<u32> = (0..n as u32).collect();
        while !remaining.is_empty() {
            let chosen: Vec<u32> = remaining
                .par_iter()
                .copied()
                .filter(|&v| builder.is_local_minimum(v, &priority))
                .collect();
            let plans: Vec<(u32, Vec<Shortcut>)> = chosen
                .par_iter()
                .map(|&v| {
                    let shortcuts =
                        builder.shortcuts_for(v, &mut workspace(n), CONTRACTION_SETTLE_LIMIT);
                    (v, shortcuts)
                })
                .collect();

            let mut affected = Vec::new();
            for (v, shortcuts) in plans {
                rank[v as usize] = next_rank;
                next_rank += 1;
                affected.extend(builder.contract(v, shortcuts));
            }
            affected.sort_unstable();
            affected.dedup();
            affected.retain(|&u| !builder.contracted[u as usize]);

            let updated: Vec<i64> = affected
                .par_iter()
                .map(|&u| builder.priority(u, SIMULATION_SETTLE_LIMIT, &mut workspace(n)))
                .collect();
            for (&u, p) in affected.iter().zip(updated) {
                priority[u as usize] = p;
            }
            remaining.retain(|&v| !builder.contracted[v as usize]);
        }

        let mut up = Vec::new();
        let mut down = Vec::new();
        for arc in 0..builder.from.len() {
            if builder.superseded[arc] {
                continue;
            }
            let (u, x, w) = (builder.from[arc], builder.to[arc], builder.weight[arc]);
            if rank[x as usize] > rank[u as usize] {
                up.push((u, x, w, arc as u32));
            } else {
                down.push((x, u, w, arc as u32));
            }
        }

        // Shortcuts only combine earlier arcs, so one pass sums their lengths.
        let mut lengths: Vec<f64> = Vec::with_capacity(builder.kinds.len());
        for kind in &builder.kinds {
            let length = match *kind {
                ArcKind::Edge(edge) => edge_length(edge),
                ArcKind::Shortcut(a, b) => lengths[a as usize] + lengths[b as usize],
            };
            lengths.push(length);
        }
        ContractionHierarchy {
            node_count: n,
            up: UpwardArcs::build(n, up),
            down: UpwardArcs::build(n, down),
            kinds: builder.kinds,
            lengths,
        }
    }

    /// Check that a hierarchy read from a file is internally consistent for
    /// a graph with `edge_count` edges, so queries cannot index out of range
    /// or loop forever unpacking shortcuts.
    pub(crate) fn validate(&self, edge_count: usize) -> Result<(), &'static str> {
        let arc_count = self.kinds.len();
        for arcs in [&self.up, &self.down] {
            let len = arcs.heads.len();
            if arcs.offsets.len() != self.node_count + 1
                || arcs.offsets.first() != Some(&0)
                || arcs.offsets.last().map(|&o| o as usize) != Some(len)
                || arcs.offsets.windows(2).any(|w| w[0] > w[1])
                || arcs.weights.len() != len
                || arcs.arcs.len() != len
            {
                return Err("routing hierarchy has a malformed adjacency");
            }
            if arcs.heads.iter().any(|&h| h as usize >= self.node_count)
                || arcs.arcs.iter().any(|&a| a as usize >= arc_count)
                || arcs.weights.iter().any(|w| !(w.is_finite() && *w >= 0.0))
            {
                return Err("routing hierarchy has an arc out of range");
            }
        }
        if self.lengths.len() != arc_count
            || self.lengths.iter().any(|l| !(l.is_finite() && *l >= 0.0))
        {
            return Err("routing hierarchy has invalid arc lengths");
        }
        // Shortcuts only ever combine earlier arcs, which also rules out cycles.
        let consistent = self.kinds.iter().enumerate().all(|(i, kind)| match *kind {
            ArcKind::Edge(e) => (e as usize) < edge_count,
            ArcKind::Shortcut(a, b) => (a as usize) < i && (b as usize) < i,
        });
        if !consistent {
            return Err("routing hierarchy has a shortcut out of range");
        }
        Ok(())
    }

    /// Number of shortcut arcs added by preprocessing.
    #[cfg(test)]
    pub(crate) fn shortcut_count(&self) -> usize {
        self.kinds
            .iter()
            .filter(|k| matches!(k, ArcKind::Shortcut(..)))
            .count()
    }

    /// Exact shortest path from any of `sources` to any of `targets`, each a
    /// `(node, offset)` pair: its cost and original edges in travel order.
    pub(crate) fn shortest_path(
        &self,
        sources: &[(u32, f64)],
        targets: &[(u32, f64)],
    ) -> Option<SearchPath> {
        let mut fwd = workspace(self.node_count);
        let mut bwd = workspace(self.node_count);
        seed(&mut fwd, sources, self.node_count, |_| 0.0, None);
        seed(&mut bwd, targets, self.node_count, |_| 0.0, None);

        let mut best = f64::INFINITY;
        let mut meet = NONE;
        loop {
            let top = |ws: &Workspace| ws.heap.peek().map_or(f64::INFINITY, |e| e.key);
            let (fwd_top, bwd_top) = (top(&fwd), top(&bwd));
            if fwd_top >= best && bwd_top >= best {
                break;
            }
            let forward = fwd_top <= bwd_top;
            let (this, other, arcs, stall_arcs) = if forward {
                (&mut fwd, &bwd, &self.up, &self.down)
            } else {
                (&mut bwd, &fwd, &self.down, &self.up)
            };
            let HeapEntry { cost, node, .. } = this.heap.pop().expect("non-empty below best");
            if cost > this.dist(node) || !this.settle(node) {
                continue;
            }
            let through = cost + other.dist(node);
            if through < best {
                best = through;
                meet = node;
            }
            // Stall on demand: a higher node already offers a shorter way
            // here, so nothing above this one can be on a shortest path.
            let stalled = stall_arcs
                .range(node)
                .any(|i| this.dist(stall_arcs.heads[i]) + stall_arcs.weights[i] < cost);
            if stalled {
                continue;
            }
            for i in arcs.range(node) {
                let (head, next_cost) = (arcs.heads[i], cost + arcs.weights[i]);
                if next_cost < best && this.relax(head, next_cost, node, arcs.arcs[i]) {
                    this.heap.push(HeapEntry {
                        key: next_cost,
                        cost: next_cost,
                        node: head,
                    });
                }
            }
        }

        if meet == NONE {
            return None;
        }
        let mut path_arcs = Vec::new();
        let mut first = meet;
        loop {
            let (pred, arc) = fwd.pred(first);
            if pred == NONE {
                break;
            }
            path_arcs.push(arc);
            first = pred;
        }
        path_arcs.reverse();
        let mut last = meet;
        loop {
            let (next, arc) = bwd.pred(last);
            if next == NONE {
                break;
            }
            path_arcs.push(arc);
            last = next;
        }
        Some(SearchPath {
            cost: best,
            first: NodeIndex::new(first as usize),
            last: NodeIndex::new(last as usize),
            edges: self.unpack(&path_arcs),
        })
    }

    /// Every node an upward search from `seeds` settles without stalling,
    /// with its cost and the length of road its best path covers: forward
    /// over `up` arcs, backward over `down` arcs.
    fn upward_space(&self, seeds: &[Seed], forward: bool) -> Vec<Seed> {
        let (arcs, stall_arcs) = if forward {
            (&self.up, &self.down)
        } else {
            (&self.down, &self.up)
        };
        let mut seed_length: HashMap<u32, (f64, f64)> = HashMap::new();
        for &(node, cost, length) in seeds {
            let entry = seed_length.entry(node).or_insert((cost, length));
            if cost < entry.0 {
                *entry = (cost, length);
            }
        }
        let costs: Vec<(u32, f64)> = seeds.iter().map(|&(node, cost, _)| (node, cost)).collect();
        let mut ws = workspace(self.node_count);
        seed(&mut ws, &costs, self.node_count, |_| 0.0, None);
        let mut space = Vec::new();
        let mut length_of: HashMap<u32, f64> = HashMap::new();
        while let Some(HeapEntry { cost, node, .. }) = ws.heap.pop() {
            if cost > ws.dist(node) || !ws.settle(node) {
                continue;
            }
            let stalled = stall_arcs
                .range(node)
                .any(|i| ws.dist(stall_arcs.heads[i]) + stall_arcs.weights[i] < cost);
            if stalled {
                continue;
            }
            // Predecessors are always expanded (never stalled) nodes.
            let length = match ws.pred(node) {
                (NONE, _) => seed_length.get(&node).map_or(0.0, |&(_, l)| l),
                (pred, arc) => length_of[&pred] + self.lengths[arc as usize],
            };
            length_of.insert(node, length);
            space.push((node, cost, length));
            for i in arcs.range(node) {
                let (head, next_cost) = (arcs.heads[i], cost + arcs.weights[i]);
                if ws.relax(head, next_cost, node, arcs.arcs[i]) {
                    ws.heap.push(HeapEntry {
                        key: next_cost,
                        cost: next_cost,
                        node: head,
                    });
                }
            }
        }
        space
    }

    /// Exact costs from every source to every target (each a set of seeds),
    /// row-major, infinite where unreachable, with the length in metres of
    /// each optimal path.
    ///
    /// The bucket method: each target's backward upward search leaves
    /// `(target, cost)` in a bucket at every node it settles, then each
    /// source's forward upward search combines its costs with the buckets it
    /// passes. A shortest path's highest node is in both search spaces, so
    /// one forward and one backward search per point cover the whole table.
    pub(crate) fn many_to_many(
        &self,
        sources: &[Vec<Seed>],
        targets: &[Vec<Seed>],
    ) -> (Vec<f64>, Vec<f64>) {
        let width = targets.len();
        let mut table = vec![(f64::INFINITY, f64::INFINITY); sources.len() * width];
        if width == 0 {
            return (Vec::new(), Vec::new());
        }
        let mut entries: Vec<(u32, u32, f64, f64)> = targets
            .par_iter()
            .enumerate()
            .flat_map_iter(|(t, seeds)| {
                self.upward_space(seeds, false)
                    .into_iter()
                    .map(move |(node, cost, length)| (node, t as u32, cost, length))
            })
            .collect();
        entries.par_sort_unstable_by_key(|&(node, target, ..)| (node, target));
        let mut offsets = vec![0usize; self.node_count + 1];
        for &(node, ..) in &entries {
            offsets[node as usize + 1] += 1;
        }
        for i in 0..self.node_count {
            offsets[i + 1] += offsets[i];
        }

        table
            .par_chunks_mut(width)
            .zip(sources.par_iter())
            .for_each(|(row, seeds)| {
                for (node, cost, length) in self.upward_space(seeds, true) {
                    let bucket = &entries[offsets[node as usize]..offsets[node as usize + 1]];
                    for &(_, target, back, back_length) in bucket {
                        let cell = &mut row[target as usize];
                        if cost + back < cell.0 {
                            *cell = (cost + back, length + back_length);
                        }
                    }
                }
            });
        table.into_iter().unzip()
    }

    /// Expand shortcut arcs into original edges, preserving travel order.
    fn unpack(&self, arcs: &[u32]) -> Vec<EdgeIndex> {
        let mut edges = Vec::with_capacity(arcs.len() * 4);
        let mut stack: Vec<u32> = arcs.iter().rev().copied().collect();
        while let Some(arc) = stack.pop() {
            match self.kinds[arc as usize] {
                ArcKind::Edge(edge) => edges.push(EdgeIndex::new(edge as usize)),
                ArcKind::Shortcut(first, second) => {
                    stack.push(second);
                    stack.push(first);
                }
            }
        }
        edges
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::search::{astar, dijkstra};
    use petgraph::graph::DiGraph;

    /// A small pseudo-random road-like grid with one-way streets.
    fn grid(size: u32, seed: u64) -> DiGraph<(), f64> {
        let mut state = seed;
        let mut rand = move || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (state >> 33) as f64 / (1u64 << 31) as f64
        };
        let mut g = DiGraph::new();
        let nodes: Vec<_> = (0..size * size).map(|_| g.add_node(())).collect();
        for r in 0..size {
            for c in 0..size {
                let here = nodes[(r * size + c) as usize];
                for (dr, dc) in [(0, 1), (1, 0)] {
                    let (nr, nc) = (r + dr, c + dc);
                    if nr < size && nc < size {
                        let there = nodes[(nr * size + nc) as usize];
                        let w = 1.0 + 9.0 * rand();
                        let roll = rand();
                        if roll > 0.15 {
                            g.add_edge(here, there, w);
                        }
                        if roll < 0.85 {
                            g.add_edge(there, here, w * (0.8 + 0.4 * rand()));
                        }
                    }
                }
            }
        }
        g
    }

    #[test]
    fn ch_matches_dijkstra_on_every_pair() {
        for seed in [1, 2, 3] {
            let g = grid(9, seed);
            let index = SearchIndex::new(&g, &[], &[]);
            let costs: Vec<f64> = index
                .out
                .edges
                .iter()
                .map(|&e| g[EdgeIndex::new(e as usize)])
                .collect();
            // Lengths equal to costs: every path's length must equal its cost.
            let ch = ContractionHierarchy::build(&index, &costs, |e| g[EdgeIndex::new(e as usize)]);
            assert!(ch.shortcut_count() > 0);

            for s in g.node_indices() {
                let root = |n: NodeIndex| [(n.index() as u32, 0.0)];
                let exact = dijkstra(&index.out, &root(s), f64::INFINITY, |slot| costs[slot]);
                for t in g.node_indices() {
                    let got = ch.shortest_path(&root(s), &root(t));
                    match (got.map(|p| (p.cost, p.edges)), exact.get(t)) {
                        (None, None) => {}
                        (Some((cost, edges)), Some(&want)) => {
                            assert!((cost - want).abs() < 1e-9, "{s:?}->{t:?}: {cost} vs {want}");
                            // The unpacked edges form a connected s → t walk
                            // whose cost is the reported optimum.
                            let mut at = s;
                            let mut total = 0.0;
                            for e in &edges {
                                let (a, b) = g.edge_endpoints(*e).unwrap();
                                assert_eq!(a, at);
                                at = b;
                                total += g[*e];
                            }
                            assert_eq!(at, t);
                            assert!((total - want).abs() < 1e-9);
                        }
                        other => panic!("{s:?}->{t:?} reachability differs: {other:?}"),
                    }
                }
            }
            // The bucket table agrees too, including multi-seed points.
            let points: Vec<Vec<Seed>> = (0..g.node_count() as u32)
                .map(|n| {
                    vec![
                        (n, 0.5, 0.5),
                        ((n * 7 + 3) % g.node_count() as u32, 2.0, 2.0),
                    ]
                })
                .collect();
            let (table, lengths) = ch.many_to_many(&points, &points);
            for (i, from) in points.iter().enumerate() {
                let from: Vec<(u32, f64)> = from.iter().map(|&(n, c, _)| (n, c)).collect();
                let exact = dijkstra(&index.out, &from, f64::INFINITY, |slot| costs[slot]);
                for (j, to) in points.iter().enumerate() {
                    let want = to
                        .iter()
                        .filter_map(|&(n, off, _)| {
                            exact.get(NodeIndex::new(n as usize)).map(|d| d + off)
                        })
                        .fold(f64::INFINITY, f64::min);
                    let got = table[i * points.len() + j];
                    let length = lengths[i * points.len() + j];
                    assert!(
                        got == length || (got - length).abs() < 1e-9,
                        "{got} vs {length}"
                    );
                    assert!(
                        got == want || (got - want).abs() < 1e-9,
                        "{i}->{j}: {got} vs {want}"
                    );
                }
            }
            // And agrees with A* on a sample.
            let a: NodeIndex = NodeIndex::new(0);
            let b: NodeIndex = NodeIndex::new(g.node_count() - 1);
            let (ra, rb) = ([(a.index() as u32, 0.0)], [(b.index() as u32, 0.0)]);
            let via_astar =
                astar(&index.out, &ra, &rb, |slot| costs[slot], |_| 0.0).map(|r| r.cost);
            let via_ch = ch.shortest_path(&ra, &rb).map(|r| r.cost);
            match (via_astar, via_ch) {
                (Some(x), Some(y)) => assert!((x - y).abs() < 1e-9),
                (x, y) => assert_eq!(x.is_some(), y.is_some()),
            }
        }
    }
}
