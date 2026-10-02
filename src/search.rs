//! Shortest-path kernels shared by routing, reachability, feasibility and the
//! contraction hierarchy.
//!
//! Searches run on [`SearchIndex`], a compact array-based (CSR) copy of the
//! graph's adjacency kept next to the petgraph graph. A relaxation touches a
//! few small arrays (neighbour, cost) instead of chasing petgraph's
//! linked edge lists through ~100-byte edge records. Per-mode costs are laid
//! out in adjacency order and built on first use.
//!
//! Point-to-point searches borrow their scratch arrays from a per-thread pool
//! and reset only the entries they touched, so a query costs O(nodes visited)
//! rather than O(graph size).

use std::cell::RefCell;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};
use std::ops::Range;
use std::sync::OnceLock;

use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};

use crate::graph::NodeMap;

/// Adjacency in compressed sparse row form, for one direction.
///
/// Slots `range(u)` hold the arcs of node `u`: `neighbors[slot]` is the node
/// at the other end and `edges[slot]` the petgraph edge index.
pub(crate) struct Adjacency {
    offsets: Vec<u32>,
    pub(crate) neighbors: Vec<u32>,
    pub(crate) edges: Vec<u32>,
}

impl Adjacency {
    /// Group `arcs` = `(owner, neighbor, edge)` by owner with a counting sort.
    /// Arcs keep their relative order within each owner.
    fn build(node_count: usize, arcs: &[(u32, u32, u32)]) -> Self {
        let mut offsets = vec![0u32; node_count + 1];
        for &(owner, _, _) in arcs {
            offsets[owner as usize + 1] += 1;
        }
        for i in 0..node_count {
            offsets[i + 1] += offsets[i];
        }
        let mut cursor = offsets.clone();
        let mut neighbors = vec![0u32; arcs.len()];
        let mut edges = vec![0u32; arcs.len()];
        for &(owner, neighbor, edge) in arcs {
            let slot = &mut cursor[owner as usize];
            neighbors[*slot as usize] = neighbor;
            edges[*slot as usize] = edge;
            *slot += 1;
        }
        Self {
            offsets,
            neighbors,
            edges,
        }
    }

    #[inline]
    pub(crate) fn range(&self, node: u32) -> Range<usize> {
        self.offsets[node as usize] as usize..self.offsets[node as usize + 1] as usize
    }

    pub(crate) fn node_count(&self) -> usize {
        self.offsets.len() - 1
    }
}

/// Edge costs for one mode, one value per adjacency slot.
pub(crate) struct SlotCosts {
    pub(crate) out: Vec<f64>,
    pub(crate) inc: Vec<f64>,
}

/// Forward and backward adjacency of a graph, plus per-mode slot costs.
///
/// Searches run over *states*. Normally a state is a graph node, but a
/// junction with turn restrictions gets extra states, one per distinct set of
/// banned exits: an edge whose turn options are restricted leads into such a
/// state, whose outgoing arcs omit the banned exits. States `0..n` are the
/// graph's nodes; extra states follow.
pub(crate) struct SearchIndex {
    /// Arcs grouped by source state; neighbour = target state.
    pub(crate) out: Adjacency,
    /// Arcs grouped by target state; neighbour = source state.
    pub(crate) inc: Adjacency,
    costs: [OnceLock<SlotCosts>; 3],
    graph_nodes: usize,
    extra: Vec<ExtraState>,
    extra_of: HashMap<u32, Vec<u32>>,
    head_override: HashMap<u32, u32>,
}

/// A restricted state of a junction: the node, and the exits it bans (sorted).
struct ExtraState {
    node: u32,
    banned: Vec<u32>,
}

impl SearchIndex {
    /// Build the index, splitting junctions so that no path takes any of the
    /// `forbidden` `(into junction, out of junction)` edge pairs.
    pub(crate) fn new<N, E>(graph: &DiGraph<N, E>, forbidden: &[(EdgeIndex, EdgeIndex)]) -> Self {
        let n = graph.node_count();
        let raw = graph.raw_edges();

        // Group incoming edges by (junction, banned exits); each group gets
        // one extra state. Sorted input keeps state numbering deterministic.
        let mut banned_by_entry: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for &(into, out) in forbidden {
            banned_by_entry
                .entry(into.index() as u32)
                .or_default()
                .push(out.index() as u32);
        }
        let mut extra: Vec<ExtraState> = Vec::new();
        let mut extra_of: HashMap<u32, Vec<u32>> = HashMap::new();
        let mut head_override: HashMap<u32, u32> = HashMap::new();
        let mut state_for: HashMap<(u32, Vec<u32>), u32> = HashMap::new();
        for (entry, mut banned) in banned_by_entry {
            banned.sort_unstable();
            banned.dedup();
            let node = raw[entry as usize].target().index() as u32;
            let state = *state_for.entry((node, banned.clone())).or_insert_with(|| {
                let state = (n + extra.len()) as u32;
                extra.push(ExtraState { node, banned });
                extra_of.entry(node).or_default().push(state);
                state
            });
            head_override.insert(entry, state);
        }

        let total = n + extra.len();
        let mut arcs: Vec<(u32, u32, u32)> = Vec::with_capacity(raw.len());
        for (id, e) in raw.iter().enumerate() {
            let id = id as u32;
            let (source, target) = (e.source().index() as u32, e.target().index() as u32);
            let head = head_override.get(&id).copied().unwrap_or(target);
            arcs.push((source, head, id));
            for &state in extra_of.get(&source).into_iter().flatten() {
                if extra[state as usize - n].banned.binary_search(&id).is_err() {
                    arcs.push((state, head, id));
                }
            }
        }
        let out = Adjacency::build(total, &arcs);
        for arc in &mut arcs {
            *arc = (arc.1, arc.0, arc.2);
        }
        let inc = Adjacency::build(total, &arcs);
        Self {
            out,
            inc,
            costs: Default::default(),
            graph_nodes: n,
            extra,
            extra_of,
            head_override,
        }
    }

    /// Whether any junction has restricted states.
    pub(crate) fn has_restricted_states(&self) -> bool {
        !self.extra.is_empty()
    }

    /// The graph node a state belongs to.
    #[inline]
    pub(crate) fn node_of(&self, state: u32) -> u32 {
        if (state as usize) < self.graph_nodes {
            state
        } else {
            self.extra[state as usize - self.graph_nodes].node
        }
    }

    /// The state reached by travelling along `edge` into `target`.
    pub(crate) fn head_state(&self, edge: EdgeIndex, target: NodeIndex) -> u32 {
        self.head_override
            .get(&(edge.index() as u32))
            .copied()
            .unwrap_or(target.index() as u32)
    }

    /// Every state of `node`.
    pub(crate) fn states_of(&self, node: NodeIndex) -> impl Iterator<Item = u32> + '_ {
        let node = node.index() as u32;
        std::iter::once(node).chain(self.extra_of.get(&node).into_iter().flatten().copied())
    }

    /// The states of `source` from which `edge` may be taken.
    pub(crate) fn tail_states(
        &self,
        edge: EdgeIndex,
        source: NodeIndex,
    ) -> impl Iterator<Item = u32> + '_ {
        let id = edge.index() as u32;
        self.states_of(source).filter(move |&state| {
            (state as usize) < self.graph_nodes
                || self.extra[state as usize - self.graph_nodes]
                    .banned
                    .binary_search(&id)
                    .is_err()
        })
    }

    /// Collapse per-state labels (in settle order) to per-node labels,
    /// keeping each node's first, i.e. cheapest, state.
    pub(crate) fn fold_states<T>(&self, labels: NodeMap<T>) -> NodeMap<T> {
        if self.extra.is_empty() {
            return labels;
        }
        let mut nodes = NodeMap::with_node_count(self.graph_nodes);
        for (state, value) in labels.into_entries() {
            let node = NodeIndex::new(self.node_of(state.index() as u32) as usize);
            if !nodes.contains_key(node) {
                nodes.insert(node, value);
            }
        }
        nodes
    }

    /// Slot costs for mode `field`, computing them with `edge_cost` the first
    /// time they are asked for.
    pub(crate) fn costs(&self, field: usize, edge_cost: impl Fn(EdgeIndex) -> f64) -> &SlotCosts {
        self.costs[field].get_or_init(|| {
            let lay_out = |adjacency: &Adjacency| {
                adjacency
                    .edges
                    .iter()
                    .map(|&edge| edge_cost(EdgeIndex::new(edge as usize)))
                    .collect()
            };
            SlotCosts {
                out: lay_out(&self.out),
                inc: lay_out(&self.inc),
            }
        })
    }

    pub(crate) fn node_count(&self) -> usize {
        self.out.node_count()
    }
}

// ---------------------------------------------------------------------------
// Heap entries and scratch space
// ---------------------------------------------------------------------------

/// Binary-heap entry that pops the smallest `key` first.
///
/// `cost` is the path cost carried alongside (equal to `key` for Dijkstra,
/// `key - heuristic` for A*). NaN never enters: costs are validated first.
#[derive(Clone, Copy, Debug)]
pub(crate) struct HeapEntry {
    pub(crate) key: f64,
    pub(crate) cost: f64,
    pub(crate) node: u32,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key
    }
}

impl Eq for HeapEntry {}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.key.total_cmp(&self.key)
    }
}

/// Searches treat non-finite or negative edge costs as impassable.
#[inline]
pub(crate) fn usable(cost: f64) -> bool {
    cost.is_finite() && cost >= 0.0
}

pub(crate) const NONE: u32 = u32::MAX;

/// Per-node labels for one search, reset in O(touched) after use.
#[derive(Default)]
pub(crate) struct Workspace {
    dist: Vec<f64>,
    /// Predecessor node and the arc (edge or slot, caller's choice) used.
    pred_node: Vec<u32>,
    pred_arc: Vec<u32>,
    settled: Vec<bool>,
    touched: Vec<u32>,
    pub(crate) heap: BinaryHeap<HeapEntry>,
    /// Node marks valid for the current `generation` only, so starting a new
    /// set of marks is O(1).
    marks: Vec<u32>,
    generation: u32,
}

impl Workspace {
    fn fit(&mut self, node_count: usize) {
        if self.dist.len() < node_count {
            self.dist.resize(node_count, f64::INFINITY);
            self.pred_node.resize(node_count, NONE);
            self.pred_arc.resize(node_count, NONE);
            self.settled.resize(node_count, false);
            self.marks.resize(node_count, 0);
        }
    }

    /// Forget all marks.
    pub(crate) fn clear_marks(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.marks.fill(0);
            self.generation = 1;
        }
    }

    /// Mark `node`; returns whether it was unmarked before.
    #[inline]
    pub(crate) fn mark(&mut self, node: u32) -> bool {
        let generation = self.generation;
        std::mem::replace(&mut self.marks[node as usize], generation) != generation
    }

    #[inline]
    pub(crate) fn is_marked(&self, node: u32) -> bool {
        self.marks[node as usize] == self.generation
    }

    #[inline]
    pub(crate) fn dist(&self, node: u32) -> f64 {
        self.dist[node as usize]
    }

    #[inline]
    pub(crate) fn pred(&self, node: u32) -> (u32, u32) {
        (self.pred_node[node as usize], self.pred_arc[node as usize])
    }

    /// Lower `node`'s label to `cost` if that improves it; returns whether it did.
    #[inline]
    pub(crate) fn relax(&mut self, node: u32, cost: f64, pred_node: u32, pred_arc: u32) -> bool {
        let i = node as usize;
        if cost < self.dist[i] {
            if self.dist[i] == f64::INFINITY {
                self.touched.push(node); // first label since the last reset
            }
            self.dist[i] = cost;
            self.pred_node[i] = pred_node;
            self.pred_arc[i] = pred_arc;
            true
        } else {
            false
        }
    }

    /// Mark `node` settled; returns false if it already was.
    #[inline]
    pub(crate) fn settle(&mut self, node: u32) -> bool {
        !std::mem::replace(&mut self.settled[node as usize], true)
    }

    pub(crate) fn reset(&mut self) {
        for &node in &self.touched {
            let i = node as usize;
            self.dist[i] = f64::INFINITY;
            self.pred_node[i] = NONE;
            self.pred_arc[i] = NONE;
            self.settled[i] = false;
        }
        self.touched.clear();
        self.heap.clear();
    }
}

thread_local! {
    static POOL: RefCell<Vec<Workspace>> = const { RefCell::new(Vec::new()) };
}

/// A workspace on loan from this thread's pool; reset and returned on drop.
pub(crate) struct PooledWorkspace(Option<Workspace>);

impl std::ops::Deref for PooledWorkspace {
    type Target = Workspace;
    fn deref(&self) -> &Workspace {
        self.0.as_ref().expect("present until drop")
    }
}

impl std::ops::DerefMut for PooledWorkspace {
    fn deref_mut(&mut self) -> &mut Workspace {
        self.0.as_mut().expect("present until drop")
    }
}

impl Drop for PooledWorkspace {
    fn drop(&mut self) {
        if let Some(mut workspace) = self.0.take() {
            workspace.reset();
            // Ignore failure during thread teardown.
            let _ = POOL.try_with(|pool| pool.borrow_mut().push(workspace));
        }
    }
}

/// Borrow a clean workspace sized for `node_count` nodes.
pub(crate) fn workspace(node_count: usize) -> PooledWorkspace {
    let mut workspace = POOL
        .try_with(|pool| pool.borrow_mut().pop())
        .ok()
        .flatten()
        .unwrap_or_default();
    workspace.fit(node_count);
    PooledWorkspace(Some(workspace))
}

// ---------------------------------------------------------------------------
// Searches
// ---------------------------------------------------------------------------

/// One-to-many Dijkstra over `adjacency`, bounded by `max_cost` (inclusive).
///
/// The search starts from every `(node, cost)` in `sources` at once, which is
/// how a point part-way along a road enters the network. `cost(slot)` prices
/// the arc in adjacency slot `slot`. Pass the forward adjacency for
/// `cost(source → v)` and the backward one for `cost(v → source)`. The
/// result lists every reached node in settle order, i.e. sorted by cost.
pub(crate) fn dijkstra(
    adjacency: &Adjacency,
    sources: &[(u32, f64)],
    max_cost: f64,
    mut cost: impl FnMut(usize) -> f64,
) -> NodeMap<f64> {
    let node_count = adjacency.node_count();
    let mut result = NodeMap::with_node_count(node_count);
    if max_cost.is_nan() || max_cost < 0.0 {
        return result;
    }

    let mut ws = workspace(node_count);
    seed(&mut ws, sources, node_count, |_| 0.0, Some(max_cost));

    while let Some(HeapEntry {
        cost: node_cost,
        node,
        ..
    }) = ws.heap.pop()
    {
        if node_cost > ws.dist(node) || !ws.settle(node) {
            continue; // stale heap entry
        }
        result.insert(NodeIndex::new(node as usize), node_cost);

        for slot in adjacency.range(node) {
            let edge_cost = cost(slot);
            if !usable(edge_cost) {
                continue;
            }
            let next_cost = node_cost + edge_cost;
            let next = adjacency.neighbors[slot];
            if next_cost <= max_cost && ws.relax(next, next_cost, node, slot as u32) {
                ws.heap.push(HeapEntry {
                    key: next_cost,
                    cost: next_cost,
                    node: next,
                });
            }
        }
    }

    result
}

/// Push each in-range source onto the workspace heap with its starting cost.
pub(crate) fn seed(
    ws: &mut Workspace,
    sources: &[(u32, f64)],
    node_count: usize,
    mut heuristic: impl FnMut(u32) -> f64,
    max_cost: Option<f64>,
) {
    for &(node, cost) in sources {
        let in_range = (node as usize) < node_count && usable(cost);
        if in_range && max_cost.is_none_or(|max| cost <= max) && ws.relax(node, cost, NONE, NONE) {
            ws.heap.push(HeapEntry {
                key: cost + heuristic(node),
                cost,
                node,
            });
        }
    }
}

/// An optimal path found by a point-to-point search.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SearchPath {
    /// Total cost including the source and target offsets.
    pub(crate) cost: f64,
    /// The source the path leaves from and the target it arrives at.
    pub(crate) first: NodeIndex,
    pub(crate) last: NodeIndex,
    /// Petgraph edges in travel order.
    pub(crate) edges: Vec<EdgeIndex>,
}

/// Point-to-point A* over the forward adjacency `out`, from any of `sources`
/// to any of `targets`, each a `(node, offset)` pair.
///
/// `heuristic` must never overestimate the remaining cost to the goal
/// including the target offset (`|_| 0.0` gives plain Dijkstra).
pub(crate) fn astar(
    out: &Adjacency,
    sources: &[(u32, f64)],
    targets: &[(u32, f64)],
    mut cost: impl FnMut(usize) -> f64,
    mut heuristic: impl FnMut(u32) -> f64,
) -> Option<SearchPath> {
    let node_count = out.node_count();
    let mut ws = workspace(node_count);
    seed(&mut ws, sources, node_count, &mut heuristic, None);
    let target_offset = |node: u32| {
        targets
            .iter()
            .filter(|&&(t, offset)| t == node && usable(offset))
            .map(|&(_, offset)| offset)
            .min_by(f64::total_cmp)
    };

    let mut best = f64::INFINITY;
    let mut best_target = NONE;
    while let Some(HeapEntry {
        key,
        cost: node_cost,
        node,
    }) = ws.heap.pop()
    {
        if key >= best {
            break;
        }
        if node_cost > ws.dist(node) || !ws.settle(node) {
            continue; // stale heap entry
        }
        if let Some(offset) = target_offset(node) {
            if node_cost + offset < best {
                best = node_cost + offset;
                best_target = node;
            }
        }

        for slot in out.range(node) {
            let edge_cost = cost(slot);
            if !usable(edge_cost) {
                continue;
            }
            let next = out.neighbors[slot];
            let next_cost = node_cost + edge_cost;
            if next_cost < best && ws.relax(next, next_cost, node, slot as u32) {
                ws.heap.push(HeapEntry {
                    key: next_cost + heuristic(next),
                    cost: next_cost,
                    node: next,
                });
            }
        }
    }

    if best_target == NONE {
        return None;
    }
    let mut edges = Vec::new();
    let mut current = best_target;
    loop {
        let (pred, slot) = ws.pred(current);
        if pred == NONE {
            break;
        }
        edges.push(EdgeIndex::new(out.edges[slot as usize] as usize));
        current = pred;
    }
    edges.reverse();
    Some(SearchPath {
        cost: best,
        first: NodeIndex::new(current as usize),
        last: NodeIndex::new(best_target as usize),
        edges,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diamond() -> (DiGraph<(), f64>, [NodeIndex; 4]) {
        // a → b → d costs 1 + 1; a → c → d costs 5 + 0.5; a → d costs 10.
        let mut g = DiGraph::new();
        let [a, b, c, d] = [(); 4].map(|_| g.add_node(()));
        g.add_edge(a, b, 1.0);
        g.add_edge(b, d, 1.0);
        g.add_edge(a, c, 5.0);
        g.add_edge(c, d, 0.5);
        g.add_edge(a, d, 10.0);
        (g, [a, b, c, d])
    }

    fn weights(g: &DiGraph<(), f64>, adjacency: &Adjacency) -> Vec<f64> {
        adjacency
            .edges
            .iter()
            .map(|&e| g[EdgeIndex::new(e as usize)])
            .collect()
    }

    #[test]
    fn dijkstra_forward_and_backward_agree_on_pair_costs() {
        let (g, [a, b, c, d]) = diamond();
        let index = SearchIndex::new(&g, &[]);
        let (out_w, inc_w) = (weights(&g, &index.out), weights(&g, &index.inc));
        let forward = dijkstra(&index.out, &[(a.index() as u32, 0.0)], f64::INFINITY, |s| {
            out_w[s]
        });
        let backward = dijkstra(&index.inc, &[(d.index() as u32, 0.0)], f64::INFINITY, |s| {
            inc_w[s]
        });

        assert_eq!(forward.get(d), Some(&2.0));
        assert_eq!(backward.get(a), Some(&2.0));
        assert_eq!(backward.get(c), Some(&0.5));
        assert_eq!(forward.get(b), Some(&1.0));
        assert_eq!(forward.len(), 4);
        let costs: Vec<f64> = forward.values().copied().collect();
        assert!(costs.windows(2).all(|w| w[0] <= w[1]), "settle order");
    }

    #[test]
    fn dijkstra_bound_is_inclusive_and_skips_bad_costs() {
        let (g, [a, b, c, d]) = diamond();
        let index = SearchIndex::new(&g, &[]);
        let w = weights(&g, &index.out);
        let bounded = dijkstra(&index.out, &[(a.index() as u32, 0.0)], 1.0, |s| w[s]);
        assert_eq!(bounded.get(b), Some(&1.0));
        assert_eq!(bounded.get(c), None);
        assert_eq!(bounded.get(d), None);

        let nan = dijkstra(
            &index.out,
            &[(a.index() as u32, 0.0)],
            f64::INFINITY,
            |_| f64::NAN,
        );
        assert_eq!(nan.len(), 1);
    }

    #[test]
    fn astar_returns_cheapest_edge_sequence_and_pool_resets() {
        let (g, [a, b, c, d]) = diamond();
        let index = SearchIndex::new(&g, &[]);
        let w = weights(&g, &index.out);
        let at = |n: NodeIndex| [(n.index() as u32, 0.0)];
        for _ in 0..3 {
            let path = astar(&index.out, &at(a), &at(d), |s| w[s], |_| 0.0).unwrap();
            assert_eq!(path.cost, 2.0);
            assert_eq!((path.first, path.last), (a, d));
            let nodes: Vec<_> = path
                .edges
                .iter()
                .map(|&e| g.edge_endpoints(e).unwrap().1)
                .collect();
            assert_eq!(nodes, vec![b, d]);
            assert_eq!(astar(&index.out, &at(d), &at(a), |s| w[s], |_| 0.0), None);
            let same = astar(&index.out, &at(a), &at(a), |s| w[s], |_| 0.0).unwrap();
            assert_eq!((same.cost, same.edges.len()), (0.0, 0));
        }
        // Offsets: starting at c with 0.1 and arriving at b with 5 makes
        // c → d (0.5) the best way into the target set {d + 0, b + 5}.
        let path = astar(
            &index.out,
            &[(a.index() as u32, 3.0), (c.index() as u32, 0.1)],
            &[(d.index() as u32, 0.0), (b.index() as u32, 5.0)],
            |s| w[s],
            |_| 0.0,
        )
        .unwrap();
        assert!((path.cost - 0.6).abs() < 1e-12);
        assert_eq!((path.first, path.last), (c, d));
    }
}
