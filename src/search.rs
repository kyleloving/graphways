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
use std::collections::BinaryHeap;
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
pub(crate) struct SearchIndex {
    /// Arcs grouped by source; neighbour = target.
    pub(crate) out: Adjacency,
    /// Arcs grouped by target; neighbour = source.
    pub(crate) inc: Adjacency,
    costs: [OnceLock<SlotCosts>; 3],
}

impl SearchIndex {
    pub(crate) fn new<N, E>(graph: &DiGraph<N, E>) -> Self {
        let raw = graph.raw_edges();
        let mut arcs: Vec<(u32, u32, u32)> = raw
            .iter()
            .enumerate()
            .map(|(id, e)| {
                (
                    e.source().index() as u32,
                    e.target().index() as u32,
                    id as u32,
                )
            })
            .collect();
        let out = Adjacency::build(graph.node_count(), &arcs);
        for arc in &mut arcs {
            *arc = (arc.1, arc.0, arc.2);
        }
        let inc = Adjacency::build(graph.node_count(), &arcs);
        Self {
            out,
            inc,
            costs: Default::default(),
        }
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
/// `cost(slot)` prices the arc in adjacency slot `slot`. Pass the forward
/// adjacency for `cost(start → v)` and the backward one for `cost(v → start)`.
/// The result lists every reached node in settle order, i.e. sorted by cost.
pub(crate) fn dijkstra(
    adjacency: &Adjacency,
    start: NodeIndex,
    max_cost: f64,
    mut cost: impl FnMut(usize) -> f64,
) -> NodeMap<f64> {
    let node_count = adjacency.node_count();
    let mut result = NodeMap::with_node_count(node_count);
    if start.index() >= node_count || max_cost.is_nan() || max_cost < 0.0 {
        return result;
    }

    let mut ws = workspace(node_count);
    let start = start.index() as u32;
    ws.relax(start, 0.0, NONE, NONE);
    ws.heap.push(HeapEntry {
        key: 0.0,
        cost: 0.0,
        node: start,
    });

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

/// Point-to-point A* over the forward adjacency `out`.
///
/// `heuristic` must never overestimate the remaining cost to `goal`
/// (`|_| 0.0` gives plain Dijkstra). Returns the total cost and the petgraph
/// edges of an optimal path in travel order.
pub(crate) fn astar(
    out: &Adjacency,
    start: NodeIndex,
    goal: NodeIndex,
    mut cost: impl FnMut(usize) -> f64,
    mut heuristic: impl FnMut(u32) -> f64,
) -> Option<(f64, Vec<EdgeIndex>)> {
    let node_count = out.node_count();
    if start.index() >= node_count || goal.index() >= node_count {
        return None;
    }

    let mut ws = workspace(node_count);
    let (start, goal) = (start.index() as u32, goal.index() as u32);
    ws.relax(start, 0.0, NONE, NONE);
    ws.heap.push(HeapEntry {
        key: heuristic(start),
        cost: 0.0,
        node: start,
    });

    while let Some(HeapEntry {
        cost: node_cost,
        node,
        ..
    }) = ws.heap.pop()
    {
        if node_cost > ws.dist(node) || !ws.settle(node) {
            continue; // stale heap entry
        }
        if node == goal {
            let mut edges = Vec::new();
            let mut current = goal;
            while current != start {
                let (pred, slot) = ws.pred(current);
                edges.push(EdgeIndex::new(out.edges[slot as usize] as usize));
                current = pred;
            }
            edges.reverse();
            return Some((node_cost, edges));
        }

        for slot in out.range(node) {
            let edge_cost = cost(slot);
            if !usable(edge_cost) {
                continue;
            }
            let next = out.neighbors[slot];
            let next_cost = node_cost + edge_cost;
            if ws.relax(next, next_cost, node, slot as u32) {
                ws.heap.push(HeapEntry {
                    key: next_cost + heuristic(next),
                    cost: next_cost,
                    node: next,
                });
            }
        }
    }

    None
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
        let index = SearchIndex::new(&g);
        let (out_w, inc_w) = (weights(&g, &index.out), weights(&g, &index.inc));
        let forward = dijkstra(&index.out, a, f64::INFINITY, |s| out_w[s]);
        let backward = dijkstra(&index.inc, d, f64::INFINITY, |s| inc_w[s]);

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
        let index = SearchIndex::new(&g);
        let w = weights(&g, &index.out);
        let bounded = dijkstra(&index.out, a, 1.0, |s| w[s]);
        assert_eq!(bounded.get(b), Some(&1.0));
        assert_eq!(bounded.get(c), None);
        assert_eq!(bounded.get(d), None);

        let nan = dijkstra(&index.out, a, f64::INFINITY, |_| f64::NAN);
        assert_eq!(nan.len(), 1);
    }

    #[test]
    fn astar_returns_cheapest_edge_sequence_and_pool_resets() {
        let (g, [a, b, _, d]) = diamond();
        let index = SearchIndex::new(&g);
        let w = weights(&g, &index.out);
        for _ in 0..3 {
            let (cost, edges) = astar(&index.out, a, d, |s| w[s], |_| 0.0).unwrap();
            assert_eq!(cost, 2.0);
            let nodes: Vec<_> = edges
                .iter()
                .map(|&e| g.edge_endpoints(e).unwrap().1)
                .collect();
            assert_eq!(nodes, vec![b, d]);
            assert_eq!(astar(&index.out, d, a, |s| w[s], |_| 0.0), None);
            assert_eq!(
                astar(&index.out, a, a, |s| w[s], |_| 0.0),
                Some((0.0, vec![]))
            );
        }
    }
}
