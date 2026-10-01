//! Shortest-path kernels shared by routing, reachability and feasibility.
//!
//! All searches label nodes through dense `Vec`s indexed by `NodeIndex`
//! rather than `HashMap`s. Graph node indices are contiguous, so this turns
//! every relaxation into a bounds-checked array access instead of a hash and
//! probe, which is where most of the time in a city-scale search used to go.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use petgraph::graph::{DiGraph, EdgeIndex, EdgeReference, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction;

/// Binary-heap entry that pops the *smallest* `key` first.
///
/// Only `key` participates in the ordering; NaN keys never enter the heap
/// because edge costs are validated before they are pushed.
#[derive(Clone, Copy, Debug)]
struct MinScored<T> {
    key: f64,
    item: T,
}

impl<T> PartialEq for MinScored<T> {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key
    }
}

impl<T> Eq for MinScored<T> {}

impl<T> PartialOrd for MinScored<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for MinScored<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.key.total_cmp(&self.key)
    }
}

/// Dijkstra and A* treat non-finite or negative edge costs as impassable.
#[inline]
fn usable(cost: f64) -> bool {
    cost.is_finite() && cost >= 0.0
}

/// Travel-time labels produced by [`dijkstra`].
pub(crate) struct Labels {
    /// Cost from the search root, `f64::INFINITY` when unreached.
    dist: Vec<f64>,
    /// Every node with a finite label, in the order it was first reached.
    reached: Vec<NodeIndex>,
}

impl Labels {
    #[inline]
    pub(crate) fn get(&self, node: NodeIndex) -> Option<f64> {
        self.dist
            .get(node.index())
            .copied()
            .filter(|cost| cost.is_finite())
    }

    /// Reached nodes and their final cost, in discovery order.
    pub(crate) fn iter(&self) -> impl ExactSizeIterator<Item = (NodeIndex, f64)> + '_ {
        self.reached
            .iter()
            .map(|&node| (node, self.dist[node.index()]))
    }
}

/// One-to-many Dijkstra bounded by `max_cost` (inclusive).
///
/// `direction` selects which way edges are followed: `Outgoing` computes
/// `cost(start → v)`, `Incoming` computes `cost(v → start)` without building
/// a reversed graph. In both cases `cost` receives the edge in its original
/// orientation.
pub(crate) fn dijkstra<N, E, F>(
    graph: &DiGraph<N, E>,
    start: NodeIndex,
    max_cost: f64,
    direction: Direction,
    mut cost: F,
) -> Labels
where
    F: FnMut(EdgeReference<'_, E>) -> f64,
{
    let mut labels = Labels {
        dist: vec![f64::INFINITY; graph.node_count()],
        reached: Vec::new(),
    };
    if start.index() >= graph.node_count() || max_cost.is_nan() || max_cost < 0.0 {
        return labels;
    }

    let mut heap = BinaryHeap::new();
    labels.dist[start.index()] = 0.0;
    labels.reached.push(start);
    heap.push(MinScored {
        key: 0.0,
        item: start,
    });

    while let Some(MinScored {
        key: node_cost,
        item: node,
    }) = heap.pop()
    {
        if node_cost > max_cost {
            break;
        }
        if node_cost > labels.dist[node.index()] {
            continue; // stale heap entry
        }

        for edge in graph.edges_directed(node, direction) {
            let edge_cost = cost(edge);
            if !usable(edge_cost) {
                continue;
            }
            let next_cost = node_cost + edge_cost;
            if next_cost > max_cost {
                continue;
            }
            let next = match direction {
                Direction::Outgoing => edge.target(),
                Direction::Incoming => edge.source(),
            };
            let slot = &mut labels.dist[next.index()];
            if next_cost < *slot {
                if slot.is_infinite() {
                    labels.reached.push(next);
                }
                *slot = next_cost;
                heap.push(MinScored {
                    key: next_cost,
                    item: next,
                });
            }
        }
    }

    labels
}

/// Point-to-point A* search.
///
/// `heuristic` must never overestimate the remaining cost to `goal`
/// (pass `|_| 0.0` for plain Dijkstra). Returns the total cost and the edges
/// of the optimal path in travel order.
pub(crate) fn astar<N, E, F, H>(
    graph: &DiGraph<N, E>,
    start: NodeIndex,
    goal: NodeIndex,
    mut cost: F,
    mut heuristic: H,
) -> Option<(f64, Vec<EdgeIndex>)>
where
    F: FnMut(EdgeReference<'_, E>) -> f64,
    H: FnMut(NodeIndex) -> f64,
{
    let node_count = graph.node_count();
    if start.index() >= node_count || goal.index() >= node_count {
        return None;
    }

    let mut best = vec![f64::INFINITY; node_count];
    let mut predecessor = vec![EdgeIndex::end(); node_count];
    let mut heap = BinaryHeap::new();

    best[start.index()] = 0.0;
    heap.push(MinScored {
        key: heuristic(start),
        item: (0.0, start),
    });

    while let Some(MinScored {
        item: (node_cost, node),
        ..
    }) = heap.pop()
    {
        if node_cost > best[node.index()] {
            continue; // stale heap entry
        }
        if node == goal {
            return Some((node_cost, unwind(graph, &predecessor, start, goal)));
        }

        for edge in graph.edges(node) {
            let edge_cost = cost(edge);
            if !usable(edge_cost) {
                continue;
            }
            let next = edge.target();
            let next_cost = node_cost + edge_cost;
            if next_cost < best[next.index()] {
                best[next.index()] = next_cost;
                predecessor[next.index()] = edge.id();
                heap.push(MinScored {
                    key: next_cost + heuristic(next),
                    item: (next_cost, next),
                });
            }
        }
    }

    None
}

fn unwind<N, E>(
    graph: &DiGraph<N, E>,
    predecessor: &[EdgeIndex],
    start: NodeIndex,
    goal: NodeIndex,
) -> Vec<EdgeIndex> {
    let mut edges = Vec::new();
    let mut current = goal;
    while current != start {
        let edge = predecessor[current.index()];
        edges.push(edge);
        current = graph
            .edge_endpoints(edge)
            .expect("predecessor edges belong to the searched graph")
            .0;
    }
    edges.reverse();
    edges
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

    #[test]
    fn dijkstra_forward_and_backward_agree_on_pair_costs() {
        let (g, [a, b, c, d]) = diamond();
        let forward = dijkstra(&g, a, f64::INFINITY, Direction::Outgoing, |e| *e.weight());
        let backward = dijkstra(&g, d, f64::INFINITY, Direction::Incoming, |e| *e.weight());

        assert_eq!(forward.get(d), Some(2.0));
        assert_eq!(backward.get(a), Some(2.0));
        assert_eq!(backward.get(c), Some(0.5));
        assert_eq!(forward.get(b), Some(1.0));
        assert_eq!(forward.iter().len(), 4);
    }

    #[test]
    fn dijkstra_bound_is_inclusive_and_skips_bad_costs() {
        let (g, [a, b, c, d]) = diamond();
        let bounded = dijkstra(&g, a, 1.0, Direction::Outgoing, |e| *e.weight());
        assert_eq!(bounded.get(b), Some(1.0));
        assert_eq!(bounded.get(c), None);
        assert_eq!(bounded.get(d), None);

        let nan = dijkstra(&g, a, f64::INFINITY, Direction::Outgoing, |_| f64::NAN);
        assert_eq!(nan.iter().len(), 1);
    }

    #[test]
    fn astar_returns_cheapest_edge_sequence() {
        let (g, [a, b, _, d]) = diamond();
        let (cost, edges) = astar(&g, a, d, |e| *e.weight(), |_| 0.0).unwrap();
        assert_eq!(cost, 2.0);
        let nodes: Vec<_> = edges
            .iter()
            .map(|&e| g.edge_endpoints(e).unwrap().1)
            .collect();
        assert_eq!(nodes, vec![b, d]);
        assert_eq!(astar(&g, d, a, |e| *e.weight(), |_| 0.0), None);
        assert_eq!(
            astar(&g, a, a, |e| *e.weight(), |_| 0.0),
            Some((0.0, vec![]))
        );
    }
}
