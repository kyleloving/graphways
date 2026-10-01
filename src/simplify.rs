//! Graph simplification: merge intersection nodes that sit within a few
//! metres of each other, then collapse chains of degree-two nodes into single
//! edges that keep the original road geometry.
//!
//! All per-node bookkeeping uses `Vec`s indexed by `NodeIndex` instead of hash
//! maps, and edge weights are moved rather than cloned wherever the old graph
//! is no longer needed.

use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction::{Incoming, Outgoing};
use rstar::RTree;

use crate::graph::{edge_geometry, NodeEntry, XmlNode, XmlWay};
use crate::utils::calculate_distance;

const CONSOLIDATION_DISTANCE_M: f64 = 5.0;

type RoadGraph = DiGraph<XmlNode, XmlWay>;

pub fn simplify_graph(graph: RoadGraph) -> RoadGraph {
    let (graph, _) = consolidate_intersections(graph, CONSOLIDATION_DISTANCE_M);

    let is_endpoint: Vec<bool> = graph
        .node_indices()
        .map(|node| is_endpoint(&graph, node))
        .collect();
    let endpoints = || graph.node_indices().filter(|n| is_endpoint[n.index()]);

    let mut simplified = DiGraph::new();
    let mut new_index = vec![NodeIndex::end(); graph.node_count()];
    for node in endpoints() {
        new_index[node.index()] = simplified.add_node(graph[node].clone());
    }

    let mut chain: Vec<EdgeIndex> = Vec::new();
    for start in endpoints() {
        for first in graph.edges(start) {
            chain.clear();
            chain.push(first.id());
            let end = follow_chain(&graph, &is_endpoint, start, first.target(), &mut chain);
            if is_endpoint[end.index()] {
                let way = collapse_path_edges(&graph, &chain);
                let (source, target) = (new_index[start.index()], new_index[end.index()]);
                add_or_keep_fastest(&mut simplified, source, target, way);
            }
        }
    }

    simplified
}

/// Add `way` from `source` to `target` unless an edge between the same pair
/// is already at least as fast; a slower existing edge is replaced in place.
///
/// Every ordered node pair therefore carries at most one edge, the fastest by
/// drive time (the first one seen on ties). Road nodes have a handful of
/// edges, so the adjacency scan in `find_edge` beats a hash map here.
fn add_or_keep_fastest(graph: &mut RoadGraph, source: NodeIndex, target: NodeIndex, way: XmlWay) {
    match graph.find_edge(source, target) {
        None => {
            graph.add_edge(source, target, way);
        }
        Some(existing) => {
            let kept = &mut graph[existing];
            if way.drive_travel_time < kept.drive_travel_time {
                *kept = way;
            }
        }
    }
}

/// Walk forward from `current` (entered from `prev`) through degree-two nodes,
/// appending traversed edges to `chain`. Returns the node where the walk
/// stopped: an endpoint, or a node with no unambiguous way forward.
fn follow_chain(
    graph: &RoadGraph,
    is_endpoint: &[bool],
    mut prev: NodeIndex,
    mut current: NodeIndex,
    chain: &mut Vec<EdgeIndex>,
) -> NodeIndex {
    while !is_endpoint[current.index()] {
        let Some((next, edge)) = next_chain_step(graph, current, prev) else {
            break;
        };
        chain.push(edge);
        prev = current;
        current = next;
    }
    current
}

/// The single onward neighbour of `current` (excluding `prev`) and the fastest
/// edge to it, or `None` when there are zero or several distinct neighbours.
fn next_chain_step(
    graph: &RoadGraph,
    current: NodeIndex,
    prev: NodeIndex,
) -> Option<(NodeIndex, EdgeIndex)> {
    let mut step: Option<(NodeIndex, EdgeIndex)> = None;
    for edge in graph.edges(current) {
        let target = edge.target();
        if target == prev {
            continue;
        }
        match step {
            None => step = Some((target, edge.id())),
            Some((seen, best)) if seen == target => {
                if edge.weight().drive_travel_time < graph[best].drive_travel_time {
                    step = Some((target, edge.id()));
                }
            }
            Some(_) => return None,
        }
    }
    step
}

fn collapse_path_edges(graph: &RoadGraph, edges: &[EdgeIndex]) -> XmlWay {
    let first = &graph[edges[0]];
    let mut way = XmlWay {
        id: first.id,
        nodes: Vec::new(),
        tags: first.tags.clone(),
        length: 0.0,
        speed_kph: 0.0,
        walk_travel_time: 0.0,
        bike_travel_time: 0.0,
        drive_travel_time: 0.0,
        geometry: Vec::new(),
    };
    let mut weighted_speed_sum = 0.0;

    for &edge in edges {
        let part = &graph[edge];
        way.length += part.length;
        way.walk_travel_time += part.walk_travel_time;
        way.bike_travel_time += part.bike_travel_time;
        way.drive_travel_time += part.drive_travel_time;
        weighted_speed_sum += part.speed_kph * part.length;

        // Consecutive edges share their joint point; keep it only once.
        let skip = usize::from(!way.geometry.is_empty());
        way.geometry
            .extend(edge_geometry(graph, edge).points().skip(skip));
    }

    way.speed_kph = if way.length > 0.0 {
        weighted_speed_sum / way.length
    } else {
        0.0
    };
    way
}

/// A node survives simplification unless it is the interior of a chain:
/// it has both in- and out-edges, no self-loop, and exactly two distinct
/// neighbours.
fn is_endpoint(graph: &RoadGraph, node: NodeIndex) -> bool {
    let mut neighbours = [NodeIndex::end(); 2];
    let mut distinct = 0;
    let (mut has_out, mut has_in) = (false, false);

    let outgoing = graph.neighbors_directed(node, Outgoing).map(|n| (n, true));
    let incoming = graph.neighbors_directed(node, Incoming).map(|n| (n, false));
    for (neighbour, is_out) in outgoing.chain(incoming) {
        if is_out {
            has_out = true;
        } else {
            has_in = true;
        }
        if neighbour == node {
            return true; // self-loop
        }
        if !neighbours[..distinct].contains(&neighbour) {
            if distinct == neighbours.len() {
                return true; // three or more distinct neighbours: a junction
            }
            neighbours[distinct] = neighbour;
            distinct += 1;
        }
    }

    !(has_out && has_in) || distinct != 2
}

/// Merge nodes within `merge_distance_m` of a cluster seed into one node.
///
/// Returns the consolidated graph and, for every old node index, the index of
/// the node it was merged into. Edges are moved, not cloned; an edge whose
/// endpoints land in the same cluster is dropped, and parallel edges between
/// two clusters are reduced to the fastest.
fn consolidate_intersections(
    graph: RoadGraph,
    merge_distance_m: f64,
) -> (RoadGraph, Vec<NodeIndex>) {
    let tree = RTree::bulk_load(
        graph
            .node_indices()
            .map(|index| NodeEntry::new(&graph[index], index))
            .collect(),
    );
    let clusters = cluster_nodes_by_distance(&graph, &tree, merge_distance_m);

    let mut new_graph = DiGraph::with_capacity(clusters.len(), graph.edge_count());
    let mut old_to_new = vec![NodeIndex::end(); graph.node_count()];
    for members in &clusters {
        let new_idx = new_graph.add_node(merge_nodes(&graph, members));
        for &old_idx in members {
            old_to_new[old_idx.index()] = new_idx;
        }
    }

    let (_, edges) = graph.into_nodes_edges();
    for edge in edges {
        let new_src = old_to_new[edge.source().index()];
        let new_dst = old_to_new[edge.target().index()];
        if new_src != new_dst {
            add_or_keep_fastest(&mut new_graph, new_src, new_dst, edge.weight);
        }
    }

    (new_graph, old_to_new)
}

/// Greedy clustering in node order: each unassigned node seeds a cluster of
/// all unassigned nodes within `merge_distance_m` of it.
fn cluster_nodes_by_distance(
    graph: &RoadGraph,
    tree: &RTree<NodeEntry>,
    merge_distance_m: f64,
) -> Vec<Vec<NodeIndex>> {
    let mut clusters = Vec::new();
    let mut assigned = vec![false; graph.node_count()];

    for idx in graph.node_indices() {
        if assigned[idx.index()] {
            continue;
        }
        let node = &graph[idx];
        let center = NodeEntry::new(node, idx).point;
        let members: Vec<NodeIndex> = tree
            .locate_within_distance(center, merge_distance_m * merge_distance_m)
            .filter(|entry| !assigned[entry.index.index()])
            .filter(|entry| {
                let candidate = &graph[entry.index];
                calculate_distance(node.lat, node.lon, candidate.lat, candidate.lon)
                    <= merge_distance_m
            })
            .map(|entry| entry.index)
            .collect();

        for member in &members {
            assigned[member.index()] = true;
        }
        clusters.push(members);
    }

    clusters
}

/// A lone node is kept as-is (OSM id and tags intact). A real cluster becomes
/// one untagged node at the members' mean position, identified by the
/// smallest member OSM id so ids stay stable and meaningful across builds.
fn merge_nodes(graph: &RoadGraph, indices: &[NodeIndex]) -> XmlNode {
    if let [single] = indices {
        return graph[*single].clone();
    }
    let count = indices.len() as f64;
    let members = || indices.iter().map(|&i| &graph[i]);
    XmlNode {
        id: members().map(|n| n.id).min().unwrap_or_default(),
        lat: members().map(|n| n.lat).sum::<f64>() / count,
        lon: members().map(|n| n.lon).sum::<f64>() / count,
        tags: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{XmlNode, XmlTag, XmlWay};

    fn make_node(id: i64, lat: f64, lon: f64) -> XmlNode {
        XmlNode {
            id,
            lat,
            lon,
            tags: Vec::new(),
        }
    }

    fn make_tag(key: &str, value: &str) -> XmlTag {
        XmlTag {
            key: key.into(),
            value: value.into(),
        }
    }

    fn make_way(id: i64, drive_travel_time: f64) -> XmlWay {
        XmlWay {
            id,
            nodes: Vec::new(),
            tags: Vec::new(),
            length: 100.0,
            speed_kph: 50.0,
            walk_travel_time: 72.0,
            bike_travel_time: 24.0,
            drive_travel_time,
            geometry: Vec::new(),
        }
    }

    fn make_way_with_length(id: i64, drive_travel_time: f64, length: f64) -> XmlWay {
        XmlWay {
            length,
            ..make_way(id, drive_travel_time)
        }
    }

    fn make_way_with_geometry(
        id: i64,
        drive_travel_time: f64,
        geometry: Vec<(f64, f64)>,
    ) -> XmlWay {
        XmlWay {
            geometry,
            ..make_way(id, drive_travel_time)
        }
    }

    #[test]
    fn test_deduplicate_keeps_fastest_edge() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        graph.add_edge(a, b, make_way(1, 100.0));
        graph.add_edge(a, b, make_way(2, 50.0));

        assert_eq!(graph.edge_count(), 2);
        let deduped = simplify_graph(graph);
        assert!(
            deduped.edge_count() <= 1,
            "Expected at most 1 edge, got {}",
            deduped.edge_count()
        );
    }

    #[test]
    fn consolidation_does_not_merge_nodes_beyond_threshold() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 38.0, -77.0));
        let b = graph.add_node(make_node(2, 38.0001, -77.0));
        graph.add_edge(a, b, make_way(1, 1.0));

        let (consolidated, map) = consolidate_intersections(graph, 5.0);

        assert_eq!(consolidated.node_count(), 2);
        assert_ne!(map[a.index()], map[b.index()]);
    }

    #[test]
    fn merged_nodes_drop_source_tags() {
        let mut graph = DiGraph::new();
        let mut n1 = make_node(1, 38.0, -77.0);
        n1.tags.push(make_tag("highway", "traffic_signals"));
        let a = graph.add_node(n1);
        let b = graph.add_node(make_node(2, 38.000001, -77.0));

        let merged = merge_nodes(&graph, &[a, b]);

        assert!(merged.tags.is_empty());
    }

    #[test]
    fn path_aggregation_uses_traversed_edge() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        let c = graph.add_node(make_node(3, 0.002, 0.0));
        let e1 = graph.add_edge(a, b, make_way(1, 10.0));
        graph.add_edge(a, b, make_way(2, 100.0));
        let e2 = graph.add_edge(b, c, make_way(3, 20.0));
        graph.add_edge(b, c, make_way(4, 200.0));

        let collapsed = collapse_path_edges(&graph, &[e1, e2]);

        assert_eq!(collapsed.drive_travel_time, 30.0);
    }

    #[test]
    fn linear_chain_collapses_to_single_summed_edge() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        let c = graph.add_node(make_node(3, 0.002, 0.0));
        let d = graph.add_node(make_node(4, 0.003, 0.0));
        graph.add_edge(a, b, make_way_with_length(1, 10.0, 100.0));
        graph.add_edge(b, c, make_way_with_length(2, 20.0, 200.0));
        graph.add_edge(c, d, make_way_with_length(3, 30.0, 300.0));

        let simplified = simplify_graph(graph);

        assert_eq!(simplified.node_count(), 2);
        assert_eq!(simplified.edge_count(), 1);
        let edge = simplified.edge_weights().next().unwrap();
        assert_eq!(edge.drive_travel_time, 60.0);
        assert_eq!(edge.length, 600.0);
    }

    #[test]
    fn linear_chain_preserves_intermediate_geometry() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        let c = graph.add_node(make_node(3, 0.002, 0.0));
        let d = graph.add_node(make_node(4, 0.003, 0.0));
        graph.add_edge(
            a,
            b,
            make_way_with_geometry(1, 10.0, vec![(0.0, 0.0), (0.0005, 0.0002), (0.001, 0.0)]),
        );
        graph.add_edge(
            b,
            c,
            make_way_with_geometry(2, 20.0, vec![(0.001, 0.0), (0.0015, 0.0002), (0.002, 0.0)]),
        );
        graph.add_edge(
            c,
            d,
            make_way_with_geometry(3, 30.0, vec![(0.002, 0.0), (0.0025, 0.0002), (0.003, 0.0)]),
        );

        let simplified = simplify_graph(graph);
        let edge = simplified.edge_weights().next().unwrap();

        assert_eq!(
            edge.geometry,
            vec![
                (0.0, 0.0),
                (0.0005, 0.0002),
                (0.001, 0.0),
                (0.0015, 0.0002),
                (0.002, 0.0),
                (0.0025, 0.0002),
                (0.003, 0.0),
            ]
        );
    }

    #[test]
    fn faster_direct_edge_beats_slower_parallel_chain() {
        // a → b directly (10 s) and a → x → b via a detour (2 × 50 s). The
        // chain is visited first but must not shadow the faster direct edge.
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.002, 0.0));
        let x = graph.add_node(make_node(3, 0.001, 0.001));
        graph.add_edge(a, b, make_way(1, 10.0));
        graph.add_edge(x, b, make_way(2, 50.0));
        graph.add_edge(a, x, make_way(3, 50.0));

        let simplified = simplify_graph(graph);

        assert_eq!(simplified.edge_count(), 1);
        assert_eq!(
            simplified.edge_weights().next().unwrap().drive_travel_time,
            10.0
        );
    }

    #[test]
    fn consolidation_keeps_fastest_parallel_edge() {
        // b1 and b2 are 0.1 m apart and merge; both connect to a.
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b1 = graph.add_node(make_node(2, 0.001, 0.0));
        let b2 = graph.add_node(make_node(3, 0.001000001, 0.0));
        graph.add_edge(a, b1, make_way(1, 30.0));
        graph.add_edge(a, b2, make_way(2, 20.0));

        let (consolidated, _) = consolidate_intersections(graph, 5.0);

        assert_eq!(consolidated.edge_count(), 1);
        assert_eq!(
            consolidated
                .edge_weights()
                .next()
                .unwrap()
                .drive_travel_time,
            20.0
        );
    }

    #[test]
    fn simplification_keeps_osm_ids_and_merges_to_smallest_member_id() {
        let mut graph = DiGraph::new();
        let mut tagged = make_node(10, 0.0, 0.0);
        tagged.tags.push(make_tag("highway", "traffic_signals"));
        let a = graph.add_node(tagged);
        let b1 = graph.add_node(make_node(30, 0.001, 0.0));
        let b2 = graph.add_node(make_node(20, 0.001000001, 0.0));
        let c = graph.add_node(make_node(40, 0.002, 0.0));
        let d = graph.add_node(make_node(50, 0.001, 0.001));
        // A T-junction at b, whose two OSM nodes sit 0.1 m apart.
        graph.add_edge(a, b1, make_way(1, 10.0));
        graph.add_edge(b2, c, make_way(2, 10.0));
        graph.add_edge(b1, d, make_way(3, 10.0));

        let simplified = simplify_graph(graph);
        let mut ids: Vec<i64> = simplified.node_weights().map(|n| n.id).collect();
        ids.sort_unstable();

        // a, c, d keep their ids; b1/b2 merge into one node with id 20.
        assert_eq!(ids, vec![10, 20, 40, 50]);
        let a_node = simplified.node_weights().find(|n| n.id == 10).unwrap();
        assert_eq!(a_node.tags.len(), 1, "unmerged nodes keep their tags");
    }

    #[test]
    fn t_junction_preserves_decision_node() {
        let mut graph = DiGraph::new();
        let west = graph.add_node(make_node(1, 0.0, 0.0));
        let center = graph.add_node(make_node(2, 0.001, 0.0));
        let east = graph.add_node(make_node(3, 0.002, 0.0));
        let north = graph.add_node(make_node(4, 0.001, 0.001));
        graph.add_edge(west, center, make_way(1, 10.0));
        graph.add_edge(center, east, make_way(2, 10.0));
        graph.add_edge(center, north, make_way(3, 10.0));

        let simplified = simplify_graph(graph);

        assert_eq!(simplified.node_count(), 4);
        assert_eq!(simplified.edge_count(), 3);
    }

    #[test]
    fn simplification_preserves_oneway_direction() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        let c = graph.add_node(make_node(3, 0.002, 0.0));
        graph.add_edge(a, b, make_way(1, 10.0));
        graph.add_edge(b, c, make_way(2, 10.0));

        let simplified = simplify_graph(graph);
        let edge = simplified.edge_references().next().unwrap();
        let source = &simplified[edge.source()];
        let target = &simplified[edge.target()];

        assert_eq!(simplified.edge_count(), 1);
        assert!(source.lat < target.lat);
    }

    #[test]
    fn simplification_does_not_connect_near_crossing_roads() {
        let mut graph = DiGraph::new();
        let west = graph.add_node(make_node(1, 0.0, -0.001));
        let east = graph.add_node(make_node(2, 0.0, 0.001));
        let south = graph.add_node(make_node(3, -0.001, 0.0));
        let north = graph.add_node(make_node(4, 0.001, 0.0));
        graph.add_edge(west, east, make_way(1, 10.0));
        graph.add_edge(south, north, make_way(2, 10.0));

        let simplified = simplify_graph(graph);

        assert_eq!(simplified.node_count(), 4);
        assert_eq!(simplified.edge_count(), 2);
    }
}
