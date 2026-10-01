//! Network reachability primitive.
//!
//! `ReachabilityResult` is the source of truth for "what's reachable from here
//! within this travel-time budget." Both isochrone polygon construction and
//! downstream filtering (POIs, candidates, two-sided feasibility) consume this
//! same result, so a single search powers all of them.
//!
use std::collections::HashMap;

use petgraph::graph::{DiGraph, EdgeIndex, EdgeReference, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction;

use crate::graph::{SpatialGraph, XmlNode, XmlWay};
use crate::overpass::NetworkType;
use crate::search::dijkstra;

/// Result of a one-to-many shortest-path search from a single origin.
///
/// `distances` contains every node reachable within `max_cost` (inclusive),
/// keyed by `NodeIndex`, with values in seconds for the chosen `NetworkType`.
#[derive(Debug, Clone)]
pub struct ReachabilityResult {
    pub start: NodeIndex,
    pub max_cost: f64,
    pub distances: HashMap<NodeIndex, f64>,
}

/// A travel-time-labeled view of the graph reachable from an origin within a budget.
///
/// This is the graph-shaped public result for reachability. It keeps the
/// original graph and the travel-time labels, and only materializes a physical
/// induced subgraph when a constrained operation needs one.
#[derive(Clone)]
pub struct ReachableGraph {
    /// Parent graph. The reachable set is stored in `result`.
    pub graph: SpatialGraph,
    pub result: ReachabilityResult,
    pub network_type: NetworkType,
}

/// Edge context passed to a custom cost closure.
///
/// The fields are always in the *original* graph orientation regardless of
/// search direction (forward Dijkstra from origin vs reverse Dijkstra from
/// destination in [`crate::feasibility::compute_feasibility_with`]). That means
/// a closure looking up density, traffic multipliers, or anything keyed by
/// edge identity sees a consistent edge view in both directions.
#[derive(Debug, Clone, Copy)]
pub struct EdgeInfo<'a> {
    pub id: EdgeIndex,
    pub source: NodeIndex,
    pub target: NodeIndex,
    pub weight: &'a XmlWay,
}

impl<'a> From<EdgeReference<'a, XmlWay>> for EdgeInfo<'a> {
    fn from(edge: EdgeReference<'a, XmlWay>) -> Self {
        EdgeInfo {
            id: edge.id(),
            source: edge.source(),
            target: edge.target(),
            weight: edge.weight(),
        }
    }
}

/// Compute reachability with a caller-supplied edge cost.
///
/// The closure is called once per edge relaxation. Use it to inject
/// density-based traffic penalties, externally-supplied multipliers from a
/// traffic API, time-of-day adjustments, or any custom cost model. Costs must
/// be non-negative and finite or Dijkstra's invariants break.
pub fn compute_reachability_with<F>(
    graph: &DiGraph<XmlNode, XmlWay>,
    start: NodeIndex,
    max_cost: f64,
    mut cost: F,
) -> ReachabilityResult
where
    F: FnMut(EdgeInfo<'_>) -> f64,
{
    let labels = dijkstra(graph, start, max_cost, Direction::Outgoing, |edge| {
        cost(edge.into())
    });
    ReachabilityResult {
        start,
        max_cost,
        distances: labels.iter().collect(),
    }
}

/// Compute reachability from `start` up to `max_cost` seconds for the given
/// network type. Nodes with travel time greater than `max_cost` are excluded.
///
/// Convenience wrapper that uses the precomputed `walk_travel_time` /
/// `bike_travel_time` / `drive_travel_time` field on each edge. For custom
/// cost models (traffic, density penalties), use [`compute_reachability_with`].
pub fn compute_reachability(
    graph: &DiGraph<XmlNode, XmlWay>,
    start: NodeIndex,
    max_cost: f64,
    network_type: NetworkType,
) -> ReachabilityResult {
    compute_reachability_with(graph, start, max_cost, |e| {
        e.weight.travel_time(network_type)
    })
}

impl ReachableGraph {
    pub fn node_count(&self) -> usize {
        self.result.distances.len()
    }

    pub fn edge_count(&self) -> usize {
        self.graph
            .graph
            .edge_references()
            .filter(|edge| {
                self.result.distances.contains_key(&edge.source())
                    && self.result.distances.contains_key(&edge.target())
            })
            .count()
    }

    pub fn contains_node_id(&self, node_id: i64) -> bool {
        self.result
            .distances
            .keys()
            .any(|&idx| self.graph.graph[idx].id == node_id)
    }

    pub fn travel_time_to_node_id(&self, node_id: i64) -> Option<f64> {
        self.result
            .distances
            .iter()
            .find_map(|(&idx, &time)| (self.graph.graph[idx].id == node_id).then_some(time))
    }

    pub fn materialize(&self) -> SpatialGraph {
        self.graph
            .induced_subgraph(|node| self.result.distances.contains_key(&node))
    }

    pub fn route(
        &self,
        origin_lat: f64,
        origin_lon: f64,
        dest_lat: f64,
        dest_lon: f64,
        max_snap_m: Option<f64>,
    ) -> Result<crate::routing::Route, crate::error::OsmGraphError> {
        self.materialize().route(
            origin_lat,
            origin_lon,
            dest_lat,
            dest_lon,
            self.network_type,
            max_snap_m,
        )
    }

    pub fn isochrones(
        &self,
        lat: f64,
        lon: f64,
        time_limits: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> Option<Vec<geo::Polygon>> {
        self.materialize()
            .isochrones(lat, lon, time_limits, self.network_type, max_snap_m)
    }
}

impl SpatialGraph {
    /// Return the graph-shaped reachability result: a lightweight view over
    /// nodes reachable from `(lat, lon)` within `max_time`, plus travel-time
    /// labels from the origin.
    pub fn reachable_graph(
        &self,
        lat: f64,
        lon: f64,
        max_time: f64,
        network_type: NetworkType,
        max_snap_m: Option<f64>,
    ) -> Option<ReachableGraph> {
        let result = self.reachability(lat, lon, max_time, network_type, max_snap_m)?;
        Some(ReachableGraph {
            graph: self.clone(),
            result,
            network_type,
        })
    }

    /// Return every node reachable from the nearest graph node to `(lat, lon)`
    /// within `max_time` seconds, along with the travel time to each.
    ///
    /// This is the primary entry point for reachability queries. The returned
    /// [`ReachabilityResult`] can be passed directly to
    /// [`crate::isochrone::build_isochrone_polygons`] or inspected directly.
    pub fn reachability(
        &self,
        lat: f64,
        lon: f64,
        max_time: f64,
        network_type: NetworkType,
        max_snap_m: Option<f64>,
    ) -> Option<ReachabilityResult> {
        let start = self.nearest_node_within(lat, lon, max_snap_m)?;
        Some(compute_reachability(
            &self.graph,
            start,
            max_time,
            network_type,
        ))
    }

    /// Fetch POIs reachable from `(lat, lon)` within `max_time` seconds,
    /// filtered by actual network travel time.
    ///
    /// Runs a reachability search, then calls
    /// [`crate::poi::fetch_pois_within_reachability`] so that POI filtering
    /// uses graph distances rather than polygon containment. Returns `None` if
    /// no graph node is found near the origin.
    pub async fn reachable_pois(
        &self,
        lat: f64,
        lon: f64,
        max_time: f64,
        network_type: NetworkType,
    ) -> Option<Result<Vec<crate::poi::ReachablePoi>, crate::error::OsmGraphError>> {
        let result = self.reachability(lat, lon, max_time, network_type, None)?;
        Some(crate::poi::fetch_pois_within_reachability(self, &result).await)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::create_graph;
    use crate::graph::{XmlNode, XmlNodeRef, XmlTag, XmlWay};

    fn node(id: i64, lat: f64, lon: f64) -> XmlNode {
        XmlNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn way(node_ids: Vec<i64>, tags: Vec<(&str, &str)>) -> XmlWay {
        XmlWay {
            id: 1,
            nodes: node_ids
                .into_iter()
                .map(|id| XmlNodeRef { node_id: id })
                .collect(),
            tags: tags
                .into_iter()
                .map(|(k, v)| XmlTag {
                    key: k.into(),
                    value: v.into(),
                })
                .collect(),
            length: 0.0,
            speed_kph: 0.0,
            walk_travel_time: 0.0,
            bike_travel_time: 0.0,
            drive_travel_time: 0.0,
            geometry: Vec::new(),
        }
    }

    #[test]
    fn budget_excludes_distant_nodes() {
        // 3 collinear nodes ~111m apart each at the equator; residential default 30 kph.
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let g = create_graph(nodes, vec![w], true, false);

        let start = g.node_indices().find(|&i| g[i].id == 1).unwrap();
        let full = compute_reachability(&g, start, f64::INFINITY, NetworkType::Drive);
        assert_eq!(
            full.distances.len(),
            3,
            "all 3 nodes should reach with infinite budget"
        );

        // ~111m at 30 kph => ~13s. 5s budget should drop the far node.
        let tight = compute_reachability(&g, start, 5.0, NetworkType::Drive);
        assert!(
            tight.distances.len() < 3,
            "tight budget should exclude at least one node"
        );
        assert!(tight.distances.values().all(|&t| t <= 5.0));
    }

    #[test]
    fn custom_cost_closure_controls_distances() {
        // 4 collinear nodes. With a constant 10s/edge cost, distances must
        // be 0, 10, 20, 30 — independent of any precomputed travel-time field.
        let nodes = vec![
            node(1, 0.0, 0.0),
            node(2, 0.0, 0.001),
            node(3, 0.0, 0.002),
            node(4, 0.0, 0.003),
        ];
        let w = way(vec![1, 2, 3, 4], vec![("highway", "residential")]);
        let g = create_graph(nodes, vec![w], true, false);

        let start = g.node_indices().find(|&i| g[i].id == 1).unwrap();
        let result = compute_reachability_with(&g, start, 100.0, |_| 10.0);

        let mut times: Vec<f64> = result.distances.values().copied().collect();
        times.sort_by(f64::total_cmp);
        assert_eq!(times, vec![0.0, 10.0, 20.0, 30.0]);
    }

    #[test]
    fn bounded_search_does_not_insert_nodes_beyond_budget() {
        let nodes = vec![
            node(1, 0.0, 0.0),
            node(2, 0.0, 0.001),
            node(3, 0.0, 0.002),
            node(4, 0.0, 0.003),
        ];
        let w = way(vec![1, 2, 3, 4], vec![("highway", "residential")]);
        let g = create_graph(nodes, vec![w], true, false);

        let start = g.node_indices().find(|&i| g[i].id == 1).unwrap();
        let result = compute_reachability_with(&g, start, 15.0, |_| 10.0);

        let mut node_ids: Vec<i64> = result.distances.keys().map(|&idx| g[idx].id).collect();
        node_ids.sort_unstable();
        assert_eq!(node_ids, vec![1, 2]);
        assert!(result.distances.values().all(|&t| t <= 15.0));
    }

    #[test]
    fn invalid_budget_returns_empty_reachability() {
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001)];
        let w = way(vec![1, 2], vec![("highway", "residential")]);
        let g = create_graph(nodes, vec![w], true, false);

        let start = g.node_indices().find(|&i| g[i].id == 1).unwrap();
        let result = compute_reachability(&g, start, f64::NAN, NetworkType::Drive);

        assert!(result.distances.is_empty());
    }

    #[test]
    fn closure_can_double_baseline_cost() {
        // Verify the closure has access to the way and produces 2x the baseline.
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let g = create_graph(nodes, vec![w], true, false);

        let start = g.node_indices().find(|&i| g[i].id == 1).unwrap();
        let baseline = compute_reachability(&g, start, f64::INFINITY, NetworkType::Drive);
        let doubled = compute_reachability_with(&g, start, f64::INFINITY, |e| {
            e.weight.travel_time(NetworkType::Drive) * 2.0
        });

        for (node, &b) in &baseline.distances {
            let d = doubled.distances[node];
            assert!(
                (d - 2.0 * b).abs() < 1e-9,
                "node {:?}: expected 2x baseline",
                node
            );
        }
    }

    #[test]
    fn reachable_graph_view_exposes_induced_counts_and_travel_times() {
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let graph = SpatialGraph::new(create_graph(nodes, vec![w], true, false));

        let reachable = graph
            .reachable_graph(0.0, 0.0, 20.0, NetworkType::Drive, None)
            .unwrap();

        assert_eq!(reachable.node_count(), 2);
        assert_eq!(reachable.edge_count(), 2);
        assert!(reachable.contains_node_id(1));
        assert!(reachable.contains_node_id(2));
        assert!(!reachable.contains_node_id(3));
        assert_eq!(reachable.travel_time_to_node_id(1), Some(0.0));
        assert!(reachable.travel_time_to_node_id(2).unwrap() > 0.0);
        assert_eq!(reachable.graph.graph.node_count(), 3);
        assert_eq!(reachable.materialize().graph.node_count(), 2);
    }
}
