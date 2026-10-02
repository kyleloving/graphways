//! Network reachability primitive.
//!
//! `ReachabilityResult` is the source of truth for "what's reachable from here
//! within this travel-time budget." Both isochrone polygon construction and
//! downstream filtering (POIs, candidates, two-sided feasibility) consume this
//! same result, so a single search powers all of them.

use petgraph::graph::{EdgeIndex, NodeIndex};

use crate::error::OsmGraphError;
use crate::graph::{seeds, Edge, LatLon, NodeMap, RoadGraph, Role, SnapResult, SpatialGraph};
use crate::search::dijkstra;

/// Result of a one-to-many shortest-path search from a single origin.
///
/// `times` holds every node reachable within `max_cost` (inclusive) with its
/// travel time in seconds, in increasing order of time.
#[derive(Debug, Clone)]
pub struct ReachabilityResult {
    /// The snapped origin point on the road network.
    pub origin: LatLon,
    pub max_cost: f64,
    pub times: NodeMap<f64>,
    /// Per search-state times, kept only when turn restrictions split some
    /// junctions into several states (see [`ReachabilityResult::time_to`]).
    pub(crate) state_times: Option<std::sync::Arc<NodeMap<f64>>>,
}

impl ReachabilityResult {
    /// A result from node travel times, e.g. computed elsewhere.
    pub fn new(origin: LatLon, max_cost: f64, times: NodeMap<f64>) -> Self {
        Self {
            origin,
            max_cost,
            times,
            state_times: None,
        }
    }

    fn from_states(sg: &SpatialGraph, origin: LatLon, max_cost: f64, states: NodeMap<f64>) -> Self {
        let index = sg.search_index();
        if index.has_restricted_states() {
            Self {
                origin,
                max_cost,
                times: index.fold_states(states.clone()),
                state_times: Some(std::sync::Arc::new(states)),
            }
        } else {
            Self::new(origin, max_cost, states)
        }
    }

    /// Travel time to `snap`, a point snapped onto the same graph: the best
    /// way of finishing along its road from a reached node, if within budget.
    /// Respects turn restrictions on the way onto that road.
    pub fn time_to(&self, sg: &SpatialGraph, snap: &SnapResult) -> Option<f64> {
        let nt = sg.network_type();
        let arrivals = sg.arrivals(snap, &mut |e| sg.graph[e].travel_time(nt));
        let best = match &self.state_times {
            Some(states) => sg
                .arrival_roots(&arrivals)
                .iter()
                .filter_map(|&(state, cost, _)| {
                    Some(states.get(NodeIndex::new(state as usize))? + cost)
                })
                .min_by(f64::total_cmp),
            None => arrivals
                .iter()
                .filter_map(|a| Some(self.times.get(a.node)? + a.cost))
                .min_by(f64::total_cmp),
        };
        best.filter(|&t| t <= self.max_cost)
    }
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
    pub weight: &'a Edge,
}

impl<'a> EdgeInfo<'a> {
    #[inline]
    pub(crate) fn of(graph: &'a RoadGraph, edge: u32) -> Self {
        let raw = &graph.raw_edges()[edge as usize];
        EdgeInfo {
            id: EdgeIndex::new(edge as usize),
            source: raw.source(),
            target: raw.target(),
            weight: &raw.weight,
        }
    }
}

/// Compute reachability from `origin` with a caller-supplied edge cost.
///
/// The closure is called once per edge relaxation. Use it to inject
/// density-based traffic penalties, externally-supplied multipliers from a
/// traffic API, time-of-day adjustments, or any custom cost model. Costs that
/// are negative, NaN or infinite make an edge impassable.
pub fn compute_reachability_with<F>(
    sg: &SpatialGraph,
    origin: &SnapResult,
    max_cost: f64,
    mut cost: F,
) -> ReachabilityResult
where
    F: FnMut(EdgeInfo<'_>) -> f64,
{
    let departures = sg.departures(origin, &mut |e| {
        cost(EdgeInfo::of(&sg.graph, e.index() as u32))
    });
    let sources = seeds(&sg.departure_roots(&departures));
    let out = &sg.search_index().out;
    let states = dijkstra(out, &sources, max_cost, |slot| {
        cost(EdgeInfo::of(&sg.graph, out.edges[slot]))
    });
    ReachabilityResult::from_states(sg, origin.snapped(), max_cost, states)
}

/// Compute reachability from `origin` up to `max_cost` seconds using the
/// graph's travel times. Nodes with travel time greater than `max_cost` are
/// excluded. For custom cost models, use [`compute_reachability_with`].
pub fn compute_reachability(
    sg: &SpatialGraph,
    origin: &SnapResult,
    max_cost: f64,
) -> ReachabilityResult {
    let nt = sg.network_type();
    let departures = sg.departures(origin, &mut |e| sg.graph[e].travel_time(nt));
    let sources = seeds(&sg.departure_roots(&departures));
    let costs = &sg.slot_costs().out;
    let states = dijkstra(&sg.search_index().out, &sources, max_cost, |slot| {
        costs[slot]
    });
    ReachabilityResult::from_states(sg, origin.snapped(), max_cost, states)
}

/// Number of edges of `sg` with both endpoints in `nodes`.
pub(crate) fn induced_edge_count<T>(sg: &SpatialGraph, nodes: &NodeMap<T>) -> usize {
    nodes
        .keys()
        .map(|node| {
            sg.graph
                .neighbors(node)
                .filter(|&next| nodes.contains_key(next))
                .count()
        })
        .sum()
}

impl ReachableGraph {
    pub fn node_count(&self) -> usize {
        self.result.times.len()
    }

    /// Number of directed edges between reachable nodes.
    pub fn edge_count(&self) -> usize {
        induced_edge_count(&self.graph, &self.result.times)
    }

    pub fn contains_node_id(&self, node_id: i64) -> bool {
        self.travel_time_to_node_id(node_id).is_some()
    }

    pub fn travel_time_to_node_id(&self, node_id: i64) -> Option<f64> {
        let node = self.graph.node_index(node_id)?;
        self.result.times.get(node).copied()
    }

    pub fn materialize(&self) -> SpatialGraph {
        self.graph
            .induced_subgraph(|node| self.result.times.contains_key(node))
    }

    /// A route that stays within the reachable subgraph.
    pub fn route(
        &self,
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        max_snap_m: Option<f64>,
    ) -> Result<crate::routing::Route, OsmGraphError> {
        self.materialize().route(origin, destination, max_snap_m)
    }

    /// Isochrones computed within the reachable subgraph.
    pub fn isochrones(
        &self,
        origin: impl Into<LatLon>,
        time_limits: &[f64],
        max_snap_m: Option<f64>,
    ) -> Result<Vec<geo::MultiPolygon>, OsmGraphError> {
        self.materialize()
            .isochrones(origin, time_limits, max_snap_m)
    }
}

impl SpatialGraph {
    /// Return the graph-shaped reachability result: a lightweight view over
    /// nodes reachable from `origin` within `max_time`, plus travel-time
    /// labels from the origin.
    pub fn reachable_graph(
        &self,
        origin: impl Into<LatLon>,
        max_time: f64,
        max_snap_m: Option<f64>,
    ) -> Result<ReachableGraph, OsmGraphError> {
        Ok(ReachableGraph {
            graph: self.clone(),
            result: self.reachability(origin, max_time, max_snap_m)?,
        })
    }

    /// Return every node reachable from `origin` (snapped to the nearest
    /// road) within `max_time` seconds, along with the travel time to each.
    ///
    /// This is the primary entry point for reachability queries. The returned
    /// [`ReachabilityResult`] can be passed directly to
    /// [`crate::isochrone::build_isochrone_polygons`] or inspected directly.
    pub fn reachability(
        &self,
        origin: impl Into<LatLon>,
        max_time: f64,
        max_snap_m: Option<f64>,
    ) -> Result<ReachabilityResult, OsmGraphError> {
        let snap = self.snap_endpoint(origin.into(), Role::Origin, max_snap_m)?;
        Ok(compute_reachability(self, &snap, max_time))
    }

    /// Fetch POIs reachable from `origin` within `max_time` seconds,
    /// filtered by actual network travel time.
    ///
    /// Runs a reachability search, then filters POIs from Overpass by the
    /// travel time to the road point each one snaps to, rather than by
    /// polygon containment.
    pub async fn reachable_pois(
        &self,
        origin: impl Into<LatLon>,
        max_time: f64,
    ) -> Result<Vec<crate::poi::ReachablePoi>, OsmGraphError> {
        let result = self.reachability(origin, max_time, None)?;
        crate::poi::fetch_pois_within_reachability(self, &result).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::create_graph;
    use crate::graph::{OsmNode, OsmNodeRef, OsmTag, OsmWay};
    use crate::overpass::NetworkType;

    fn spatial(g: RoadGraph) -> SpatialGraph {
        SpatialGraph::new(g, NetworkType::Drive)
    }

    fn node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn way(node_ids: Vec<i64>, tags: Vec<(&str, &str)>) -> OsmWay {
        OsmWay {
            id: 1,
            nodes: node_ids
                .into_iter()
                .map(|id| OsmNodeRef { node_id: id })
                .collect(),
            tags: tags
                .into_iter()
                .map(|(k, v)| OsmTag {
                    key: k.into(),
                    value: v.into(),
                })
                .collect(),
        }
    }

    #[test]
    fn budget_excludes_distant_nodes() {
        // 3 collinear nodes ~111m apart each at the equator; residential default 30 kph.
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let g = spatial(create_graph(nodes, vec![w], true, false));

        let start = g.snap_to_node(g.node_index(1).unwrap());
        let full = compute_reachability(&g, &start, f64::INFINITY);
        assert_eq!(
            full.times.len(),
            3,
            "all 3 nodes should reach with infinite budget"
        );

        // ~111m at 30 kph => ~13s. 5s budget should drop the far node.
        let tight = compute_reachability(&g, &start, 5.0);
        assert!(
            tight.times.len() < 3,
            "tight budget should exclude at least one node"
        );
        assert!(tight.times.values().all(|&t| t <= 5.0));
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
        let g = spatial(create_graph(nodes, vec![w], true, false));

        let start = g.snap_to_node(g.node_index(1).unwrap());
        let result = compute_reachability_with(&g, &start, 100.0, |_| 10.0);

        let mut times: Vec<f64> = result.times.values().copied().collect();
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
        let g = spatial(create_graph(nodes, vec![w], true, false));

        let start = g.snap_to_node(g.node_index(1).unwrap());
        let result = compute_reachability_with(&g, &start, 15.0, |_| 10.0);

        let mut node_ids: Vec<i64> = result.times.keys().map(|idx| g.graph[idx].id).collect();
        node_ids.sort_unstable();
        assert_eq!(node_ids, vec![1, 2]);
        assert!(result.times.values().all(|&t| t <= 15.0));
    }

    #[test]
    fn invalid_budget_returns_empty_reachability() {
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001)];
        let w = way(vec![1, 2], vec![("highway", "residential")]);
        let g = spatial(create_graph(nodes, vec![w], true, false));

        let start = g.snap_to_node(g.node_index(1).unwrap());
        let result = compute_reachability(&g, &start, f64::NAN);

        assert!(result.times.is_empty());
    }

    #[test]
    fn closure_can_double_baseline_cost() {
        // Verify the closure has access to the way and produces 2x the baseline.
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let g = spatial(create_graph(nodes, vec![w], true, false));

        let start = g.snap_to_node(g.node_index(1).unwrap());
        let baseline = compute_reachability(&g, &start, f64::INFINITY);
        let doubled = compute_reachability_with(&g, &start, f64::INFINITY, |e| {
            e.weight.travel_time(NetworkType::Drive) * 2.0
        });

        for (node, &b) in &baseline.times {
            let d = doubled.times[*node];
            assert!(
                (d - 2.0 * b).abs() < 1e-9,
                "node {:?}: expected 2x baseline",
                node
            );
        }
    }

    #[test]
    fn times_are_in_increasing_order() {
        let nodes = vec![
            node(1, 0.0, 0.0),
            node(2, 0.0, 0.001),
            node(3, 0.0, 0.002),
            node(4, 0.0, 0.003),
        ];
        let w = way(vec![1, 2, 3, 4], vec![("highway", "residential")]);
        let g = spatial(create_graph(nodes, vec![w], true, false));
        let result = compute_reachability(&g, &g.snap_to_node(g.node_index(3).unwrap()), 1e9);

        let times: Vec<f64> = result.times.values().copied().collect();
        assert_eq!(times.len(), 4);
        assert!(times.windows(2).all(|w| w[0] <= w[1]));
    }

    #[test]
    fn reachable_graph_view_exposes_induced_counts_and_travel_times() {
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let graph = SpatialGraph::new(
            create_graph(nodes, vec![w], true, false),
            NetworkType::Drive,
        );

        let reachable = graph.reachable_graph((0.0, 0.0), 20.0, None).unwrap();

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

    #[test]
    fn origin_mid_road_starts_part_way_along_it() {
        // Two-way residential street 1 → 2 → 3, ~111 m per edge at 30 km/h.
        let nodes = vec![node(1, 0.0, 0.0), node(2, 0.0, 0.001), node(3, 0.0, 0.002)];
        let w = way(vec![1, 2, 3], vec![("highway", "residential")]);
        let g = SpatialGraph::new(
            create_graph(nodes, vec![w], true, false),
            NetworkType::Drive,
        );
        let edge_time = g.graph.edge_weights().next().unwrap().drive_travel_time;

        // A quarter of the way from node 1 to node 2.
        let result = g.reachability((0.0, 0.00025), f64::INFINITY, None).unwrap();

        let at = |id| result.times[g.node_index(id).unwrap()];
        assert!((at(1) - 0.25 * edge_time).abs() < 1e-9);
        assert!((at(2) - 0.75 * edge_time).abs() < 1e-9);
        assert!((at(3) - 1.75 * edge_time).abs() < 1e-9);

        // And the time back to a point on the road counts the partial edge.
        let destination = g.snap_point((0.0, 0.0015)).unwrap();
        let time = result.time_to(&g, &destination).unwrap();
        assert!((time - 1.25 * edge_time).abs() < 1e-9, "{time}");
    }
}
