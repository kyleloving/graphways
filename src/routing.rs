//! Point-to-point routing.
//!
//! [`SpatialGraph::prepare_routing`] builds a contraction hierarchy for a
//! mode; after that, routes for that mode are answered by a bidirectional
//! search over the hierarchy in well under a millisecond on a city graph.
//! Without it, routing falls back to A* with an admissible straight-line
//! heuristic. Both return exactly optimal routes.

use petgraph::graph::{EdgeIndex, NodeIndex};

use crate::ch::ContractionHierarchy;
use crate::error::OsmGraphError;
use crate::graph::{edge_geometry, CostField, SnapResult, SpatialGraph};
use crate::overpass::NetworkType;
use crate::reachability::EdgeInfo;
use crate::search::astar;
use crate::utils::calculate_distance;

#[derive(Debug, Clone)]
pub struct Route {
    /// Ordered list of (lat, lon) coordinates along the route
    pub coordinates: Vec<(f64, f64)>,
    /// Cumulative travel time in seconds at each coordinate (parallel to `coordinates`)
    pub cumulative_times_s: Vec<f64>,
    /// Total route distance in meters
    pub distance_m: f64,
    /// Total travel time in seconds for the given network type
    pub duration_s: f64,
    /// Snap diagnostics for the requested origin coordinate.
    pub origin_snap: SnapResult,
    /// Snap diagnostics for the requested destination coordinate.
    pub destination_snap: SnapResult,
}

/// Optimal path edges for a built-in mode: through the contraction hierarchy
/// when one has been prepared, otherwise A* guided by the straight-line
/// distance at the fastest speed any edge allows (admissible, so still exact).
fn shortest_path_edges(
    sg: &SpatialGraph,
    origin: NodeIndex,
    dest: NodeIndex,
    network_type: NetworkType,
) -> Option<Vec<EdgeIndex>> {
    let field = CostField::of(network_type);
    if let Some(hierarchy) = sg.hierarchy_slot(field).get() {
        return hierarchy
            .shortest_path(origin, dest)
            .map(|(_, edges)| edges);
    }

    let speed = sg.max_straight_line_speed(field);
    let goal = &sg.graph[dest];
    let remaining_lower_bound = |node: u32| {
        if !(speed.is_finite() && speed > 0.0) {
            return 0.0;
        }
        let n = &sg.graph[NodeIndex::new(node as usize)];
        calculate_distance(n.lat, n.lon, goal.lat, goal.lon) / speed
    };
    let costs = &sg.slot_costs(field).out;
    astar(
        &sg.search_index().out,
        origin,
        dest,
        |slot| costs[slot],
        remaining_lower_bound,
    )
    .map(|(_, edges)| edges)
}

/// Route coordinates with travel time interpolated along each edge's shape
/// in proportion to segment length.
struct RouteGeometry {
    coordinates: Vec<(f64, f64)>,
    cumulative_times_s: Vec<f64>,
    distance_m: f64,
    duration_s: f64,
}

fn route_geometry_and_times(
    sg: &SpatialGraph,
    origin: NodeIndex,
    edges: &[EdgeIndex],
    mut edge_time_of: impl FnMut(EdgeIndex) -> f64,
) -> RouteGeometry {
    let mut out = RouteGeometry {
        coordinates: Vec::new(),
        cumulative_times_s: Vec::new(),
        distance_m: 0.0,
        duration_s: 0.0,
    };
    if edges.is_empty() {
        let node = &sg.graph[origin];
        out.coordinates.push((node.lat, node.lon));
        out.cumulative_times_s.push(0.0);
        return out;
    }

    let mut segment_lengths = Vec::new();
    for &edge in edges {
        let way = &sg.graph[edge];
        let geometry = edge_geometry(&sg.graph, edge);
        let edge_time = edge_time_of(edge);
        let edge_start_time = out.duration_s;

        segment_lengths.clear();
        segment_lengths.extend(
            geometry
                .points()
                .zip(geometry.points().skip(1))
                .map(|(a, b)| calculate_distance(a.0, a.1, b.0, b.1)),
        );
        let geometry_length: f64 = segment_lengths.iter().sum();
        let evenly_split = edge_time / (segment_lengths.len().max(1) as f64);

        let mut points = geometry.points();
        let first = points
            .next()
            .expect("edge geometry has at least two points");
        if out.coordinates.is_empty() {
            out.coordinates.push(first);
            out.cumulative_times_s.push(edge_start_time);
        }

        let mut elapsed_on_edge = 0.0;
        for (point, &segment_len) in points.zip(&segment_lengths) {
            elapsed_on_edge += if geometry_length > 0.0 {
                edge_time * (segment_len / geometry_length)
            } else {
                evenly_split
            };
            out.coordinates.push(point);
            out.cumulative_times_s
                .push(edge_start_time + elapsed_on_edge);
        }

        out.distance_m += way.length;
        out.duration_s += edge_time;
        // Pin each edge's last timestamp to the exact running total.
        if let Some(last) = out.cumulative_times_s.last_mut() {
            *last = out.duration_s;
        }
    }

    out
}

/// Snap both endpoints, enforcing `max_snap_m` when given.
fn snap_endpoints(
    sg: &SpatialGraph,
    origin: (f64, f64),
    destination: (f64, f64),
    max_snap_m: Option<f64>,
) -> Result<(SnapResult, SnapResult), OsmGraphError> {
    let origin_snap = sg
        .snap_point(origin.0, origin.1)
        .ok_or(OsmGraphError::OriginNodeNotFound)?;
    let destination_snap = sg
        .snap_point(destination.0, destination.1)
        .ok_or(OsmGraphError::DestinationNodeNotFound)?;
    if let Some(max_distance_m) = max_snap_m {
        for (role, snap) in [("origin", origin_snap), ("destination", destination_snap)] {
            if snap.distance_m > max_distance_m {
                return Err(OsmGraphError::SnapDistanceExceeded {
                    role,
                    distance_m: snap.distance_m,
                    max_distance_m,
                });
            }
        }
    }
    Ok((origin_snap, destination_snap))
}

fn assemble(
    sg: &SpatialGraph,
    (origin_snap, destination_snap): (SnapResult, SnapResult),
    edges: Option<Vec<EdgeIndex>>,
    edge_time_of: impl FnMut(EdgeIndex) -> f64,
) -> Result<Route, OsmGraphError> {
    let edges = edges.ok_or(OsmGraphError::PathNotFound)?;
    let geometry = route_geometry_and_times(sg, origin_snap.node_index, &edges, edge_time_of);
    Ok(Route {
        coordinates: geometry.coordinates,
        cumulative_times_s: geometry.cumulative_times_s,
        distance_m: geometry.distance_m,
        duration_s: geometry.duration_s,
        origin_snap,
        destination_snap,
    })
}

pub fn route(
    sg: &SpatialGraph,
    origin_lat: f64,
    origin_lon: f64,
    dest_lat: f64,
    dest_lon: f64,
    network_type: NetworkType,
    max_snap_m: Option<f64>,
) -> Result<Route, OsmGraphError> {
    let snaps = snap_endpoints(
        sg,
        (origin_lat, origin_lon),
        (dest_lat, dest_lon),
        max_snap_m,
    )?;
    let edges = shortest_path_edges(sg, snaps.0.node_index, snaps.1.node_index, network_type);
    assemble(sg, snaps, edges, |edge| {
        sg.graph[edge].travel_time(network_type)
    })
}

impl SpatialGraph {
    /// Preprocess this graph for fast routing in `network_type`.
    ///
    /// Builds a contraction hierarchy for the mode's travel times (a few
    /// seconds on a city graph; once per mode, shared by every clone of this
    /// graph). Afterwards [`SpatialGraph::route`] for that mode answers in
    /// well under a millisecond. Routing works without it, just more slowly.
    pub fn prepare_routing(&self, network_type: NetworkType) {
        let field = CostField::of(network_type);
        self.hierarchy_slot(field).get_or_init(|| {
            ContractionHierarchy::build(self.search_index(), &self.slot_costs(field).out)
        });
    }

    /// Whether [`SpatialGraph::prepare_routing`] has run for `network_type`'s
    /// travel times.
    pub fn is_routing_prepared(&self, network_type: NetworkType) -> bool {
        self.hierarchy_slot(CostField::of(network_type))
            .get()
            .is_some()
    }

    /// Find the fastest route between two lat/lon points.
    ///
    /// Snaps both points to the nearest graph nodes and finds an optimal
    /// path, through the contraction hierarchy if
    /// [`SpatialGraph::prepare_routing`] has run for this mode and with A*
    /// otherwise. Returns [`OsmGraphError::OriginNodeNotFound`] or
    /// [`OsmGraphError::DestinationNodeNotFound`] if snapping fails, and
    /// [`OsmGraphError::PathNotFound`] if the snapped nodes are disconnected.
    pub fn route(
        &self,
        origin_lat: f64,
        origin_lon: f64,
        dest_lat: f64,
        dest_lon: f64,
        network_type: NetworkType,
        max_snap_m: Option<f64>,
    ) -> Result<Route, OsmGraphError> {
        route(
            self,
            origin_lat,
            origin_lon,
            dest_lat,
            dest_lon,
            network_type,
            max_snap_m,
        )
    }

    /// Find the cheapest route under a caller-supplied edge cost, e.g. live
    /// traffic or a penalty on certain road classes.
    ///
    /// The returned durations are in the closure's units. Costs that are
    /// negative, NaN or infinite make an edge impassable. Arbitrary costs rule
    /// out precomputation and distance bounds, so this runs a plain Dijkstra
    /// search; prefer [`SpatialGraph::route`] for the built-in travel times.
    pub fn route_with<F>(
        &self,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
        mut cost: F,
    ) -> Result<Route, OsmGraphError>
    where
        F: FnMut(EdgeInfo<'_>) -> f64,
    {
        let snaps = snap_endpoints(self, origin, destination, max_snap_m)?;
        let out = &self.search_index().out;
        let edges = astar(
            out,
            snaps.0.node_index,
            snaps.1.node_index,
            |slot| cost(EdgeInfo::of(&self.graph, out.edges[slot])),
            |_| 0.0,
        )
        .map(|(_, edges)| edges);
        assemble(self, snaps, edges, |edge| {
            cost(EdgeInfo::of(&self.graph, edge.index() as u32))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Edge, SpatialGraph, XmlNode, XmlTag};
    use crate::overpass::NetworkType;
    use petgraph::graph::DiGraph;

    fn make_node(id: i64, lat: f64, lon: f64) -> XmlNode {
        XmlNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn make_way(drive_travel_time: f64, length: f64) -> Edge {
        Edge {
            way_id: 1,
            tags: vec![XmlTag {
                key: "highway".into(),
                value: "residential".into(),
            }]
            .into(),
            length,
            speed_kph: 50.0,
            walk_travel_time: length / (5.0 / 3.6),
            bike_travel_time: length / (15.0 / 3.6),
            drive_travel_time,
            geometry: Vec::new(),
        }
    }

    fn make_profile_way(drive_travel_time: f64, walk_travel_time: f64, length: f64) -> Edge {
        Edge {
            way_id: 1,
            tags: vec![XmlTag {
                key: "highway".into(),
                value: "residential".into(),
            }]
            .into(),
            length,
            speed_kph: 50.0,
            walk_travel_time,
            bike_travel_time: walk_travel_time,
            drive_travel_time,
            geometry: Vec::new(),
        }
    }

    fn make_way_with_geometry(
        drive_travel_time: f64,
        length: f64,
        geometry: Vec<(f64, f64)>,
    ) -> Edge {
        Edge {
            geometry,
            ..make_way(drive_travel_time, length)
        }
    }

    fn linear_graph() -> SpatialGraph {
        // A → B → C along a straight line
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.002, 0.0));
        g.add_edge(a, b, make_way(10.0, 111.0));
        g.add_edge(b, c, make_way(10.0, 111.0));
        SpatialGraph::new(g)
    }

    #[test]
    fn test_cumulative_times_starts_at_zero() {
        let sg = linear_graph();
        let r = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();
        assert_eq!(r.cumulative_times_s[0], 0.0);
    }

    #[test]
    fn test_cumulative_times_parallel_to_coordinates() {
        let sg = linear_graph();
        let r = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();
        assert_eq!(r.cumulative_times_s.len(), r.coordinates.len());
    }

    #[test]
    fn test_cumulative_times_monotonic() {
        let sg = linear_graph();
        let r = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();
        for w in r.cumulative_times_s.windows(2) {
            assert!(w[1] >= w[0], "times decreased: {:?}", r.cumulative_times_s);
        }
    }

    #[test]
    fn test_cumulative_times_last_equals_duration() {
        let sg = linear_graph();
        let r = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();
        let last = *r.cumulative_times_s.last().unwrap();
        assert!(
            (last - r.duration_s).abs() < 1e-6,
            "last cumulative time {last:.6} != duration {:.6}",
            r.duration_s
        );
    }

    #[test]
    fn test_route_chooses_faster_path_not_fewer_edges() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.002, 0.0));
        g.add_edge(a, c, make_way(100.0, 100.0));
        g.add_edge(a, b, make_way(10.0, 50.0));
        g.add_edge(b, c, make_way(10.0, 50.0));
        let sg = SpatialGraph::new(g);

        let route = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();

        assert_eq!(route.coordinates.len(), 3);
        assert_eq!(route.duration_s, 20.0);
        assert_eq!(route.distance_m, 100.0);
    }

    #[test]
    fn test_route_totals_use_selected_parallel_edge() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.002, 0.0));
        g.add_edge(a, b, make_way(100.0, 1_000.0));
        g.add_edge(a, b, make_way(10.0, 50.0));
        g.add_edge(b, c, make_way(10.0, 50.0));
        let sg = SpatialGraph::new(g);

        let route = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();

        assert_eq!(route.duration_s, 20.0);
        assert_eq!(route.distance_m, 100.0);
    }

    #[test]
    fn test_route_uses_edge_geometry_between_nodes() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        g.add_edge(
            a,
            b,
            make_way_with_geometry(
                10.0,
                100.0,
                vec![(0.0, 0.0), (0.0005, 0.0002), (0.001, 0.0)],
            ),
        );
        let sg = SpatialGraph::new(g);

        let route = route(&sg, 0.0, 0.0, 0.001, 0.0, NetworkType::Drive, None).unwrap();

        assert_eq!(
            route.coordinates,
            vec![(0.0, 0.0), (0.0005, 0.0002), (0.001, 0.0)]
        );
        assert_eq!(route.cumulative_times_s.len(), route.coordinates.len());
        assert_eq!(*route.cumulative_times_s.last().unwrap(), route.duration_s);
    }

    #[test]
    fn test_route_oneway_succeeds_forward_and_fails_reverse() {
        let sg = linear_graph();

        let forward = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None);
        let reverse = route(&sg, 0.002, 0.0, 0.0, 0.0, NetworkType::Drive, None);

        assert!(forward.is_ok());
        assert!(matches!(reverse, Err(OsmGraphError::PathNotFound)));
    }

    #[test]
    fn test_route_origin_equals_destination_is_zero_cost() {
        let sg = linear_graph();

        let route = route(&sg, 0.0, 0.0, 0.0, 0.0, NetworkType::Drive, None).unwrap();

        assert_eq!(route.coordinates.len(), 1);
        assert_eq!(route.duration_s, 0.0);
        assert_eq!(route.distance_m, 0.0);
        assert_eq!(route.cumulative_times_s, vec![0.0]);
    }

    #[test]
    fn test_route_uses_network_specific_costs() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.002, 0.0));
        g.add_edge(a, c, make_profile_way(10.0, 100.0, 100.0));
        g.add_edge(a, b, make_profile_way(30.0, 5.0, 50.0));
        g.add_edge(b, c, make_profile_way(30.0, 5.0, 50.0));
        let sg = SpatialGraph::new(g);

        let drive = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, None).unwrap();
        let walk = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Walk, None).unwrap();

        assert_eq!(drive.coordinates.len(), 2);
        assert_eq!(drive.duration_s, 10.0);
        assert_eq!(walk.coordinates.len(), 3);
        assert_eq!(walk.duration_s, 10.0);
    }

    #[test]
    fn test_route_disconnected_components_return_path_not_found() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 1.0, 1.0));
        let d = g.add_node(make_node(4, 1.001, 1.0));
        g.add_edge(a, b, make_way(10.0, 100.0));
        g.add_edge(c, d, make_way(10.0, 100.0));
        let sg = SpatialGraph::new(g);

        let result = route(&sg, 0.0, 0.0, 1.001, 1.0, NetworkType::Drive, None);

        assert!(matches!(result, Err(OsmGraphError::PathNotFound)));
    }

    #[test]
    fn astar_and_ch_costs_match_dijkstra_on_fixture() {
        use crate::reachability::compute_reachability;

        for network_type in [NetworkType::Walk, NetworkType::Drive] {
            let sg = SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", network_type, None)
                .unwrap();
            let prepared = sg.clone();
            prepared.prepare_routing(network_type);
            assert!(prepared.is_routing_prepared(network_type));
            assert!(
                sg.is_routing_prepared(network_type),
                "clones share the hierarchy"
            );
            let unprepared = SpatialGraph::new((*sg.graph).clone());

            let path_cost = |edges: Vec<EdgeIndex>| {
                edges
                    .iter()
                    .map(|&e| sg.graph[e].travel_time(network_type))
                    .sum::<f64>()
            };
            for origin in sg.graph.node_indices() {
                let exact = compute_reachability(&sg, origin, f64::INFINITY, network_type).times;
                for dest in sg.graph.node_indices() {
                    let want = exact.get(dest).copied();
                    for graph in [&prepared, &unprepared] {
                        let got =
                            shortest_path_edges(graph, origin, dest, network_type).map(path_cost);
                        match (got, want) {
                            (Some(a), Some(d)) => assert!((a - d).abs() < 1e-9, "{a} vs {d}"),
                            (None, None) => {}
                            other => panic!("search disagrees with Dijkstra: {other:?}"),
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn route_with_uses_custom_costs() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.002, 0.0));
        g.add_edge(a, c, make_way(10.0, 100.0));
        g.add_edge(a, b, make_way(30.0, 50.0));
        g.add_edge(b, c, make_way(30.0, 50.0));
        let sg = SpatialGraph::new(g);

        // Penalise the direct edge tenfold: the two-hop path wins.
        let route = sg
            .route_with((0.0, 0.0), (0.002, 0.0), None, |e| {
                let base = e.weight.drive_travel_time;
                if e.source == a && e.target == c {
                    base * 10.0
                } else {
                    base
                }
            })
            .unwrap();

        assert_eq!(route.coordinates.len(), 3);
        assert_eq!(route.duration_s, 60.0);
    }

    #[test]
    fn test_route_respects_max_snap_distance() {
        let sg = linear_graph();

        let close = route(&sg, 0.0, 0.0, 0.002, 0.0, NetworkType::Drive, Some(1.0));
        assert!(close.is_ok());

        let far = route(&sg, 0.0, 0.0005, 0.002, 0.0, NetworkType::Drive, Some(1.0));
        assert!(matches!(
            far,
            Err(OsmGraphError::SnapDistanceExceeded { role: "origin", .. })
        ));
    }
}
