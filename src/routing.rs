//! Point-to-point routing.
//!
//! Both endpoints snap to the closest point on a road, so routes start and
//! end part-way along edges. [`SpatialGraph::prepare_routing`] builds a
//! contraction hierarchy; after that, routes are answered by a bidirectional
//! search over the hierarchy in well under a millisecond on a city graph.
//! Without it, routing falls back to A* with an admissible straight-line
//! heuristic. Both return exactly optimal routes.

use petgraph::graph::EdgeIndex;

use crate::ch::ContractionHierarchy;
use crate::error::OsmGraphError;
use crate::graph::{anchor_at, seeds, LatLon, Piece, Pricing, Role, SnapResult, SpatialGraph};
use crate::reachability::EdgeInfo;
use crate::search::{astar, SearchPath};

#[derive(Debug, Clone)]
pub struct Route {
    /// Ordered list of (lat, lon) coordinates along the route, from the
    /// snapped origin to the snapped destination.
    pub coordinates: Vec<(f64, f64)>,
    /// Cumulative travel time in seconds at each coordinate (parallel to `coordinates`)
    pub cumulative_times_s: Vec<f64>,
    /// Total route distance in meters
    pub distance_m: f64,
    /// Total travel time in seconds for the graph's network type
    pub duration_s: f64,
    /// The stretches of road travelled, in order.
    pub pieces: Vec<Piece>,
    /// Snap diagnostics for the requested origin coordinate.
    pub origin_snap: SnapResult,
    /// Snap diagnostics for the requested destination coordinate.
    pub destination_snap: SnapResult,
}

/// The cheapest sequence of pieces from `origin` to `destination` under
/// `cost`, using `search` to cross the network between anchors.
pub(crate) fn best_pieces(
    sg: &SpatialGraph,
    origin: &SnapResult,
    destination: &SnapResult,
    cost: &mut dyn FnMut(EdgeIndex) -> f64,
    pricing: Pricing,
    search: impl FnOnce(
        &[(u32, f64)],
        &[(u32, f64)],
        &mut dyn FnMut(EdgeIndex) -> f64,
    ) -> Option<SearchPath>,
) -> Option<(f64, Vec<Piece>)> {
    let departures = sg.departures(origin, cost, pricing);
    let arrivals = sg.arrivals(destination, cost, pricing);
    let direct = sg.direct_piece(origin, destination, cost, pricing);
    let (from, to) = (sg.departure_roots(&departures), sg.arrival_roots(&arrivals));

    let via_network = search(&seeds(&from), &seeds(&to), cost).map(|path| {
        let mut pieces = Vec::with_capacity(path.edges.len() + 2);
        pieces.extend(anchor_at(&departures, &from, path.first).and_then(|a| a.piece));
        pieces.extend(path.edges.iter().map(|&e| Piece::whole(e)));
        pieces.extend(anchor_at(&arrivals, &to, path.last).and_then(|a| a.piece));
        (path.cost, pieces)
    });
    match (direct, via_network) {
        (Some((d, piece)), Some((n, _))) if d <= n => Some((d, vec![piece])),
        (Some((d, piece)), None) => Some((d, vec![piece])),
        (_, network) => network,
    }
}

/// Optimal pieces under the graph's own travel times: through the
/// contraction hierarchy when one has been prepared, otherwise A* guided by
/// the straight-line distance at the fastest speed any edge allows.
fn shortest_pieces(
    sg: &SpatialGraph,
    origin: &SnapResult,
    destination: &SnapResult,
) -> Option<(f64, Vec<Piece>)> {
    let nt = sg.network_type();
    let mut cost = |e: EdgeIndex| sg.graph[e].travel_time(nt);
    best_pieces(
        sg,
        origin,
        destination,
        &mut cost,
        Pricing::Native,
        |sources, targets, _| {
            if let Some(hierarchy) = sg.hierarchy_slot().get() {
                return hierarchy.shortest_path(sources, targets);
            }
            let speed = sg.max_straight_line_speed();
            let goal = destination.snapped();
            let remaining_lower_bound = |state: u32| {
                if !(speed.is_finite() && speed > 0.0) {
                    return 0.0;
                }
                sg.state_point(state).distance_m(goal) / speed
            };
            let costs = &sg.slot_costs().out;
            astar(
                &sg.search_index().out,
                sources,
                targets,
                |slot| costs[slot],
                remaining_lower_bound,
            )
        },
    )
}

/// Route coordinates with travel time interpolated along each piece's shape
/// in proportion to segment length. Turn costs and signal waits (seconds) are
/// part of the graph's own travel times only (`Pricing::Native`).
fn assemble(
    sg: &SpatialGraph,
    (origin_snap, destination_snap): (SnapResult, SnapResult),
    pieces: Vec<Piece>,
    cost: &mut dyn FnMut(EdgeIndex) -> f64,
    pricing: Pricing,
) -> Route {
    let mut coordinates = Vec::new();
    let mut cumulative_times_s = Vec::new();
    let mut distance_m = 0.0;
    let mut duration_s = 0.0;
    let mut segment_lengths = Vec::new();

    let mut previous: Option<EdgeIndex> = None;
    for piece in &pieces {
        // Time lost turning onto this piece at the junction it starts from,
        // counted even for an empty piece: the search priced that turn too.
        if let Some(into) = previous.replace(piece.edge) {
            if pricing == Pricing::Native {
                duration_s += sg.turn_cost(into, piece.edge);
            }
        }
        if piece.share() <= 0.0 {
            continue;
        }
        let points = piece.points(&sg.graph);
        let piece_time = sg.piece_cost(piece, cost(piece.edge), pricing);
        // A signal wait the piece reaches is spent at its end, not spread
        // along the way.
        let wait = if piece.to >= 1.0 {
            sg.end_wait(piece.edge, pricing)
        } else {
            0.0
        };
        let moving_time = piece_time - wait;
        let piece_start_time = duration_s;

        segment_lengths.clear();
        segment_lengths.extend(
            points
                .windows(2)
                .map(|w| LatLon::from(w[0]).distance_m(LatLon::from(w[1]))),
        );
        let geometry_length: f64 = segment_lengths.iter().sum();
        let evenly_split = moving_time / (segment_lengths.len().max(1) as f64);

        if coordinates.is_empty() {
            coordinates.push(points[0]);
            cumulative_times_s.push(piece_start_time);
        }
        let mut elapsed = 0.0;
        for (&point, &segment_len) in points[1..].iter().zip(&segment_lengths) {
            elapsed += if geometry_length > 0.0 {
                moving_time * (segment_len / geometry_length)
            } else {
                evenly_split
            };
            coordinates.push(point);
            cumulative_times_s.push(piece_start_time + elapsed);
        }

        distance_m += sg.graph[piece.edge].length * piece.share();
        duration_s += piece_time;
        // Pin each piece's last timestamp to the exact running total.
        if let Some(last) = cumulative_times_s.last_mut() {
            *last = duration_s;
        }
    }

    if let Some(last) = cumulative_times_s.last_mut() {
        *last = duration_s;
    }
    if coordinates.is_empty() {
        // Origin and destination snapped to the same point.
        coordinates.push((origin_snap.snapped_lat, origin_snap.snapped_lon));
        cumulative_times_s.push(0.0);
    }
    Route {
        coordinates,
        cumulative_times_s,
        distance_m,
        duration_s,
        pieces,
        origin_snap,
        destination_snap,
    }
}

fn snap_both(
    sg: &SpatialGraph,
    origin: LatLon,
    destination: LatLon,
    max_snap_m: Option<f64>,
) -> Result<(SnapResult, SnapResult), OsmGraphError> {
    Ok((
        sg.snap_endpoint(origin, Role::Origin, max_snap_m)?,
        sg.snap_endpoint(destination, Role::Destination, max_snap_m)?,
    ))
}

impl SpatialGraph {
    /// Preprocess this graph for fast routing.
    ///
    /// Builds a contraction hierarchy for the graph's travel times (a few
    /// seconds on a city walking graph, using all cores; shared by every
    /// clone of this graph). Afterwards [`SpatialGraph::route`] answers in
    /// well under a millisecond. Routing works without it, just more slowly.
    pub fn prepare_routing(&self) {
        self.hierarchy_slot().get_or_init(|| {
            ContractionHierarchy::build(self.search_index(), &self.slot_costs().out, |edge| {
                self.graph.raw_edges()[edge as usize].weight.length
            })
        });
    }

    /// Whether [`SpatialGraph::prepare_routing`] has run.
    pub fn is_routing_prepared(&self) -> bool {
        self.hierarchy_slot().get().is_some()
    }

    /// Find the fastest route between two points.
    ///
    /// Both points snap to the nearest road; `max_snap_m` rejects points
    /// farther than that from any road. Routes are exactly optimal, through
    /// the contraction hierarchy if [`SpatialGraph::prepare_routing`] has run
    /// and with A* otherwise. Errors with
    /// [`OsmGraphError::SnapDistanceExceeded`] for a rejected snap and
    /// [`OsmGraphError::PathNotFound`] when no route exists.
    pub fn route(
        &self,
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        max_snap_m: Option<f64>,
    ) -> Result<Route, OsmGraphError> {
        let snaps = snap_both(self, origin.into(), destination.into(), max_snap_m)?;
        let (_, pieces) =
            shortest_pieces(self, &snaps.0, &snaps.1).ok_or(OsmGraphError::PathNotFound)?;
        let nt = self.network_type();
        Ok(assemble(
            self,
            snaps,
            pieces,
            &mut |e| self.graph[e].travel_time(nt),
            Pricing::Native,
        ))
    }

    /// Find the cheapest route under a caller-supplied edge cost, e.g. live
    /// traffic or a penalty on certain road classes.
    ///
    /// The returned durations are in the closure's units. Costs that are
    /// negative, NaN or infinite make an edge impassable. Turn restrictions
    /// still apply, but turn costs (in seconds) are not added, since the
    /// closure's units need not be seconds. Arbitrary costs rule
    /// out precomputation and distance bounds, so this runs a plain Dijkstra
    /// search; prefer [`SpatialGraph::route`] for the built-in travel times.
    pub fn route_with<F>(
        &self,
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        max_snap_m: Option<f64>,
        mut cost: F,
    ) -> Result<Route, OsmGraphError>
    where
        F: FnMut(EdgeInfo<'_>) -> f64,
    {
        let snaps = snap_both(self, origin.into(), destination.into(), max_snap_m)?;
        let mut edge_cost = |e: EdgeIndex| cost(EdgeInfo::of(&self.graph, e.index() as u32));
        let out = &self.search_index().out;
        let (_, pieces) = best_pieces(
            self,
            &snaps.0,
            &snaps.1,
            &mut edge_cost,
            Pricing::Custom,
            |sources, targets, edge_cost| {
                astar(
                    out,
                    sources,
                    targets,
                    |slot| edge_cost(EdgeIndex::new(out.edges[slot] as usize)),
                    |_| 0.0,
                )
            },
        )
        .ok_or(OsmGraphError::PathNotFound)?;
        Ok(assemble(
            self,
            snaps,
            pieces,
            &mut edge_cost,
            Pricing::Custom,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Edge, OsmNode, OsmTag, SpatialGraph};
    use crate::overpass::NetworkType;
    use petgraph::graph::DiGraph;

    fn make_node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn make_way(drive_travel_time: f64, length: f64) -> Edge {
        Edge {
            way_id: 1,
            last_way_id: 1,
            tags: vec![OsmTag {
                key: "highway".into(),
                value: "residential".into(),
            }]
            .into(),
            length,
            speed_kph: 50.0,
            walk_travel_time: length / (5.0 / 3.6),
            bike_travel_time: length / (15.0 / 3.6),
            drive_travel_time,
            signal_delay_s: 0.0,
            geometry: Vec::new(),
        }
    }

    fn make_profile_way(drive_travel_time: f64, walk_travel_time: f64, length: f64) -> Edge {
        Edge {
            way_id: 1,
            last_way_id: 1,
            tags: vec![OsmTag {
                key: "highway".into(),
                value: "residential".into(),
            }]
            .into(),
            length,
            speed_kph: 50.0,
            walk_travel_time,
            bike_travel_time: walk_travel_time,
            drive_travel_time,
            signal_delay_s: 0.0,
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
        SpatialGraph::new(g, NetworkType::Drive)
    }

    #[test]
    fn test_cumulative_times_starts_at_zero() {
        let sg = linear_graph();
        let r = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();
        assert_eq!(r.cumulative_times_s[0], 0.0);
    }

    #[test]
    fn test_cumulative_times_parallel_to_coordinates() {
        let sg = linear_graph();
        let r = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();
        assert_eq!(r.cumulative_times_s.len(), r.coordinates.len());
    }

    #[test]
    fn test_cumulative_times_monotonic() {
        let sg = linear_graph();
        let r = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();
        for w in r.cumulative_times_s.windows(2) {
            assert!(w[1] >= w[0], "times decreased: {:?}", r.cumulative_times_s);
        }
    }

    #[test]
    fn test_cumulative_times_last_equals_duration() {
        let sg = linear_graph();
        let r = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();
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
        let sg = SpatialGraph::new(g, NetworkType::Drive);

        let route = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();

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
        let sg = SpatialGraph::new(g, NetworkType::Drive);

        let route = sg.route((0.0, 0.0), (0.002, 0.0), None).unwrap();

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
        let sg = SpatialGraph::new(g, NetworkType::Drive);

        let route = sg.route((0.0, 0.0), (0.001, 0.0), None).unwrap();

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

        let forward = sg.route((0.0, 0.0), (0.002, 0.0), None);
        let reverse = sg.route((0.002, 0.0), (0.0, 0.0), None);

        assert!(forward.is_ok());
        assert!(matches!(reverse, Err(OsmGraphError::PathNotFound)));
    }

    #[test]
    fn test_route_origin_equals_destination_is_zero_cost() {
        let sg = linear_graph();

        let route = sg.route((0.0, 0.0), (0.0, 0.0), None).unwrap();

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
        let drive_graph = SpatialGraph::new(g.clone(), NetworkType::Drive);
        let walk_graph = SpatialGraph::new(g, NetworkType::Walk);

        let drive = drive_graph.route((0.0, 0.0), (0.002, 0.0), None).unwrap();
        let walk = walk_graph.route((0.0, 0.0), (0.002, 0.0), None).unwrap();

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
        let sg = SpatialGraph::new(g, NetworkType::Drive);

        let result = sg.route((0.0, 0.0), (1.001, 1.0), None);

        assert!(matches!(result, Err(OsmGraphError::PathNotFound)));
    }

    #[test]
    fn astar_and_ch_costs_match_dijkstra_on_fixture() {
        use crate::reachability::compute_reachability;

        for network_type in [NetworkType::Walk, NetworkType::Drive] {
            let sg = SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", network_type, false)
                .unwrap();
            let prepared = sg.clone();
            prepared.prepare_routing();
            assert!(prepared.is_routing_prepared());
            assert!(sg.is_routing_prepared(), "clones share the hierarchy");
            let unprepared = SpatialGraph::new((*sg.graph).clone(), network_type);

            for origin in sg.graph.node_indices() {
                let from = sg.snap_to_node(origin);
                let exact = compute_reachability(&sg, &from, f64::INFINITY).times;
                for dest in sg.graph.node_indices() {
                    let want = exact.get(dest).copied();
                    for graph in [&prepared, &unprepared] {
                        let got = shortest_pieces(graph, &from, &graph.snap_to_node(dest))
                            .map(|(cost, _)| cost);
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
    fn routes_start_and_end_mid_road() {
        let sg = linear_graph();
        // 1/4 of the way along A→B to 1/2 of the way along B→C.
        let r = sg
            .route((0.00025, 0.00001), (0.0015, -0.00001), None)
            .unwrap();

        assert!(
            (r.duration_s - (7.5 + 5.0)).abs() < 1e-9,
            "{}",
            r.duration_s
        );
        assert!((r.distance_m - (0.75 * 111.0 + 0.5 * 111.0)).abs() < 1e-9);
        let first = r.coordinates[0];
        assert!((first.0 - 0.00025).abs() < 1e-12 && first.1.abs() < 1e-12);
        let last = *r.coordinates.last().unwrap();
        assert!((last.0 - 0.0015).abs() < 1e-12 && last.1.abs() < 1e-12);
        assert!(r.origin_snap.distance_m > 1.0 && r.origin_snap.distance_m < 1.2);
        assert_eq!(r.pieces.len(), 2);
    }

    #[test]
    fn same_road_trips_go_directly_and_can_turn_back_on_two_way_roads() {
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        g.add_edge(a, b, make_way(10.0, 111.0));
        g.add_edge(b, a, make_way(10.0, 111.0));
        let sg = SpatialGraph::new(g, NetworkType::Drive);

        let ahead = sg.route((0.0002, 0.0), (0.0008, 0.0), None).unwrap();
        let behind = sg.route((0.0008, 0.0), (0.0002, 0.0), None).unwrap();

        for r in [&ahead, &behind] {
            assert!((r.duration_s - 6.0).abs() < 1e-9, "{}", r.duration_s);
            assert_eq!(r.pieces.len(), 1, "no detour through a node");
        }
    }

    #[test]
    fn one_way_mid_road_trip_backwards_has_no_route() {
        let sg = linear_graph();
        let r = sg.route((0.0008, 0.0), (0.0002, 0.0), None);
        assert!(matches!(r, Err(OsmGraphError::PathNotFound)));
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
        let sg = SpatialGraph::new(g, NetworkType::Drive);

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
    fn custom_costs_pay_no_turn_costs() {
        // a → b → c with a 10 s turn at b.
        let mut g = DiGraph::new();
        let a = g.add_node(make_node(1, 0.0, 0.0));
        let b = g.add_node(make_node(2, 0.001, 0.0));
        let c = g.add_node(make_node(3, 0.001, 0.001));
        let ab = g.add_edge(a, b, make_way(10.0, 111.0));
        let bc = g.add_edge(b, c, make_way(10.0, 111.0));
        let sg = SpatialGraph::with_turns(g, NetworkType::Drive, vec![], vec![(ab, bc, 10.0)]);

        let timed = sg.route((0.0, 0.0), (0.001, 0.001), None).unwrap();
        assert!(
            (timed.duration_s - 30.0).abs() < 1e-9,
            "{}",
            timed.duration_s
        );

        // Shortest distance: metres only, no seconds of turning mixed in.
        let metres = sg
            .route_with((0.0, 0.0), (0.001, 0.001), None, |e| e.weight.length)
            .unwrap();
        assert!(
            (metres.duration_s - 222.0).abs() < 1e-9,
            "{}",
            metres.duration_s
        );
        let matrix =
            sg.travel_time_matrix_with(&[(0.0, 0.0)], &[(0.001, 0.001)], None, |e| e.weight.length);
        assert!((matrix.durations_s[0][0].unwrap() - 222.0).abs() < 1e-9);
    }

    #[test]
    fn signal_waits_sit_at_the_end_of_their_edge() {
        // A straight road a - s - b with a signal at s; simplification merges
        // it into one edge a → b (and back) carrying the wait.
        let xml = r#"<osm>
            <node id="1" lat="0" lon="0"/>
            <node id="2" lat="0" lon="0.005"><tag k="highway" v="traffic_signals"/></node>
            <node id="3" lat="0" lon="0.01"/>
            <way id="1"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        </osm>"#;
        let sg = SpatialGraph::from_osm(xml, NetworkType::Drive, false).unwrap();
        assert_eq!(sg.graph.edge_count(), 2);
        let edge = sg.graph.edge_weights().next().unwrap();
        assert!((edge.signal_delay_s - 2.0).abs() < 1e-9);
        let moving = edge.drive_travel_time - edge.signal_delay_s;

        let whole = sg.route((0.0, 0.0), (0.0, 0.01), None).unwrap();
        assert!((whole.duration_s - (moving + 2.0)).abs() < 1e-6);
        // A trip that stops a tenth of the way along pays no wait at all.
        let short = sg.route((0.0, 0.0), (0.0, 0.001), None).unwrap();
        assert!(
            (short.duration_s - 0.1 * moving).abs() < 1e-6,
            "{}",
            short.duration_s
        );
        // One that starts a tenth of the way along still pays it in full...
        let rest = sg.route((0.0, 0.001), (0.0, 0.01), None).unwrap();
        assert!((rest.duration_s - (0.9 * moving + 2.0)).abs() < 1e-6);
        // ...at the end, not spread along the way: the midpoint is reached
        // after 0.4 of the moving time.
        assert_eq!(rest.coordinates.len(), 3);
        assert!((rest.cumulative_times_s[1] - 0.4 * moving).abs() < 1e-6);
    }

    #[test]
    fn test_route_respects_max_snap_distance() {
        let sg = linear_graph();

        let close = sg.route((0.0, 0.0), (0.002, 0.0), Some(1.0));
        assert!(close.is_ok());

        let far = sg.route((0.0, 0.0005), (0.002, 0.0), Some(1.0));
        assert!(matches!(
            far,
            Err(OsmGraphError::SnapDistanceExceeded { role: "origin", .. })
        ));
    }
}
