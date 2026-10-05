//! Randomized cross-checks of every search against plain Dijkstra.
//!
//! Each seed builds a small street grid with random one-way streets, speeds,
//! detours, turn restrictions and turn costs, then routes between random
//! points part-way along roads with A*, with the contraction hierarchy, and
//! through both matrix paths. Every answer must match an exhaustive Dijkstra
//! search.

use std::sync::Arc;

use graphways::graph::{Edge, LatLon, OsmNode, OsmTag, RoadGraph, SpatialGraph};
use graphways::overpass::NetworkType;
use graphways::reachability::compute_reachability;
use petgraph::graph::{EdgeIndex, NodeIndex};

struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize % n.max(1)
    }
}

const SIZE: usize = 7;
const STEP_DEG: f64 = 0.001;

fn random_city(seed: u64) -> SpatialGraph {
    let mut rng = Rng(seed);
    let mut graph = RoadGraph::new();
    let tags: Arc<[OsmTag]> = vec![OsmTag {
        key: "highway".into(),
        value: "residential".into(),
    }]
    .into();
    let nodes: Vec<NodeIndex> = (0..SIZE * SIZE)
        .map(|i| {
            graph.add_node(OsmNode {
                id: i as i64 + 1,
                lat: 48.0 + (i / SIZE) as f64 * STEP_DEG + (rng.unit() - 0.5) * 2e-4,
                lon: 11.0 + (i % SIZE) as f64 * STEP_DEG + (rng.unit() - 0.5) * 2e-4,
                tags: vec![],
            })
        })
        .collect();
    let mut way_id = 0;
    for r in 0..SIZE {
        for c in 0..SIZE {
            for (dr, dc) in [(0, 1), (1, 0)] {
                let (nr, nc) = (r + dr, c + dc);
                if nr >= SIZE || nc >= SIZE || rng.unit() < 0.1 {
                    continue; // missing street
                }
                let (a, b) = (nodes[r * SIZE + c], nodes[nr * SIZE + nc]);
                let straight = LatLon::new(graph[a].lat, graph[a].lon)
                    .distance_m(LatLon::new(graph[b].lat, graph[b].lon));
                // Streets curve a little: never shorter than the straight line.
                let length = straight * (1.0 + 0.3 * rng.unit());
                let speed = [20.0, 30.0, 50.0, 70.0][rng.below(4)];
                way_id += 1;
                let edge = Edge::from_length(way_id, tags.clone(), length, speed);
                let roll = rng.unit();
                if roll > 0.2 {
                    graph.add_edge(a, b, edge.clone());
                }
                if roll < 0.8 {
                    graph.add_edge(b, a, edge);
                }
            }
        }
    }
    // Ban a random selection of turns (including some U-turns) and put a
    // random cost on many of the rest.
    let mut forbidden = Vec::new();
    let mut turn_costs = Vec::new();
    for node in graph.node_indices() {
        let into: Vec<EdgeIndex> = graph
            .edges_directed(node, petgraph::Direction::Incoming)
            .map(|e| petgraph::visit::EdgeRef::id(&e))
            .collect();
        let out: Vec<EdgeIndex> = graph
            .edges_directed(node, petgraph::Direction::Outgoing)
            .map(|e| petgraph::visit::EdgeRef::id(&e))
            .collect();
        for &i in &into {
            for &o in &out {
                let roll = rng.unit();
                if roll < 0.15 {
                    forbidden.push((i, o));
                } else if roll < 0.6 {
                    turn_costs.push((i, o, 30.0 * rng.unit()));
                }
            }
        }
    }
    SpatialGraph::with_turns(graph, NetworkType::Drive, forbidden, turn_costs)
}

fn random_point(rng: &mut Rng) -> (f64, f64) {
    let span = (SIZE - 1) as f64 * STEP_DEG;
    (48.0 + rng.unit() * span, 11.0 + rng.unit() * span)
}

#[test]
fn every_search_agrees_with_dijkstra_on_random_cities() {
    let (mut routed, mut pairs) = (0, 0);
    for seed in 1..=12 {
        let sg = random_city(seed);
        assert!(!sg.forbidden_turns().is_empty());
        let mut rng = Rng(seed * 7919);
        let points: Vec<(f64, f64)> = (0..14).map(|_| random_point(&mut rng)).collect();

        let mut astar = Vec::new();
        for &o in &points {
            for &d in &points {
                astar.push(sg.route(o, d, None).ok());
            }
        }
        let unprepared_matrix = sg.travel_time_matrix(&points, &points, None);
        sg.prepare_routing();
        let prepared_matrix = sg.travel_time_matrix(&points, &points, None);

        for (i, &o) in points.iter().enumerate() {
            let from = sg.snap_point(o).unwrap();
            let reach = compute_reachability(&sg, &from, f64::INFINITY);
            for (j, &d) in points.iter().enumerate() {
                let to = sg.snap_point(d).unwrap();
                let ch = sg.route(o, d, None).ok();
                let a = &astar[i * points.len() + j];
                let cell = |m: &graphways::matrix::TravelTimeMatrix| m.durations_s[i][j];
                let answers = [
                    a.as_ref().map(|r| r.duration_s),
                    ch.as_ref().map(|r| r.duration_s),
                    cell(&unprepared_matrix),
                    cell(&prepared_matrix),
                ];
                pairs += 1;
                routed += usize::from(answers[0].is_some());
                let context = format!("seed {seed}, {i} -> {j}: {answers:?}");
                for answer in &answers[1..] {
                    match (answers[0], *answer) {
                        (Some(x), Some(y)) => assert!((x - y).abs() < 1e-6, "{context}"),
                        (x, y) => assert_eq!(x.is_some(), y.is_some(), "{context}"),
                    }
                }

                // Dijkstra through the network: equal, unless the trip stays
                // on one road, which can only make the route cheaper.
                let network = reach.time_to(&sg, &to);
                let same_road = from.edge.is_some()
                    && to.edge.is_some()
                    && (from.edge == to.edge || {
                        let (a, b) = sg.graph.edge_endpoints(from.edge.unwrap()).unwrap();
                        sg.graph.edge_endpoints(to.edge.unwrap()) == Some((b, a))
                    });
                match (answers[0], network) {
                    (Some(route), Some(dijkstra)) if same_road => {
                        assert!(route <= dijkstra + 1e-6, "{context} vs {dijkstra}")
                    }
                    (Some(route), Some(dijkstra)) => {
                        assert!((route - dijkstra).abs() < 1e-6, "{context} vs {dijkstra}")
                    }
                    (None, None) => {}
                    (Some(_), None) => assert!(same_road, "{context}: Dijkstra found nothing"),
                    (None, Some(dijkstra)) => panic!("{context}: Dijkstra found {dijkstra}"),
                }

                // Route geometry is consistent with its own totals.
                if let Some(route) = &ch {
                    let last = *route.cumulative_times_s.last().unwrap();
                    assert!((last - route.duration_s).abs() < 1e-6, "{context}");
                    assert!(route.cumulative_times_s.windows(2).all(|w| w[1] >= w[0]));
                    // Matrices report the length of that same fastest route.
                    for m in [&unprepared_matrix, &prepared_matrix] {
                        let length = m.distances_m[i][j].expect("routed pair has a length");
                        assert!((length - route.distance_m).abs() < 1e-6, "{context}");
                    }
                }
            }
        }
    }
    // Most pairs connect; the rest exercise the unreachable paths.
    assert!(
        routed * 2 > pairs && routed < pairs,
        "{routed} of {pairs} routed"
    );
}
