//! Travel-time matrices: the fastest time from each of many origins to each
//! of many destinations, and the length of each of those fastest routes.
//!
//! With a prepared graph ([`SpatialGraph::prepare_routing`]) a matrix costs
//! one small upward search per point over the contraction hierarchy, so
//! thousands of points take well under a second. Unprepared graphs and
//! custom costs run one full Dijkstra search per point on the smaller side
//! of the matrix instead. Both are exact and run on all cores.

use std::collections::HashMap;

use petgraph::graph::EdgeIndex;
use rayon::prelude::*;

use crate::ch::Seed;
use crate::graph::{Anchor, LatLon, Role, Roots, SnapResult, SpatialGraph};
use crate::reachability::EdgeInfo;
use crate::search::dijkstra_with_lengths;

/// Travel times between every origin and every destination.
#[derive(Debug, Clone)]
pub struct TravelTimeMatrix {
    /// `durations_s[i][j]` is the travel time in seconds (or the cost
    /// closure's units) from origin `i` to destination `j`; `None` when no
    /// route exists or either point could not be snapped.
    pub durations_s: Vec<Vec<Option<f64>>>,
    /// `distances_m[i][j]` is the length in metres of that fastest route;
    /// `None` exactly where `durations_s` is.
    pub distances_m: Vec<Vec<Option<f64>>>,
    /// Where each origin joined the network; `None` if it was farther than
    /// `max_snap_m` from every road (or the graph has no roads).
    pub origin_snaps: Vec<Option<SnapResult>>,
    /// Where each destination joined the network, as for `origin_snaps`.
    pub destination_snaps: Vec<Option<SnapResult>>,
}

/// How to cross the network between anchors.
pub(crate) enum Costs<'a> {
    /// The graph's own travel times, through the hierarchy if prepared.
    Native,
    /// Caller-supplied per-slot costs (forward, backward adjacency).
    Custom { out: &'a [f64], inc: &'a [f64] },
}

fn snap_all(
    sg: &SpatialGraph,
    points: &[LatLon],
    role: Role,
    max_snap_m: Option<f64>,
) -> Vec<Option<SnapResult>> {
    points
        .par_iter()
        .map(|&p| sg.snap_endpoint(p, role, max_snap_m).ok())
        .collect()
}

/// `(state, cost, metres)` seeds for a point's roots: the metres are the
/// share of road between the point and the junction each root stands for.
fn seeds_with_lengths(sg: &SpatialGraph, anchors: &[Anchor], roots: &Roots) -> Vec<Seed> {
    roots
        .iter()
        .map(|&(state, cost, i)| {
            let length = anchors[i]
                .piece
                .map_or(0.0, |p| p.share() * sg.graph[p.edge].length);
            (state, cost, length)
        })
        .collect()
}

/// Snapped points and search seeds for one origins x destinations query,
/// computed row by row so callers can reduce each row as it is produced.
pub(crate) struct Table<'g> {
    sg: &'g SpatialGraph,
    edge_cost: Box<dyn Fn(EdgeIndex) -> f64 + Sync + 'g>,
    pub(crate) origin_snaps: Vec<Option<SnapResult>>,
    pub(crate) destination_snaps: Vec<Option<SnapResult>>,
    from: Vec<Vec<Seed>>,
    to: Vec<Vec<Seed>>,
    /// Destinations by the road they snapped to (and its twin), for trips
    /// that never leave one road.
    by_edge: HashMap<EdgeIndex, Vec<usize>>,
}

/// A cell of a table: `(cost, metres)`, infinite when unreachable.
pub(crate) type Cell = (f64, f64);
const UNREACHABLE: Cell = (f64::INFINITY, f64::INFINITY);

impl<'g> Table<'g> {
    pub(crate) fn new(
        sg: &'g SpatialGraph,
        origins: &[LatLon],
        destinations: &[LatLon],
        max_snap_m: Option<f64>,
        edge_cost: impl Fn(EdgeIndex) -> f64 + Sync + 'g,
    ) -> Self {
        let origin_snaps = snap_all(sg, origins, Role::Origin, max_snap_m);
        let destination_snaps = snap_all(sg, destinations, Role::Destination, max_snap_m);

        // Seeds for each point: where it enters (or leaves) the search graph
        // and at what cost. Unsnapped points get none and stay unreachable.
        let from: Vec<Vec<Seed>> = origin_snaps
            .par_iter()
            .map(|snap| match snap {
                Some(snap) => {
                    let anchors = sg.departures(snap, &mut |e| edge_cost(e));
                    seeds_with_lengths(sg, &anchors, &sg.departure_roots(&anchors))
                }
                None => Vec::new(),
            })
            .collect();
        let to: Vec<Vec<Seed>> = destination_snaps
            .par_iter()
            .map(|snap| match snap {
                Some(snap) => {
                    let anchors = sg.arrivals(snap, &mut |e| edge_cost(e));
                    seeds_with_lengths(sg, &anchors, &sg.arrival_roots(&anchors))
                }
                None => Vec::new(),
            })
            .collect();

        let mut by_edge: HashMap<EdgeIndex, Vec<usize>> = HashMap::new();
        for (j, snap) in destination_snaps.iter().enumerate() {
            // Indexed under the road's twin too: `twin` is not symmetric when
            // parallel edges join the same nodes, and `direct_piece` checks both.
            if let Some(edge) = snap.as_ref().and_then(|s| s.edge) {
                for key in std::iter::once(edge).chain(sg.twin(edge)) {
                    by_edge.entry(key).or_default().push(j);
                }
            }
        }
        Table {
            sg,
            edge_cost: Box::new(edge_cost),
            origin_snaps,
            destination_snaps,
            from,
            to,
            by_edge,
        }
    }

    /// A trip along a single road can beat any path through a junction.
    fn same_road(&self, origin: usize, row: &mut [Cell]) {
        let Some(origin) = &self.origin_snaps[origin] else {
            return;
        };
        let Some(edge) = origin.edge else { return };
        let candidates = [Some(edge), self.sg.twin(edge)];
        let mut same_road: Vec<usize> = candidates
            .iter()
            .flatten()
            .filter_map(|e| self.by_edge.get(e))
            .flatten()
            .copied()
            .collect();
        same_road.sort_unstable();
        same_road.dedup();
        for j in same_road {
            let destination = self.destination_snaps[j].as_ref().expect("indexed above");
            if let Some((cost, piece)) = self
                .sg
                .direct_piece(origin, destination, &mut |e| (self.edge_cost)(e))
            {
                if cost < row[j].0 {
                    row[j] = (cost, piece.share() * self.sg.graph[piece.edge].length);
                }
            }
        }
    }

    /// Compute every origin's row of cells and pass it to `reduce`, in
    /// parallel. With the graph's own costs and a prepared hierarchy this
    /// uses the bucket method; otherwise one Dijkstra search per origin, or,
    /// unless `bounded_memory`, one per destination when those are fewer
    /// (which holds the whole table in memory).
    pub(crate) fn rows<R: Send>(
        &self,
        costs: Costs<'_>,
        bounded_memory: bool,
        reduce: impl Fn(usize, &[Cell]) -> R + Sync,
    ) -> Vec<R> {
        let width = self.to.len();
        let finish = |i: usize, mut row: Vec<Cell>| {
            self.same_road(i, &mut row);
            reduce(i, &row)
        };
        if let (Costs::Native, Some(hierarchy)) = (&costs, self.sg.hierarchy_slot().get()) {
            let buckets = hierarchy.buckets(&self.to);
            return self
                .from
                .par_iter()
                .enumerate()
                .map(|(i, seeds)| {
                    let mut row = vec![UNREACHABLE; width];
                    if !seeds.is_empty() {
                        hierarchy.fill_row(&buckets, seeds, &mut row);
                    }
                    finish(i, row)
                })
                .collect();
        }

        let slot_costs;
        let (out_costs, inc_costs) = match costs {
            Costs::Native => {
                slot_costs = self.sg.slot_costs();
                (&slot_costs.out[..], &slot_costs.inc[..])
            }
            Costs::Custom { out, inc } => (out, inc),
        };
        let index = self.sg.search_index();
        let edge_length = |adjacency: &crate::search::Adjacency, slot: usize| {
            self.sg.graph.raw_edges()[adjacency.edges[slot] as usize]
                .weight
                .length
        };
        let best = |labels: &crate::graph::NodeMap<Cell>, seeds: &[Seed]| {
            seeds
                .iter()
                .filter_map(|&(state, cost, length)| {
                    let &(c, l) = labels.get(petgraph::graph::NodeIndex::new(state as usize))?;
                    Some((c + cost, l + length))
                })
                .fold(UNREACHABLE, |a, b| if b.0 < a.0 { b } else { a })
        };

        if bounded_memory || self.from.len() <= width {
            return self
                .from
                .par_iter()
                .enumerate()
                .map(|(i, sources)| {
                    let row = if sources.is_empty() || width == 0 {
                        vec![UNREACHABLE; width]
                    } else {
                        let labels = dijkstra_with_lengths(
                            &index.out,
                            sources,
                            |s| out_costs[s],
                            |s| edge_length(&index.out, s),
                        );
                        self.to
                            .iter()
                            .map(|targets| best(&labels, targets))
                            .collect()
                    };
                    finish(i, row)
                })
                .collect();
        }

        // Fewer destinations: search backward from each, then read rows out.
        let columns: Vec<Vec<Cell>> = self
            .to
            .par_iter()
            .map(|targets| {
                if targets.is_empty() {
                    return vec![UNREACHABLE; self.from.len()];
                }
                let labels = dijkstra_with_lengths(
                    &index.inc,
                    targets,
                    |s| inc_costs[s],
                    |s| edge_length(&index.inc, s),
                );
                self.from
                    .iter()
                    .map(|sources| best(&labels, sources))
                    .collect()
            })
            .collect();
        (0..self.from.len())
            .into_par_iter()
            .map(|i| finish(i, columns.iter().map(|column| column[i]).collect()))
            .collect()
    }
}

fn matrix(
    sg: &SpatialGraph,
    origins: Vec<LatLon>,
    destinations: Vec<LatLon>,
    max_snap_m: Option<f64>,
    edge_cost: &(dyn Fn(EdgeIndex) -> f64 + Sync),
    costs: Costs<'_>,
) -> TravelTimeMatrix {
    let table = Table::new(sg, &origins, &destinations, max_snap_m, edge_cost);
    let known = |x: f64, cell: &Cell| cell.0.is_finite().then_some(x);
    type Row = Vec<Option<f64>>;
    let rows: Vec<(Row, Row)> = table.rows(costs, false, |_, row| {
        (
            row.iter().map(|c| known(c.0, c)).collect(),
            row.iter().map(|c| known(c.1, c)).collect(),
        )
    });
    let (durations_s, distances_m) = rows.into_iter().unzip();
    TravelTimeMatrix {
        durations_s,
        distances_m,
        origin_snaps: table.origin_snaps,
        destination_snaps: table.destination_snaps,
    }
}

impl SpatialGraph {
    /// Fastest travel times from every origin to every destination.
    ///
    /// Each point snaps to the nearest road as in [`SpatialGraph::route`];
    /// a point farther than `max_snap_m` from every road gets a `None` snap
    /// and `None` times rather than failing the whole matrix. Run
    /// [`SpatialGraph::prepare_routing`] first for large matrices: prepared
    /// graphs answer with the contraction hierarchy, unprepared ones with
    /// one Dijkstra search per point on the smaller side.
    pub fn travel_time_matrix<O, D>(
        &self,
        origins: &[O],
        destinations: &[D],
        max_snap_m: Option<f64>,
    ) -> TravelTimeMatrix
    where
        O: Into<LatLon> + Copy,
        D: Into<LatLon> + Copy,
    {
        let nt = self.network_type();
        matrix(
            self,
            origins.iter().map(|&p| p.into()).collect(),
            destinations.iter().map(|&p| p.into()).collect(),
            max_snap_m,
            &|e| self.graph[e].travel_time(nt),
            Costs::Native,
        )
    }

    /// Like [`SpatialGraph::travel_time_matrix`], under a caller-supplied
    /// edge cost (see [`SpatialGraph::route_with`]). Always runs one Dijkstra
    /// search per point on the smaller side of the matrix.
    pub fn travel_time_matrix_with<O, D, F>(
        &self,
        origins: &[O],
        destinations: &[D],
        max_snap_m: Option<f64>,
        cost: F,
    ) -> TravelTimeMatrix
    where
        O: Into<LatLon> + Copy,
        D: Into<LatLon> + Copy,
        F: Fn(EdgeInfo<'_>) -> f64 + Sync,
    {
        let edge_cost = |e: EdgeIndex| cost(EdgeInfo::of(&self.graph, e.index() as u32));
        let index = self.search_index();
        let slot_costs = |adjacency: &crate::search::Adjacency| -> Vec<f64> {
            adjacency
                .edges
                .par_iter()
                .enumerate()
                .map(|(slot, &e)| adjacency.slot_cost(slot, edge_cost(EdgeIndex::new(e as usize))))
                .collect()
        };
        let (out, inc) = (slot_costs(&index.out), slot_costs(&index.inc));
        matrix(
            self,
            origins.iter().map(|&p| p.into()).collect(),
            destinations.iter().map(|&p| p.into()).collect(),
            max_snap_m,
            &edge_cost,
            Costs::Custom {
                out: &out,
                inc: &inc,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::graph::SpatialGraph;
    use crate::overpass::NetworkType;

    fn points(sg: &SpatialGraph) -> Vec<(f64, f64)> {
        // Every node, plus points part-way along roads and one far away.
        let mut points: Vec<(f64, f64)> = sg.graph.node_weights().map(|n| (n.lat, n.lon)).collect();
        let shifted: Vec<(f64, f64)> = sg
            .graph
            .edge_indices()
            .step_by(3)
            .map(|e| {
                let (a, b) = sg.graph.edge_endpoints(e).unwrap();
                let (a, b) = (&sg.graph[a], &sg.graph[b]);
                (a.lat * 0.7 + b.lat * 0.3, a.lon * 0.7 + b.lon * 0.3)
            })
            .collect();
        points.extend(shifted);
        points.push((0.0, 0.0));
        points
    }

    #[test]
    fn matrix_matches_point_to_point_routes() {
        for network_type in [NetworkType::Walk, NetworkType::Drive] {
            let sg = SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", network_type, false)
                .unwrap();
            let pts = points(&sg);
            let (origins, destinations) = (&pts[..pts.len().min(25)], &pts[..]);
            let unprepared = SpatialGraph::new((*sg.graph).clone(), network_type);
            let prepared = sg;
            prepared.prepare_routing();

            let tables = [
                prepared.travel_time_matrix(origins, destinations, Some(100.0)),
                unprepared.travel_time_matrix(origins, destinations, Some(100.0)),
                // Fewer destinations than origins: backward searches.
                unprepared.travel_time_matrix(destinations, origins, Some(100.0)),
                unprepared.travel_time_matrix_with(origins, destinations, Some(100.0), |e| {
                    e.weight.travel_time(network_type)
                }),
            ];
            for (t, table) in tables.iter().enumerate() {
                let flipped = t == 2;
                let (rows, cols) = if flipped {
                    (destinations, origins)
                } else {
                    (origins, destinations)
                };
                assert_eq!(table.durations_s.len(), rows.len());
                for (i, &o) in rows.iter().enumerate() {
                    for (j, &d) in cols.iter().enumerate() {
                        let route = prepared.route(o, d, Some(100.0)).ok();
                        let want = route.as_ref().map(|r| r.duration_s);
                        let got = table.durations_s[i][j];
                        assert_eq!(got.is_some(), table.distances_m[i][j].is_some());
                        match (got, want) {
                            (Some(a), Some(b)) => {
                                assert!((a - b).abs() < 1e-6, "table {t} {i}->{j}: {a} vs {b}");
                                let (got, want) = (
                                    table.distances_m[i][j].unwrap(),
                                    route.as_ref().unwrap().distance_m,
                                );
                                assert!(
                                    (got - want).abs() < 1e-6,
                                    "table {t} {i}->{j}: {got} m vs {want} m"
                                );
                            }
                            (None, None) => {}
                            other => panic!("table {t} {i}->{j}: {other:?}"),
                        }
                    }
                }
            }
            // The far-away point is unsnapped, not an error.
            let last = tables[0].destination_snaps.len() - 1;
            assert!(tables[0].destination_snaps[last].is_none());
            assert!(tables[0].durations_s.iter().all(|row| row[last].is_none()));
        }
    }

    #[test]
    fn same_road_trips_match_routes_with_parallel_edges() {
        use crate::graph::{Edge, OsmNode, OsmTag, RoadGraph};
        use std::sync::Arc;
        let tags: Arc<[OsmTag]> = vec![OsmTag {
            key: "highway".into(),
            value: "residential".into(),
        }]
        .into();
        let node = |id, lon| OsmNode {
            id,
            lat: 48.0,
            lon,
            tags: vec![],
        };
        let mut g = RoadGraph::new();
        let (n0, n1) = (g.add_node(node(1, 11.0)), g.add_node(node(2, 11.001)));
        g.add_edge(n0, n1, Edge::from_length(10, tags.clone(), 74.5, 30.0));
        g.add_edge(n1, n0, Edge::from_length(10, tags.clone(), 74.5, 30.0));
        let mut curved = Edge::from_length(20, tags, 75.0, 30.0);
        curved.geometry = vec![(48.0, 11.0), (48.00003, 11.0005), (48.0, 11.001)];
        g.add_edge(n0, n1, curved);
        let sg = SpatialGraph::new(g, NetworkType::Walk);
        let (o, d) = ((48.00004, 11.0006), (47.99999, 11.0002));

        let route = sg.route(o, d, None).unwrap().duration_s;
        let unprepared = sg.travel_time_matrix(&[o], &[d], None).durations_s[0][0];
        sg.prepare_routing();
        let prepared = sg.travel_time_matrix(&[o], &[d], None).durations_s[0][0];
        for cell in [unprepared, prepared] {
            assert!((cell.unwrap() - route).abs() < 1e-9, "{cell:?} vs {route}");
        }
    }

    #[test]
    fn non_finite_points_are_unsnapped_not_fatal() {
        let sg =
            SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", NetworkType::Walk, false)
                .unwrap();
        let points = [(f64::NAN, 11.0), (48.0, f64::INFINITY), (48.0, 11.0)];
        let table = sg.travel_time_matrix(&points, &points, None);
        assert!(table.origin_snaps[0].is_none() && table.origin_snaps[1].is_none());
        assert_eq!(table.durations_s[2][2], Some(0.0));
        assert!(sg.route((f64::NAN, 0.0), (48.0, 11.0), None).is_err());
    }

    #[test]
    fn empty_sides_give_empty_tables() {
        let sg =
            SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", NetworkType::Walk, false)
                .unwrap();
        let none: [(f64, f64); 0] = [];
        let some = [(
            sg.graph[petgraph::graph::NodeIndex::new(0)].lat,
            sg.graph[petgraph::graph::NodeIndex::new(0)].lon,
        )];
        assert_eq!(
            sg.travel_time_matrix(&none, &some, None).durations_s.len(),
            0
        );
        let wide = sg.travel_time_matrix(&some, &none, None);
        assert_eq!(wide.durations_s, vec![Vec::<Option<f64>>::new()]);
    }
}
