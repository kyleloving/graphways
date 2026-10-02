//! Travel-time matrices: the fastest time from each of many origins to each
//! of many destinations.
//!
//! With a prepared graph ([`SpatialGraph::prepare_routing`]) a matrix costs
//! one small upward search per point over the contraction hierarchy, so
//! thousands of points take well under a second. Unprepared graphs and
//! custom costs run one full Dijkstra search per point on the smaller side
//! of the matrix instead. Both are exact and run on all cores.

use std::collections::HashMap;

use petgraph::graph::EdgeIndex;
use rayon::prelude::*;

use crate::graph::{seeds, LatLon, Role, SnapResult, SpatialGraph};
use crate::reachability::EdgeInfo;
use crate::search::dijkstra;

/// Travel times between every origin and every destination.
#[derive(Debug, Clone)]
pub struct TravelTimeMatrix {
    /// `durations_s[i][j]` is the travel time in seconds (or the cost
    /// closure's units) from origin `i` to destination `j`; `None` when no
    /// route exists or either point could not be snapped.
    pub durations_s: Vec<Vec<Option<f64>>>,
    /// Where each origin joined the network; `None` if it was farther than
    /// `max_snap_m` from every road (or the graph has no roads).
    pub origin_snaps: Vec<Option<SnapResult>>,
    /// Where each destination joined the network, as for `origin_snaps`.
    pub destination_snaps: Vec<Option<SnapResult>>,
}

/// How to cross the network between anchors.
enum Costs<'a> {
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

fn matrix(
    sg: &SpatialGraph,
    origins: Vec<LatLon>,
    destinations: Vec<LatLon>,
    max_snap_m: Option<f64>,
    edge_cost: &(dyn Fn(EdgeIndex) -> f64 + Sync),
    costs: Costs<'_>,
) -> TravelTimeMatrix {
    let origin_snaps = snap_all(sg, &origins, Role::Origin, max_snap_m);
    let destination_snaps = snap_all(sg, &destinations, Role::Destination, max_snap_m);

    // Seeds for each point: where it enters (or leaves) the search graph and
    // at what cost. Unsnapped points get none and stay unreachable.
    let from: Vec<Vec<(u32, f64)>> = origin_snaps
        .par_iter()
        .map(|snap| match snap {
            Some(snap) => {
                let anchors = sg.departures(snap, &mut |e| edge_cost(e));
                seeds(&sg.departure_roots(&anchors))
            }
            None => Vec::new(),
        })
        .collect();
    let to: Vec<Vec<(u32, f64)>> = destination_snaps
        .par_iter()
        .map(|snap| match snap {
            Some(snap) => {
                let anchors = sg.arrivals(snap, &mut |e| edge_cost(e));
                seeds(&sg.arrival_roots(&anchors))
            }
            None => Vec::new(),
        })
        .collect();

    let width = to.len();
    let mut table = match (&costs, sg.hierarchy_slot().get()) {
        (Costs::Native, Some(hierarchy)) => hierarchy.many_to_many(&from, &to),
        _ => {
            let (out, inc) = match costs {
                Costs::Native => {
                    let slot_costs = sg.slot_costs();
                    (&slot_costs.out[..], &slot_costs.inc[..])
                }
                Costs::Custom { out, inc } => (out, inc),
            };
            dijkstra_table(sg, &from, &to, out, inc)
        }
    };

    // A trip along a single road can beat any path through a junction.
    let mut by_edge: HashMap<EdgeIndex, Vec<usize>> = HashMap::new();
    for (j, snap) in destination_snaps.iter().enumerate() {
        if let Some(edge) = snap.as_ref().and_then(|s| s.edge) {
            by_edge.entry(edge).or_default().push(j);
        }
    }
    if !by_edge.is_empty() && width > 0 {
        table
            .par_chunks_mut(width)
            .zip(origin_snaps.par_iter())
            .for_each(|(row, origin)| {
                let Some(origin) = origin else { return };
                let Some(edge) = origin.edge else { return };
                let candidates = [Some(edge), sg.twin(edge)];
                let same_road = candidates.iter().flatten().filter_map(|e| by_edge.get(e));
                for &j in same_road.flatten() {
                    let destination = destination_snaps[j].as_ref().expect("indexed above");
                    if let Some((cost, _)) =
                        sg.direct_piece(origin, destination, &mut |e| edge_cost(e))
                    {
                        row[j] = row[j].min(cost);
                    }
                }
            });
    }

    let durations_s = if width == 0 {
        vec![Vec::new(); from.len()]
    } else {
        table
            .chunks(width)
            .map(|row| row.iter().map(|&c| c.is_finite().then_some(c)).collect())
            .collect()
    };
    TravelTimeMatrix {
        durations_s,
        origin_snaps,
        destination_snaps,
    }
}

/// One Dijkstra search per point on the smaller side: forward from each
/// origin, or backward from each destination when there are fewer of those.
fn dijkstra_table(
    sg: &SpatialGraph,
    from: &[Vec<(u32, f64)>],
    to: &[Vec<(u32, f64)>],
    out_costs: &[f64],
    inc_costs: &[f64],
) -> Vec<f64> {
    let index = sg.search_index();
    let width = to.len();
    let mut table = vec![f64::INFINITY; from.len() * width];
    if width == 0 || from.is_empty() {
        return table;
    }
    let best = |labels: &crate::graph::NodeMap<f64>, seeds: &[(u32, f64)]| {
        seeds
            .iter()
            .filter_map(|&(state, offset)| {
                labels
                    .get(petgraph::graph::NodeIndex::new(state as usize))
                    .map(|d| d + offset)
            })
            .fold(f64::INFINITY, f64::min)
    };
    if from.len() <= to.len() {
        table
            .par_chunks_mut(width)
            .zip(from.par_iter())
            .for_each(|(row, sources)| {
                if sources.is_empty() {
                    return;
                }
                let labels = dijkstra(&index.out, sources, f64::INFINITY, |s| out_costs[s]);
                for (cell, targets) in row.iter_mut().zip(to) {
                    *cell = best(&labels, targets);
                }
            });
    } else {
        let columns: Vec<Vec<f64>> = to
            .par_iter()
            .map(|targets| {
                if targets.is_empty() {
                    return vec![f64::INFINITY; from.len()];
                }
                let labels = dijkstra(&index.inc, targets, f64::INFINITY, |s| inc_costs[s]);
                from.iter().map(|sources| best(&labels, sources)).collect()
            })
            .collect();
        for (j, column) in columns.into_iter().enumerate() {
            for (i, cost) in column.into_iter().enumerate() {
                table[i * width + j] = cost;
            }
        }
    }
    table
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
        let slot_costs = |edges: &[u32]| -> Vec<f64> {
            edges
                .par_iter()
                .map(|&e| edge_cost(EdgeIndex::new(e as usize)))
                .collect()
        };
        let (out, inc) = (slot_costs(&index.out.edges), slot_costs(&index.inc.edges));
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
                        let want = prepared.route(o, d, Some(100.0)).ok().map(|r| r.duration_s);
                        let got = table.durations_s[i][j];
                        match (got, want) {
                            (Some(a), Some(b)) => {
                                assert!((a - b).abs() < 1e-6, "table {t} {i}->{j}: {a} vs {b}")
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
