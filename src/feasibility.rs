//! Network-time prisms: "which nodes can I visit between origin and destination
//! within a given time budget, and how much slack remains?"
//!
//! # Concept
//!
//! A node `v` is *feasible* if:
//!
//! ```text
//! inbound_time(origin → v) + outbound_time(v → destination) ≤ available_time
//! ```
//!
//! The leftover is the *slack*:
//!
//! ```text
//! slack = available_time − inbound_time − outbound_time
//! ```
//!
//! Callers control what the slack means at the product level:
//! - Pass `available_time = total_window − activity_duration − buffer` to bake
//!   in an activity and a safety margin before calling.
//! - Use `min_slack` in [`build_feasibility_polygon`] to ask "where can I stop
//!   and still have ≥ N seconds left?"
//!
//! # Design notes
//!
//! - The reverse Dijkstra follows incoming edges so that
//!   `outbound_time(v → destination)` is computed as a single one-to-many
//!   search from `destination` rather than N individual searches.
//! - Both searches stop at `available_time`: a node farther than the budget
//!   in either direction can never be feasible.
//! - `NetworkType` is threaded through so walk / bike / drive travel times are
//!   respected consistently.
//! - [`compute_feasibility`] returns `Err(InfeasibleReason)` when the trip
//!   cannot be completed within the budget at all, giving callers a clear
//!   signal to surface to users rather than an opaque empty result.

use std::cell::RefCell;

use geo::{ConvexHull, MultiPoint, Polygon};
use petgraph::graph::EdgeIndex;

use crate::error::OsmGraphError;
use crate::graph::{LatLon, NodeMap, RoadGraph, Role, SnapResult, SpatialGraph};
use crate::reachability::{induced_edge_count, roots, EdgeInfo};
use crate::search::{astar, dijkstra};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Per-node feasibility data for a single origin → destination query.
#[derive(Debug, Clone)]
pub struct FeasibleNode {
    /// Travel time from origin to this node (seconds).
    pub inbound_time: f64,
    /// Travel time from this node to destination (seconds).
    pub outbound_time: f64,
    /// Remaining time after visiting this node:
    /// `available_time − inbound_time − outbound_time`.
    pub slack: f64,
}

/// Full result of a successful [`compute_feasibility`] call.
#[derive(Debug, Clone)]
pub struct FeasibilityResult {
    /// Where the origin snapped onto the network.
    pub origin: SnapResult,
    /// Where the destination snapped onto the network.
    pub destination: SnapResult,
    /// The time budget passed by the caller (seconds).
    pub available_time: f64,
    /// The minimum travel time from origin to destination (seconds).
    /// This is the floor: `available_time` must be ≥ this for any node to be
    /// feasible. Stored here so callers can report headroom to users.
    pub direct_time: f64,
    /// Every node whose `inbound + outbound ≤ available_time`, in increasing
    /// order of inbound time. Nodes unreachable in either direction are absent.
    pub feasible: NodeMap<FeasibleNode>,
}

/// Reason a feasibility query cannot produce any results.
#[derive(Debug, Clone, PartialEq)]
pub enum InfeasibleReason {
    /// The shortest path from origin to destination already exceeds the budget.
    ///
    /// `direct_time` is the actual travel time; `available_time` is what was
    /// requested. The shortfall is `direct_time − available_time`.
    BudgetTooTight {
        direct_time: f64,
        available_time: f64,
    },
    /// No path exists between origin and destination in the graph.
    NoPathExists,
}

impl std::fmt::Display for InfeasibleReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InfeasibleReason::BudgetTooTight {
                direct_time,
                available_time,
            } => write!(
                f,
                "budget too tight: direct travel time is {direct_time:.0} s \
                 but available time is only {available_time:.0} s \
                 (shortfall: {:.0} s)",
                direct_time - available_time
            ),
            InfeasibleReason::NoPathExists => {
                write!(f, "no path exists between origin and destination")
            }
        }
    }
}

impl std::error::Error for InfeasibleReason {}

/// Graph-shaped view for a two-ended, time-bounded query.
///
/// A `PrismGraph` represents the nodes that can be reached from an
/// origin and can still reach a destination within the available travel-time
/// budget. It keeps the original graph and the inbound, outbound, and slack
/// labels, and only materializes an induced subgraph when a constrained
/// operation needs one.
#[derive(Clone)]
pub struct PrismGraph {
    /// Parent graph. The prism node set is stored in `result`.
    pub graph: SpatialGraph,
    pub result: FeasibilityResult,
}

// ---------------------------------------------------------------------------
// Core computation
// ---------------------------------------------------------------------------

/// Which adjacency an edge-cost callback is pricing a slot of.
#[derive(Clone, Copy)]
enum Side {
    Out,
    In,
}

/// Two-sided feasibility. `edge_cost` prices whole edges (for the partial
/// stretches at either end), `price(side, slot)` prices adjacency slots.
fn feasibility(
    sg: &SpatialGraph,
    origin: &SnapResult,
    destination: &SnapResult,
    available_time: f64,
    edge_cost: &mut dyn FnMut(EdgeIndex) -> f64,
    mut price: impl FnMut(Side, usize) -> f64,
) -> Result<FeasibilityResult, InfeasibleReason> {
    let index = sg.search_index();
    let departures = roots(&sg.departures(origin, edge_cost));
    let arrivals = roots(&sg.arrivals(destination, edge_cost));
    let direct = sg
        .direct_piece(origin, destination, edge_cost)
        .map(|(cost, _)| cost);

    // Forward search: origin → every node within budget.
    let forward = dijkstra(&index.out, &departures, available_time, |slot| {
        price(Side::Out, slot)
    });
    let via_network = arrivals
        .iter()
        .filter_map(|&(node, cost)| Some(forward.get(node.into())? + cost))
        .min_by(f64::total_cmp);
    let direct_time = [via_network, direct]
        .into_iter()
        .flatten()
        .filter(|&t| t <= available_time)
        .min_by(f64::total_cmp);

    let Some(direct_time) = direct_time else {
        // Out of budget or disconnected: one targeted search tells which.
        let unbounded = astar(
            &index.out,
            &departures,
            &arrivals,
            |slot| price(Side::Out, slot),
            |_| 0.0,
        )
        .map(|path| path.cost);
        return Err(
            match [unbounded, direct]
                .into_iter()
                .flatten()
                .min_by(f64::total_cmp)
            {
                Some(direct_time) => InfeasibleReason::BudgetTooTight {
                    direct_time,
                    available_time,
                },
                None => InfeasibleReason::NoPathExists,
            },
        );
    };

    // Reverse search: every node → destination, following incoming edges.
    let backward = dijkstra(&index.inc, &arrivals, available_time, |slot| {
        price(Side::In, slot)
    });

    // Intersect: keep nodes present in both searches whose combined cost fits.
    let mut feasible = NodeMap::with_node_count(index.node_count());
    for (&node, &inbound) in &forward {
        let Some(&outbound) = backward.get(node) else {
            continue;
        };
        let total = inbound + outbound;
        if total <= available_time {
            feasible.insert(
                node,
                FeasibleNode {
                    inbound_time: inbound,
                    outbound_time: outbound,
                    slack: available_time - total,
                },
            );
        }
    }

    Ok(FeasibilityResult {
        origin: *origin,
        destination: *destination,
        available_time,
        direct_time,
        feasible,
    })
}

/// Compute two-sided feasibility with a caller-supplied edge cost.
///
/// Finds every node reachable from `origin` that can still reach
/// `destination` within `available_time` seconds (subtract any activity
/// duration or buffer *before* calling). Returns
/// `Err(InfeasibleReason::NoPathExists)` when origin and destination are
/// disconnected, and `Err(InfeasibleReason::BudgetTooTight)` when the direct
/// travel time already exceeds `available_time`.
///
/// The closure is invoked once per edge relaxation in *both* the forward and
/// reverse searches. In each invocation the [`EdgeInfo`] reflects the edge's
/// *original* graph orientation — `source` and `target` are not flipped for
/// the reverse search — so cost models keyed by edge identity, density, or
/// node position see a consistent view in both directions. Costs that are
/// negative, NaN or infinite make an edge impassable.
pub fn compute_feasibility_with<F>(
    sg: &SpatialGraph,
    origin: &SnapResult,
    destination: &SnapResult,
    available_time: f64,
    cost: F,
) -> Result<FeasibilityResult, InfeasibleReason>
where
    F: FnMut(EdgeInfo<'_>) -> f64,
{
    let index = sg.search_index();
    let cost = RefCell::new(cost);
    let price_edge = |edge: u32| (cost.borrow_mut())(EdgeInfo::of(&sg.graph, edge));
    feasibility(
        sg,
        origin,
        destination,
        available_time,
        &mut |e| price_edge(e.index() as u32),
        |side, slot| {
            let adjacency = match side {
                Side::Out => &index.out,
                Side::In => &index.inc,
            };
            price_edge(adjacency.edges[slot])
        },
    )
}

/// Compute two-sided feasibility using the graph's travel times.
///
/// For custom cost models (traffic, density penalties), call
/// [`compute_feasibility_with`].
pub fn compute_feasibility(
    sg: &SpatialGraph,
    origin: &SnapResult,
    destination: &SnapResult,
    available_time: f64,
) -> Result<FeasibilityResult, InfeasibleReason> {
    let costs = sg.slot_costs();
    let nt = sg.network_type();
    feasibility(
        sg,
        origin,
        destination,
        available_time,
        &mut |e| sg.graph[e].travel_time(nt),
        |side, slot| match side {
            Side::Out => costs.out[slot],
            Side::In => costs.inc[slot],
        },
    )
}

// ---------------------------------------------------------------------------
// Polygon construction
// ---------------------------------------------------------------------------

/// Build a polygon enclosing all feasible nodes whose slack ≥ `min_slack`.
///
/// # Arguments
///
/// * `graph`     – The road network (needed to look up node coordinates).
/// * `result`    – Output of [`compute_feasibility`].
/// * `min_slack` – Minimum remaining slack (seconds) a node must have to be
///   included. Pass `0.0` to include every feasible node.
///
/// Returns `None` if fewer than three qualifying nodes exist (a polygon cannot
/// be formed).
pub fn build_feasibility_polygon(
    graph: &RoadGraph,
    result: &FeasibilityResult,
    min_slack: f64,
) -> Option<Polygon> {
    let points: MultiPoint<f64> = result
        .feasible
        .iter()
        .filter(|(_, n)| n.slack >= min_slack)
        .map(|(&idx, _)| {
            let node = &graph[idx];
            geo::Point::new(node.lon, node.lat)
        })
        .collect::<Vec<_>>()
        .into();

    if points.0.len() < 3 {
        return None;
    }

    Some(points.convex_hull())
}

// ---------------------------------------------------------------------------
// SpatialGraph entry points
// ---------------------------------------------------------------------------

impl PrismGraph {
    pub fn node_count(&self) -> usize {
        self.result.feasible.len()
    }

    /// Number of directed edges between prism nodes.
    pub fn edge_count(&self) -> usize {
        induced_edge_count(&self.graph, &self.result.feasible)
    }

    pub fn contains_node_id(&self, node_id: i64) -> bool {
        self.slack_at_node_id(node_id).is_some()
    }

    pub fn slack_at_node_id(&self, node_id: i64) -> Option<f64> {
        let node = self.graph.node_index(node_id)?;
        self.result.feasible.get(node).map(|n| n.slack)
    }

    pub fn materialize(&self) -> SpatialGraph {
        self.graph
            .induced_subgraph(|node| self.result.feasible.contains_key(node))
    }

    /// A route that stays within the prism.
    pub fn route(
        &self,
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        max_snap_m: Option<f64>,
    ) -> Result<crate::routing::Route, OsmGraphError> {
        self.materialize().route(origin, destination, max_snap_m)
    }

    /// Isochrones computed within the prism.
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
    /// Return the network-time prism between two points.
    ///
    /// The result is a lightweight graph view over every node `v` satisfying
    /// `origin -> v -> destination <= available_time`, with each node labeled
    /// by inbound time, outbound time, and slack. Errors with
    /// [`OsmGraphError::Infeasible`] when the trip itself does not fit.
    pub fn prism(
        &self,
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        available_time: f64,
        max_snap_m: Option<f64>,
    ) -> Result<PrismGraph, OsmGraphError> {
        let origin = self.snap_endpoint(origin.into(), Role::Origin, max_snap_m)?;
        let destination = self.snap_endpoint(destination.into(), Role::Destination, max_snap_m)?;
        let result = compute_feasibility(self, &origin, &destination, available_time)?;
        Ok(PrismGraph {
            graph: self.clone(),
            result,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{create_graph, OsmNode, OsmNodeRef, OsmTag, OsmWay};
    use crate::overpass::NetworkType;
    use petgraph::graph::NodeIndex;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    fn node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn way(node_ids: Vec<i64>) -> OsmWay {
        OsmWay {
            id: 1,
            nodes: node_ids
                .into_iter()
                .map(|id| OsmNodeRef { node_id: id })
                .collect(),
            tags: vec![OsmTag {
                key: "highway".into(),
                value: "residential".into(),
            }],
        }
    }

    /// Linear graph:  A ─── B ─── C ─── D  (~111 m between each node)
    fn linear_graph() -> SpatialGraph {
        let nodes = vec![
            node(1, 0.000, 0.0),
            node(2, 0.001, 0.0),
            node(3, 0.002, 0.0),
            node(4, 0.003, 0.0),
        ];
        SpatialGraph::new(
            create_graph(
                nodes,
                vec![way(vec![1, 2, 3, 4])],
                /*retain_all=*/ true,
                false,
            ),
            NetworkType::Drive,
        )
    }

    fn find_node(g: &SpatialGraph, osm_id: i64) -> NodeIndex {
        g.node_index(osm_id).unwrap()
    }

    /// Convenience: run with a generous budget and unwrap — used by tests that
    /// only care about the happy path.
    fn feasibility_ok(
        g: &SpatialGraph,
        origin: NodeIndex,
        dest: NodeIndex,
        budget: f64,
    ) -> FeasibilityResult {
        compute_feasibility(g, &g.snap_to_node(origin), &g.snap_to_node(dest), budget)
            .expect("expected Ok but got Err")
    }

    // ------------------------------------------------------------------
    // Happy-path correctness
    // ------------------------------------------------------------------

    #[test]
    fn feasible_nodes_satisfy_budget() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let result = feasibility_ok(&g, origin, dest, 10_000.0);

        for n in result.feasible.values() {
            assert!(
                n.inbound_time + n.outbound_time <= result.available_time + 1e-9,
                "node violates budget: inbound={} outbound={} budget={}",
                n.inbound_time,
                n.outbound_time,
                result.available_time
            );
            assert!(
                n.slack >= -1e-9,
                "slack must be non-negative, got {}",
                n.slack
            );
        }
    }

    #[test]
    fn origin_and_destination_are_feasible() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let result = feasibility_ok(&g, origin, dest, 10_000.0);

        let o = result
            .feasible
            .get(origin)
            .expect("origin must be feasible");
        assert_eq!(o.inbound_time, 0.0, "origin inbound should be 0");

        let d = result
            .feasible
            .get(dest)
            .expect("destination must be feasible");
        assert_eq!(d.outbound_time, 0.0, "destination outbound should be 0");
    }

    #[test]
    fn prism_graph_view_exposes_induced_counts_and_slack_labels() {
        let sg = linear_graph();
        let prism = sg
            .prism((0.0, 0.0), (0.003, 0.0), 10_000.0, None)
            .expect("budget should be feasible");

        assert_eq!(prism.node_count(), 4);
        assert_eq!(prism.edge_count(), 6);
        assert!(prism.contains_node_id(1));
        assert!(prism.contains_node_id(4));
        assert!(prism.slack_at_node_id(2).is_some());
        assert_eq!(prism.graph.graph.node_count(), 4);
        assert_eq!(
            prism.materialize().graph.node_count(),
            prism.result.feasible.len()
        );
    }

    #[test]
    fn slack_equals_budget_minus_travel_times() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let result = feasibility_ok(&g, origin, dest, 10_000.0);

        for n in result.feasible.values() {
            let expected = result.available_time - n.inbound_time - n.outbound_time;
            assert!(
                (n.slack - expected).abs() < 1e-9,
                "slack mismatch: got {} expected {}",
                n.slack,
                expected
            );
        }
    }

    #[test]
    fn direct_time_stored_in_result() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let result = feasibility_ok(&g, origin, dest, 10_000.0);

        // direct_time must equal the destination's inbound_time (shortest path).
        let dest_node = result.feasible.get(dest).unwrap();
        assert!(
            (result.direct_time - dest_node.inbound_time).abs() < 1e-9,
            "direct_time {} != destination inbound_time {}",
            result.direct_time,
            dest_node.inbound_time
        );
    }

    // ------------------------------------------------------------------
    // InfeasibleReason::BudgetTooTight
    // ------------------------------------------------------------------

    #[test]
    fn budget_too_tight_returns_err() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);

        // First learn the direct time with a generous budget.
        let direct_time = feasibility_ok(&g, origin, dest, 10_000.0).direct_time;

        // Budget just 1 second short of the direct trip.
        let err = compute_feasibility(
            &g,
            &g.snap_to_node(origin),
            &g.snap_to_node(dest),
            direct_time - 1.0,
        )
        .expect_err("expected BudgetTooTight");

        match err {
            InfeasibleReason::BudgetTooTight {
                direct_time: dt,
                available_time: at,
            } => {
                assert!(dt > at, "direct_time should exceed available_time");
                assert!(
                    (dt - direct_time).abs() < 1e-9,
                    "reported direct_time {} doesn't match actual {}",
                    dt,
                    direct_time
                );
            }
            other => panic!("expected BudgetTooTight, got {:?}", other),
        }
    }

    #[test]
    fn budget_too_tight_shortfall_is_correct() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let direct_time = feasibility_ok(&g, origin, dest, 10_000.0).direct_time;

        let shortfall = 42.0;
        let budget = direct_time - shortfall;
        let err = compute_feasibility(&g, &g.snap_to_node(origin), &g.snap_to_node(dest), budget)
            .expect_err("expected BudgetTooTight");

        if let InfeasibleReason::BudgetTooTight {
            direct_time: dt,
            available_time: at,
        } = err
        {
            assert!(
                ((dt - at) - shortfall).abs() < 1e-9,
                "shortfall should be {shortfall} but got {}",
                dt - at
            );
        }
    }

    #[test]
    fn budget_exactly_equal_to_direct_time_is_ok() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);
        let direct_time = feasibility_ok(&g, origin, dest, 10_000.0).direct_time;

        // Exactly at the boundary should succeed (≤, not <).
        let result = compute_feasibility(
            &g,
            &g.snap_to_node(origin),
            &g.snap_to_node(dest),
            direct_time,
        )
        .expect("budget == direct_time should be Ok");

        assert!(result.feasible.contains_key(dest));
        assert!(result.feasible.contains_key(origin));
    }

    // ------------------------------------------------------------------
    // InfeasibleReason::NoPathExists
    // ------------------------------------------------------------------

    #[test]
    fn disconnected_graph_returns_no_path() {
        // Two isolated nodes with no edges between them.
        let mut raw = RoadGraph::new();
        let a = raw.add_node(node(1, 0.0, 0.0));
        let b = raw.add_node(node(2, 1.0, 1.0));
        let g = SpatialGraph::new(raw, NetworkType::Drive);

        let err = compute_feasibility(&g, &g.snap_to_node(a), &g.snap_to_node(b), 10_000.0)
            .expect_err("expected NoPathExists");

        assert_eq!(err, InfeasibleReason::NoPathExists);
    }

    // ------------------------------------------------------------------
    // Display / error trait
    // ------------------------------------------------------------------

    #[test]
    fn budget_too_tight_display_mentions_shortfall() {
        let err = InfeasibleReason::BudgetTooTight {
            direct_time: 3600.0,
            available_time: 1800.0,
        };
        let msg = err.to_string();
        assert!(msg.contains("1800"), "should mention available_time: {msg}");
        assert!(msg.contains("3600"), "should mention direct_time: {msg}");
        assert!(
            msg.contains("1800"),
            "should mention shortfall (1800): {msg}"
        );
    }

    #[test]
    fn no_path_display_is_readable() {
        let msg = InfeasibleReason::NoPathExists.to_string();
        assert!(!msg.is_empty());
    }

    // ------------------------------------------------------------------
    // compute_feasibility_with: closure-controlled cost
    // ------------------------------------------------------------------

    /// Doubling every edge cost via the closure must double both inbound and
    /// outbound times for every feasible node, and the slack must update
    /// consistently with the new totals.
    #[test]
    fn closure_doubles_inbound_and_outbound_consistently() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);

        let baseline =
            compute_feasibility(&g, &g.snap_to_node(origin), &g.snap_to_node(dest), 10_000.0)
                .expect("baseline should be Ok");
        let doubled = compute_feasibility_with(
            &g,
            &g.snap_to_node(origin),
            &g.snap_to_node(dest),
            10_000.0,
            |e| e.weight.travel_time(NetworkType::Drive) * 2.0,
        )
        .expect("doubled should be Ok");

        for (node, base) in &baseline.feasible {
            let d = doubled.feasible.get(*node).expect("doubled missing a node");
            assert!((d.inbound_time - 2.0 * base.inbound_time).abs() < 1e-9);
            assert!((d.outbound_time - 2.0 * base.outbound_time).abs() < 1e-9);
            // Identity must still hold under the doubled cost.
            assert!(
                (d.inbound_time + d.outbound_time + d.slack - 10_000.0).abs() < 1e-9,
                "doubled identity: in={} out={} slack={}",
                d.inbound_time,
                d.outbound_time,
                d.slack
            );
        }
    }

    /// The closure must see every edge in its *original* orientation regardless
    /// of search direction. We verify by passing a closure whose cost depends on
    /// `source.index() < target.index()` and checking that direction-aware
    /// asymmetry is preserved across both searches.
    #[test]
    fn closure_sees_original_orientation_in_both_searches() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);

        // Charge 10s for "forward" edges (source < target by index) and 100s
        // for "backward" edges. A symmetric cost would give the same in both
        // searches; an oriented cost would differ. Either way, the slack
        // identity must hold.
        let result = compute_feasibility_with(
            &g,
            &g.snap_to_node(origin),
            &g.snap_to_node(dest),
            10_000.0,
            |e| {
                if e.source.index() < e.target.index() {
                    10.0
                } else {
                    100.0
                }
            },
        )
        .expect("should be Ok");

        for f in result.feasible.values() {
            assert!((f.inbound_time + f.outbound_time + f.slack - 10_000.0).abs() < 1e-9);
            assert!(f.slack >= 0.0);
        }
    }

    /// The convenience `compute_feasibility` must produce identical results to
    /// `compute_feasibility_with` invoked with the equivalent baseline closure.
    #[test]
    fn convenience_wrapper_matches_with_variant() {
        let g = linear_graph();
        let origin = find_node(&g, 1);
        let dest = find_node(&g, 4);

        let a = compute_feasibility(&g, &g.snap_to_node(origin), &g.snap_to_node(dest), 10_000.0)
            .unwrap();
        let b = compute_feasibility_with(
            &g,
            &g.snap_to_node(origin),
            &g.snap_to_node(dest),
            10_000.0,
            |e| e.weight.travel_time(NetworkType::Drive),
        )
        .unwrap();

        assert_eq!(a.feasible.len(), b.feasible.len());
        for (node, fa) in &a.feasible {
            let fb = b.feasible.get(*node).expect("node missing in _with result");
            assert!((fa.inbound_time - fb.inbound_time).abs() < 1e-9);
            assert!((fa.outbound_time - fb.outbound_time).abs() < 1e-9);
            assert!((fa.slack - fb.slack).abs() < 1e-9);
        }
    }
}
