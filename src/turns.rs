//! Turn costs at junctions for driving graphs.

use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction;

use crate::graph::RoadGraph;
use crate::profile::TurnCosts;

/// Compass heading (degrees clockwise from north) from `a` to `b`, both
/// `(lat, lon)`, in a local projection; `None` for coincident points.
fn heading(a: (f64, f64), b: (f64, f64)) -> Option<f64> {
    let dy = b.0 - a.0;
    let dx = (b.1 - a.1) * a.0.to_radians().cos();
    (dx != 0.0 || dy != 0.0).then(|| dx.atan2(dy).to_degrees())
}

/// Heading of `edge` as it arrives at its target (`arriving`) or leaves its
/// source, from the shape points next to that end.
fn end_heading(graph: &RoadGraph, edge: EdgeIndex, arriving: bool) -> Option<f64> {
    let (source, target) = graph.edge_endpoints(edge)?;
    let geometry = graph[edge].oriented_geometry(&graph[source], &graph[target]);
    let points: Vec<(f64, f64)> = geometry.points().collect();
    if arriving {
        let end = *points.last()?;
        points.iter().rev().skip(1).find_map(|&p| heading(p, end))
    } else {
        let start = *points.first()?;
        points.iter().skip(1).find_map(|&p| heading(start, p))
    }
}

/// Signed turn angle from heading `from` to heading `to`, in (-180, 180]:
/// 0 is straight on, positive turns right.
fn turn_angle(from: f64, to: f64) -> f64 {
    let mut angle = (to - from) % 360.0;
    if angle > 180.0 {
        angle -= 360.0;
    } else if angle <= -180.0 {
        angle += 360.0;
    }
    angle
}

/// The cost of every turn worth pricing, as `(into junction, out of
/// junction, seconds)`: all turns at junctions of three or more roads, and
/// U-turns everywhere (back along the road just travelled).
pub(crate) fn turn_costs(graph: &RoadGraph, costs: &TurnCosts) -> Vec<(EdgeIndex, EdgeIndex, f64)> {
    let mut out = Vec::new();
    if costs.is_none() {
        return out;
    }
    let mut neighbours: Vec<NodeIndex> = Vec::new();
    for node in graph.node_indices() {
        neighbours.clear();
        neighbours.extend(graph.neighbors_undirected(node).filter(|&n| n != node));
        neighbours.sort_unstable();
        neighbours.dedup();
        let junction = neighbours.len() > 2;

        for into in graph.edges_directed(node, Direction::Incoming) {
            let arriving = end_heading(graph, into.id(), true);
            for exit in graph.edges_directed(node, Direction::Outgoing) {
                if exit.id() == into.id() {
                    continue; // a self-loop is not a turn
                }
                let u_turn = exit.target() == into.source();
                if !junction && !u_turn {
                    continue;
                }
                let angle = match (arriving, end_heading(graph, exit.id(), false)) {
                    (Some(from), Some(to)) => turn_angle(from, to),
                    _ if u_turn => 180.0,
                    _ => 0.0,
                };
                let cost = costs.cost(angle, u_turn);
                // Below a hundredth of a second the extra search state costs
                // more than the precision gains.
                if cost >= 0.01 {
                    out.push((into.id(), exit.id(), cost));
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Edge, OsmNode, OsmTag};
    use std::sync::Arc;

    #[test]
    fn angles_are_signed_with_right_positive() {
        let north = heading((0.0, 0.0), (1.0, 0.0)).unwrap();
        let east = heading((0.0, 0.0), (0.0, 1.0)).unwrap();
        let west = heading((0.0, 0.0), (0.0, -1.0)).unwrap();
        assert!((turn_angle(north, east) - 90.0).abs() < 1e-9);
        assert!((turn_angle(north, west) + 90.0).abs() < 1e-9);
        assert!((turn_angle(east, west).abs() - 180.0).abs() < 1e-9);
    }

    #[test]
    fn crossroads_price_left_right_and_u_turns() {
        // A crossroads at the origin with two-way arms north, east, south, west.
        let tags: Arc<[OsmTag]> = Vec::new().into();
        let mut g = RoadGraph::new();
        let node = |id, lat, lon| OsmNode {
            id,
            lat,
            lon,
            tags: vec![],
        };
        let centre = g.add_node(node(0, 0.0, 0.0));
        let arms: Vec<NodeIndex> = [(0.001, 0.0), (0.0, 0.001), (-0.001, 0.0), (0.0, -0.001)]
            .iter()
            .enumerate()
            .map(|(i, &(lat, lon))| g.add_node(node(i as i64 + 1, lat, lon)))
            .collect();
        let mut into = Vec::new();
        let mut exit = Vec::new();
        for &arm in &arms {
            into.push(g.add_edge(arm, centre, Edge::from_length(1, tags.clone(), 111.0, 50.0)));
            exit.push(g.add_edge(centre, arm, Edge::from_length(1, tags.clone(), 111.0, 50.0)));
        }
        let costs = turn_costs(&g, &TurnCosts::default());
        let cost_of = |a: EdgeIndex, b: EdgeIndex| {
            costs
                .iter()
                .find(|&&(x, y, _)| x == a && y == b)
                .map_or(0.0, |&(_, _, c)| c)
        };
        // Arriving from the south (heading north): east is a right turn.
        let from_south = into[2];
        let right = cost_of(from_south, exit[1]);
        let left = cost_of(from_south, exit[3]);
        let straight = cost_of(from_south, exit[0]);
        let back = cost_of(from_south, exit[2]);
        assert!(
            straight < 0.01 && right < left && left < back,
            "{straight} {right} {left} {back}"
        );
        assert!(back > 20.0);
        // Dead ends at the arm tips allow only U-turns, which are priced.
        assert!(costs.iter().any(|&(a, b, _)| a == exit[0] && b == into[0]));
    }
}
