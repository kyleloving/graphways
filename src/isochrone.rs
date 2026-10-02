//! Isochrones: the areas reachable within given travel times.
//!
//! Travel times from a reachability search are treated as a surface over a
//! Delaunay triangulation of the reached nodes (plus the origin point), and
//! each isochrone is the region of that surface at or below its limit. The
//! region's boundary is traced as oriented rings, so isochrones can have
//! holes (an unreachable lake or rail yard) and several parts.
//!
//! Polygons use the `geo` convention: `x` is longitude, `y` is latitude.

use crate::graph::{LatLon, RoadGraph, SnapResult, SpatialGraph};
use crate::reachability::{compute_reachability, ReachabilityResult};
use geo::{Contains, Coord, LineString, MultiPolygon, Point, Polygon};
use petgraph::graph::NodeIndex;
use std::collections::{HashMap, HashSet};

/// Contour points closer than 10 cm are treated as the same point.
const CONTOUR_KEY_SCALE: f64 = 10.0;
const METERS_PER_DEGREE: f64 = 111_320.0;

/// Holes and separate parts smaller than this (2 ha, about 140 m × 140 m)
/// are dropped: at that size they are usually single slow nodes or
/// triangulation artefacts rather than real unreachable areas.
pub const MIN_RING_AREA_M2: f64 = 20_000.0;

/// Build one isochrone per requested time limit from a precomputed
/// `ReachabilityResult`. The triangulation is built once and shared by every
/// limit. The returned vector is in the same order as `time_limits`.
///
/// Limits greater than `result.max_cost` are clamped to `max_cost` — the result
/// only contains nodes that were searched within that budget.
pub fn build_isochrone_polygons(
    sg: &SpatialGraph,
    result: &ReachabilityResult,
    time_limits: &[f64],
) -> Vec<MultiPolygon> {
    let frontier = frontier(sg, result);
    match TriangulatedSurface::new(&sg.graph, result, &frontier) {
        Some(surface) => time_limits
            .iter()
            .map(|&limit| surface.region(limit.min(result.max_cost)))
            .collect(),
        None => time_limits.iter().map(|_| MultiPolygon(vec![])).collect(),
    }
}

/// Isochrones for every limit from one reachability search sized to the
/// largest limit: one search and one triangulation serve all limits.
pub fn isochrones_from(
    sg: &SpatialGraph,
    origin: &SnapResult,
    time_limits: &[f64],
) -> Vec<MultiPolygon> {
    let max_cost = time_limits.iter().copied().fold(0.0_f64, f64::max);
    let result = compute_reachability(sg, origin, max_cost);
    build_isochrone_polygons(sg, &result, time_limits)
}

/// Nodes one edge beyond the reached set, with the time they would be
/// reached at (more than the budget). They tell the surface where
/// reachability stops, so isochrones end between the last reached and the
/// first unreached node instead of at the hull of the reached ones, and
/// unreachable pockets show up as holes.
fn frontier(sg: &SpatialGraph, result: &ReachabilityResult) -> Vec<(NodeIndex, f64)> {
    let out = &sg.search_index().out;
    let costs = &sg.slot_costs().out;
    let mut best: HashMap<u32, f64> = HashMap::new();
    let index = sg.search_index();
    for (&node, &time) in &result.times {
        for slot in out.range(node.index() as u32) {
            let next = index.node_of(out.neighbors[slot]);
            let cost = costs[slot];
            if cost.is_finite()
                && cost >= 0.0
                && !result.times.contains_key(NodeIndex::new(next as usize))
            {
                let entry = best.entry(next).or_insert(f64::INFINITY);
                *entry = entry.min(time + cost);
            }
        }
    }
    let mut frontier: Vec<(NodeIndex, f64)> = best
        .into_iter()
        .map(|(node, time)| (NodeIndex::new(node as usize), time))
        .collect();
    frontier.sort_unstable_by_key(|&(node, _)| node);
    frontier
}

impl SpatialGraph {
    /// Build isochrones for one or more time limits (seconds) from `origin`,
    /// snapped to the nearest road.
    ///
    /// Each isochrone covers every point reachable within its limit and may
    /// have holes and several parts. The returned `Vec` is in the same order
    /// as `time_limits`.
    pub fn isochrones(
        &self,
        origin: impl Into<LatLon>,
        time_limits: &[f64],
        max_snap_m: Option<f64>,
    ) -> Result<Vec<MultiPolygon>, crate::error::OsmGraphError> {
        let origin = self.snap_endpoint(origin.into(), crate::graph::Role::Origin, max_snap_m)?;
        Ok(isochrones_from(self, &origin, time_limits))
    }
}

/// A reached point in a local metric projection, with its travel time.
#[derive(Clone, Copy)]
struct IsoVertex {
    x: f64,
    y: f64,
    lat: f64,
    lon: f64,
    time: f64,
}

#[derive(Clone, Copy)]
struct ContourPoint {
    x: f64,
    y: f64,
    lat: f64,
    lon: f64,
}

/// A piece of region boundary, oriented so the region lies on its left.
#[derive(Clone, Copy)]
struct ContourSegment {
    from: ContourPoint,
    to: ContourPoint,
}

type ContourKey = (i64, i64);

/// Travel time as a piecewise-linear surface over a Delaunay triangulation
/// of the reached points.
struct TriangulatedSurface {
    vertices: Vec<IsoVertex>,
    /// Vertex indices, three per triangle, each triangle counter-clockwise.
    triangles: Vec<usize>,
    /// Convex hull vertex indices, counter-clockwise.
    hull: Vec<usize>,
}

impl TriangulatedSurface {
    fn new(
        graph: &RoadGraph,
        result: &ReachabilityResult,
        frontier: &[(NodeIndex, f64)],
    ) -> Option<Self> {
        if result.times.is_empty() {
            return None;
        }
        let mut points: Vec<(LatLon, f64)> =
            Vec::with_capacity(result.times.len() + frontier.len() + 1);
        points.push((result.origin, 0.0));
        points.extend(
            result
                .times
                .iter()
                .chain(frontier.iter().map(|(node, time)| (node, time)))
                .map(|(&node, &time)| ((&graph[node]).into(), time)),
        );
        let mean_lat = points.iter().map(|(p, _)| p.lat).sum::<f64>() / points.len() as f64;
        let cos_lat = mean_lat.to_radians().cos();

        // Coincident points would make the triangulation degenerate; keep the
        // first (fastest, as `times` is in settle order) of each.
        let mut seen = HashSet::new();
        let vertices: Vec<IsoVertex> = points
            .into_iter()
            .filter_map(|(p, time)| {
                let (x, y) = (
                    p.lon * METERS_PER_DEGREE * cos_lat,
                    p.lat * METERS_PER_DEGREE,
                );
                let key = (
                    (x * CONTOUR_KEY_SCALE) as i64,
                    (y * CONTOUR_KEY_SCALE) as i64,
                );
                seen.insert(key).then_some(IsoVertex {
                    x,
                    y,
                    lat: p.lat,
                    lon: p.lon,
                    time,
                })
            })
            .collect();
        if vertices.len() < 3 {
            return None;
        }

        let triangulation = delaunator::triangulate(
            &vertices
                .iter()
                .map(|v| delaunator::Point { x: v.x, y: v.y })
                .collect::<Vec<_>>(),
        );
        if triangulation.triangles.is_empty() {
            return None; // all points collinear
        }
        let mut triangles = triangulation.triangles;
        for t in triangles.chunks_exact_mut(3) {
            if cross(&vertices[t[0]], &vertices[t[1]], &vertices[t[2]]) < 0.0 {
                t.swap(1, 2);
            }
        }
        let mut hull = triangulation.hull;
        let hull_area: f64 = (0..hull.len())
            .map(|i| {
                let (a, b) = (&vertices[hull[i]], &vertices[hull[(i + 1) % hull.len()]]);
                a.x * b.y - b.x * a.y
            })
            .sum();
        if hull_area < 0.0 {
            hull.reverse();
        }
        Some(Self {
            vertices,
            triangles,
            hull,
        })
    }

    /// The region where travel time is at most `limit`.
    fn region(&self, limit: f64) -> MultiPolygon {
        let rings = stitch_rings(&self.boundary_segments(limit));
        assemble_polygons(rings)
    }

    /// Every boundary segment of the region, oriented with the region on the
    /// left: level-line pieces inside triangles, plus stretches of the
    /// triangulation's hull, beyond which nothing was reached.
    fn boundary_segments(&self, limit: f64) -> Vec<ContourSegment> {
        let inside = |v: &IsoVertex| v.time <= limit;
        let mut segments = Vec::new();
        for t in self.triangles.chunks_exact(3) {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| self.vertices[i]);
            let mut exit = None;
            let mut entry = None;
            for (from, to) in [(a, b), (b, c), (c, a)] {
                match (inside(&from), inside(&to)) {
                    (true, false) => exit = Some(crossing(from, to, limit)),
                    (false, true) => entry = Some(crossing(from, to, limit)),
                    _ => {}
                }
            }
            if let (Some(from), Some(to)) = (exit, entry) {
                segments.push(ContourSegment { from, to });
            }
        }
        for i in 0..self.hull.len() {
            let from = self.vertices[self.hull[i]];
            let to = self.vertices[self.hull[(i + 1) % self.hull.len()]];
            let segment = match (inside(&from), inside(&to)) {
                (true, true) => Some((point(from), point(to))),
                (true, false) => Some((point(from), crossing(from, to, limit))),
                (false, true) => Some((crossing(from, to, limit), point(to))),
                (false, false) => None,
            };
            if let Some((from, to)) = segment {
                segments.push(ContourSegment { from, to });
            }
        }
        segments.retain(|s| contour_key(s.from) != contour_key(s.to));
        segments
    }
}

fn cross(a: &IsoVertex, b: &IsoVertex, c: &IsoVertex) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn point(v: IsoVertex) -> ContourPoint {
    ContourPoint {
        x: v.x,
        y: v.y,
        lat: v.lat,
        lon: v.lon,
    }
}

/// Where the level line `limit` crosses the edge between `from` and `to`
/// (which lie on opposite sides of it).
fn crossing(from: IsoVertex, to: IsoVertex, limit: f64) -> ContourPoint {
    let ratio = ((limit - from.time) / (to.time - from.time)).clamp(0.0, 1.0);
    ContourPoint {
        x: from.x + (to.x - from.x) * ratio,
        y: from.y + (to.y - from.y) * ratio,
        lat: from.lat + (to.lat - from.lat) * ratio,
        lon: from.lon + (to.lon - from.lon) * ratio,
    }
}

fn contour_key(point: ContourPoint) -> ContourKey {
    (
        (point.x * CONTOUR_KEY_SCALE).round() as i64,
        (point.y * CONTOUR_KEY_SCALE).round() as i64,
    )
}

/// Join oriented segments head to tail into closed rings (first point
/// repeated last). Open chains, which only arise from numerical
/// degeneracies, are dropped.
fn stitch_rings(segments: &[ContourSegment]) -> Vec<Vec<ContourPoint>> {
    let mut outgoing: HashMap<ContourKey, Vec<usize>> = HashMap::with_capacity(segments.len());
    for (i, segment) in segments.iter().enumerate() {
        outgoing
            .entry(contour_key(segment.from))
            .or_default()
            .push(i);
    }
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    for first in 0..segments.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let start = contour_key(segments[first].from);
        let mut ring = vec![segments[first].from, segments[first].to];
        let mut current = first;
        let closed = loop {
            let at = contour_key(segments[current].to);
            if at == start {
                break true;
            }
            let next = outgoing
                .get(&at)
                .and_then(|candidates| candidates.iter().copied().find(|&i| !used[i]));
            let Some(next) = next else {
                break false;
            };
            used[next] = true;
            ring.push(segments[next].to);
            current = next;
        };
        if closed && ring.len() >= 4 {
            rings.push(ring);
        }
    }
    rings
}

/// Signed area in square metres: positive for counter-clockwise rings.
fn signed_area(ring: &[ContourPoint]) -> f64 {
    ring.windows(2)
        .map(|w| w[0].x * w[1].y - w[1].x * w[0].y)
        .sum::<f64>()
        * 0.5
}

fn line_string(ring: &[ContourPoint]) -> LineString {
    LineString(ring.iter().map(|p| Coord { x: p.lon, y: p.lat }).collect())
}

/// Turn boundary rings into polygons: counter-clockwise rings are outer
/// boundaries, clockwise ones are holes, each given to the smallest outer
/// ring that contains it. Rings under [`MIN_RING_AREA_M2`] are dropped,
/// except that the largest part is always kept.
fn assemble_polygons(rings: Vec<Vec<ContourPoint>>) -> MultiPolygon {
    let (mut outers, mut holes): (Vec<_>, Vec<_>) = rings
        .into_iter()
        .map(|ring| (signed_area(&ring), ring))
        .partition(|(area, _)| *area > 0.0);
    outers.sort_by(|a, b| b.0.total_cmp(&a.0));
    let largest = outers.first().map_or(0.0, |(area, _)| *area);
    outers.retain(|(area, _)| *area >= MIN_RING_AREA_M2.min(largest));
    holes.retain(|(area, _)| -*area >= MIN_RING_AREA_M2);

    let shells: Vec<Polygon> = outers
        .iter()
        .map(|(_, ring)| Polygon::new(line_string(ring), vec![]))
        .collect();
    let mut interiors: Vec<Vec<LineString>> = vec![Vec::new(); shells.len()];
    for (_, hole) in &holes {
        let probe = Point::new(hole[0].lon, hole[0].lat);
        // Outers are sorted largest first, so the last match is the smallest.
        if let Some(owner) = shells.iter().rposition(|shell| shell.contains(&probe)) {
            interiors[owner].push(line_string(hole));
        }
    }
    MultiPolygon(
        shells
            .into_iter()
            .zip(interiors)
            .map(|(shell, interiors)| Polygon::new(shell.exterior().clone(), interiors))
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Edge, NodeMap, OsmNode};
    use crate::overpass::NetworkType;
    use geo::Area;
    use petgraph::graph::DiGraph;

    fn contour_point(x: f64, y: f64) -> ContourPoint {
        ContourPoint {
            x,
            y,
            lat: y,
            lon: x,
        }
    }

    #[test]
    fn oriented_segments_stitch_into_closed_rings() {
        let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let segments: Vec<ContourSegment> = (0..4)
            .map(|i| ContourSegment {
                from: contour_point(corners[i].0, corners[i].1),
                to: contour_point(corners[(i + 1) % 4].0, corners[(i + 1) % 4].1),
            })
            .collect();

        let rings = stitch_rings(&segments);
        assert_eq!(rings.len(), 1);
        assert_eq!(rings[0].len(), 5);
        assert!(signed_area(&rings[0]) > 0.0, "counter-clockwise is outer");
    }

    fn make_node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode {
            id,
            lat,
            lon,
            tags: Vec::new(),
        }
    }

    fn make_way(seconds: f64) -> Edge {
        Edge {
            way_id: 1,
            length: seconds,
            speed_kph: 50.0,
            walk_travel_time: seconds,
            bike_travel_time: seconds,
            drive_travel_time: seconds,
            ..Edge::default()
        }
    }

    fn square_graph() -> (SpatialGraph, NodeIndex) {
        let mut graph = DiGraph::new();
        let center = graph.add_node(make_node(0, 0.0, 0.0));
        let north = graph.add_node(make_node(1, 0.001, 0.0));
        let east = graph.add_node(make_node(2, 0.0, 0.001));
        let south = graph.add_node(make_node(3, -0.001, 0.0));
        let west = graph.add_node(make_node(4, 0.0, -0.001));
        for node in [north, east, south, west] {
            graph.add_edge(center, node, make_way(10.0));
            graph.add_edge(node, center, make_way(10.0));
        }
        (SpatialGraph::new(graph, NetworkType::Drive), center)
    }

    /// A `size` × `size` grid, `spacing` degrees apart, with bidirectional
    /// edges costing `cost(a, b)` seconds.
    fn grid(size: usize, spacing: f64, cost: impl Fn(usize, usize) -> f64) -> SpatialGraph {
        let mut graph = DiGraph::new();
        let nodes: Vec<_> = (0..size * size)
            .map(|i| {
                let (r, c) = (i / size, i % size);
                graph.add_node(make_node(i as i64, r as f64 * spacing, c as f64 * spacing))
            })
            .collect();
        for i in 0..size * size {
            let (r, c) = (i / size, i % size);
            for j in [
                (c + 1 < size).then(|| i + 1),
                (r + 1 < size).then(|| i + size),
            ]
            .into_iter()
            .flatten()
            {
                graph.add_edge(nodes[i], nodes[j], make_way(cost(i, j)));
                graph.add_edge(nodes[j], nodes[i], make_way(cost(j, i)));
            }
        }
        SpatialGraph::new(graph, NetworkType::Drive)
    }

    #[test]
    fn empty_reachability_returns_empty_polygons_in_input_order() {
        let graph = SpatialGraph::new(RoadGraph::new(), NetworkType::Drive);
        let result = ReachabilityResult::new(LatLon::default(), 0.0, NodeMap::with_node_count(0));

        let polygons = build_isochrone_polygons(&graph, &result, &[60.0, 30.0]);

        assert_eq!(polygons.len(), 2);
        assert!(polygons.iter().all(|p| p.0.is_empty()));
    }

    #[test]
    fn fewer_than_three_reachable_points_returns_empty_polygon() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        graph.add_edge(a, b, make_way(10.0));
        let sg = SpatialGraph::new(graph, NetworkType::Drive);

        let polygons = isochrones_from(&sg, &sg.snap_to_node(a), &[10.0]);

        assert!(polygons[0].0.is_empty());
    }

    #[test]
    fn isochrone_output_order_matches_input_limits() {
        let (graph, start) = square_graph();

        let polygons = isochrones_from(&graph, &graph.snap_to_node(start), &[20.0, 5.0]);

        assert_eq!(polygons.len(), 2);
        assert!(polygons[0].unsigned_area() > polygons[1].unsigned_area());
    }

    #[test]
    fn increasing_time_limits_have_non_decreasing_area() {
        let (graph, start) = square_graph();

        let polygons = isochrones_from(&graph, &graph.snap_to_node(start), &[5.0, 20.0]);

        assert!(polygons[0].unsigned_area() <= polygons[1].unsigned_area());
    }

    #[test]
    fn polygons_use_lon_as_x() {
        let (graph, start) = square_graph();
        let polygons = isochrones_from(&graph, &graph.snap_to_node(start), &[20.0]);
        // The square spans ±0.001° in both axes; check a vertex is (lon, lat).
        let ring = polygons[0].0[0].exterior();
        assert!(ring.0.iter().any(|c| c.x > 0.0009 && c.y.abs() < 1e-9));
    }

    #[test]
    fn slow_pocket_becomes_a_hole() {
        // 7×7 grid ~330 m apart; every edge touching the centre node is very
        // slow, so it is reached long after its neighbours.
        let center = 3 * 7 + 3;
        let sg = grid(7, 0.003, |a, b| {
            if a == center || b == center {
                2000.0
            } else {
                30.0
            }
        });
        let origin = sg.snap_to_node(sg.node_index(0).unwrap());

        let polygons = isochrones_from(&sg, &origin, &[400.0]);

        assert_eq!(polygons[0].0.len(), 1);
        assert_eq!(polygons[0].0[0].interiors().len(), 1, "centre is a hole");
    }

    #[test]
    fn separated_reachable_areas_become_separate_parts() {
        // Columns 0-1 and 5-6 are fast; the three middle columns are slow,
        // except one fast express edge from the left block to the right one.
        let size = 7;
        let sg = {
            let slow = |i: usize| (2..=4).contains(&(i % size));
            let mut sg = grid(
                size,
                0.003,
                |a, b| if slow(a) || slow(b) { 3000.0 } else { 30.0 },
            );
            let mut graph = (*sg.graph).clone();
            let (left, right) = (NodeIndex::new(0), NodeIndex::new(5));
            graph.add_edge(left, right, make_way(30.0));
            sg = SpatialGraph::new(graph, NetworkType::Drive);
            sg
        };
        let origin = sg.snap_to_node(NodeIndex::new(0));

        let polygons = isochrones_from(&sg, &origin, &[600.0]);

        assert_eq!(polygons[0].0.len(), 2, "left and right blocks");
    }
}
