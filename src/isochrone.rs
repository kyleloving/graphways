#[cfg(feature = "extension-module")]
use crate::error::OsmGraphError;
use crate::graph::{self, RoadGraph, SpatialGraph};
#[cfg(feature = "extension-module")]
use crate::overpass;
use crate::overpass::NetworkType;
use crate::reachability::{compute_reachability, ReachabilityResult};
use geo::{ConvexHull, LineString, MultiPoint, Polygon};
use petgraph::prelude::*;
use std::collections::{HashMap, HashSet};

const SATURATED_REUSE_RATIO: f64 = 0.99;
const CONTOUR_KEY_SCALE: f64 = 10.0;

/// Build one isochrone polygon per requested time limit from a precomputed
/// `ReachabilityResult`, by contouring a Delaunay triangulation of the
/// reached nodes' travel times. The triangulation is built once and shared by
/// every limit. The returned vector is in the same order as `time_limits`.
///
/// Limits greater than `result.max_cost` are clamped to `max_cost` — the result
/// only contains nodes that were searched within that budget.
pub fn build_isochrone_polygons(
    graph: &RoadGraph,
    result: &ReachabilityResult,
    time_limits: &[f64],
) -> Vec<Polygon> {
    let mut node_times: Vec<(NodeIndex, f64)> =
        result.times.iter().map(|(&n, &t)| (n, t)).collect();
    // Break time ties by node index so the output never depends on hash order.
    node_times.sort_unstable_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    let Some(&(_, max_seen)) = node_times.last() else {
        return time_limits.iter().map(|_| empty_polygon()).collect();
    };

    build_triangulated_isochrones(graph, &node_times, time_limits, max_seen.max(0.0))
}

fn is_saturated_limit(node_times: &[(NodeIndex, f64)], limit: f64) -> bool {
    if node_times.is_empty() {
        return false;
    }
    let reachable = upper_bound_node_times(node_times, limit);
    (reachable as f64 / node_times.len() as f64) >= SATURATED_REUSE_RATIO
}

fn convex_hull_from_points(points: Vec<(f64, f64)>) -> Polygon {
    if points.len() < 3 {
        return empty_polygon();
    }

    let points: MultiPoint<f64> = points.into();
    points.convex_hull()
}

fn empty_polygon() -> Polygon {
    Polygon::new(LineString::new(vec![]), vec![])
}

fn upper_bound_node_times(node_times: &[(NodeIndex, f64)], time: f64) -> usize {
    node_times.partition_point(|(_, candidate)| *candidate <= time)
}

/// A reached node in a local metric projection, with its travel time.
#[derive(Clone, Copy)]
struct IsoVertex {
    x: f64,
    y: f64,
    lat: f64,
    lon: f64,
    time: f64,
}

/// Travel time as a piecewise-linear surface over a Delaunay triangulation
/// of the reached nodes; isochrones are its level lines.
struct TriangulatedSurface {
    vertices: Vec<IsoVertex>,
    /// Vertex indices, three per triangle.
    triangles: Vec<usize>,
}

#[derive(Clone, Copy)]
struct ContourPoint {
    x: f64,
    y: f64,
    lat: f64,
    lon: f64,
}

#[derive(Clone, Copy)]
struct ContourSegment {
    from: ContourPoint,
    to: ContourPoint,
}

type ContourKey = (i64, i64);

fn build_triangulated_isochrones(
    graph: &RoadGraph,
    node_times: &[(NodeIndex, f64)],
    time_limits: &[f64],
    max_seen: f64,
) -> Vec<Polygon> {
    let hull_of_everything = || {
        convex_hull_from_points(
            node_times
                .iter()
                .map(|(node, _)| graph::node_to_latlon(graph, *node))
                .collect(),
        )
    };

    let Some(surface) = TriangulatedSurface::from_graph_times(graph, node_times) else {
        let hull = hull_of_everything();
        return time_limits.iter().map(|_| hull.clone()).collect();
    };

    // A limit that (nearly) every reached node satisfies gets the hull of all
    // reached nodes; it is computed at most once and shared.
    let saturated: Vec<bool> = time_limits
        .iter()
        .map(|&limit| limit >= max_seen || is_saturated_limit(node_times, limit))
        .collect();
    let saturated_polygon = saturated.contains(&true).then(hull_of_everything);

    time_limits
        .iter()
        .zip(&saturated)
        .map(|(&limit, &is_saturated)| match &saturated_polygon {
            Some(polygon) if is_saturated => polygon.clone(),
            _ => surface.contour_polygon(limit),
        })
        .collect()
}

impl TriangulatedSurface {
    fn from_graph_times(graph: &RoadGraph, node_times: &[(NodeIndex, f64)]) -> Option<Self> {
        if node_times.len() < 3 {
            return None;
        }

        let origin_lat = node_times
            .iter()
            .map(|(node, _)| graph[*node].lat)
            .sum::<f64>()
            / node_times.len() as f64;
        let cos_lat = origin_lat.to_radians().cos();
        let mut seen = HashSet::new();
        let mut vertices = Vec::with_capacity(node_times.len());

        for &(node, time) in node_times {
            let osm_node = &graph[node];
            let x = osm_node.lon * 111_320.0 * cos_lat;
            let y = osm_node.lat * 111_320.0;
            let key = (
                (x * CONTOUR_KEY_SCALE) as i64,
                (y * CONTOUR_KEY_SCALE) as i64,
            );
            if seen.insert(key) {
                vertices.push(IsoVertex {
                    x,
                    y,
                    lat: osm_node.lat,
                    lon: osm_node.lon,
                    time,
                });
            }
        }

        if vertices.len() < 3 {
            return None;
        }

        let points: Vec<delaunator::Point> = vertices
            .iter()
            .map(|v| delaunator::Point { x: v.x, y: v.y })
            .collect();
        let triangles = delaunator::triangulate(&points).triangles;
        Some(Self {
            vertices,
            triangles,
        })
    }

    fn contour_polygon(&self, limit: f64) -> Polygon {
        let segments = self.contour_segments(limit);
        if segments.is_empty() {
            return empty_polygon();
        }

        if let Some(ring) = largest_closed_ring(&segments) {
            return Polygon::new(LineString::from(ring), vec![]);
        }

        let mut points = Vec::with_capacity(segments.len() * 2);
        for segment in segments {
            points.push((segment.from.lat, segment.from.lon));
            points.push((segment.to.lat, segment.to.lon));
        }
        convex_hull_from_points(points)
    }

    fn contour_segments(&self, limit: f64) -> Vec<ContourSegment> {
        self.triangles
            .chunks_exact(3)
            .filter_map(|t| {
                let triangle = [t[0], t[1], t[2]].map(|i| self.vertices[i]);
                triangle_contour_segment(triangle, limit)
            })
            .collect()
    }
}

fn triangle_contour_segment(vertices: [IsoVertex; 3], limit: f64) -> Option<ContourSegment> {
    let edges = [
        (vertices[0], vertices[1]),
        (vertices[1], vertices[2]),
        (vertices[2], vertices[0]),
    ];
    // A level line crosses a triangle on exactly zero or two of its edges.
    let mut crossings = edges.into_iter().filter_map(|(from, to)| {
        let crosses = (from.time <= limit) != (to.time <= limit);
        crosses.then(|| {
            let ratio = ((limit - from.time) / (to.time - from.time)).clamp(0.0, 1.0);
            interpolate_contour_point(from, to, ratio)
        })
    });

    let (from, to) = (crossings.next()?, crossings.next()?);
    (crossings.next().is_none() && contour_key(from) != contour_key(to))
        .then_some(ContourSegment { from, to })
}

fn interpolate_contour_point(from: IsoVertex, to: IsoVertex, ratio: f64) -> ContourPoint {
    ContourPoint {
        x: from.x + (to.x - from.x) * ratio,
        y: from.y + (to.y - from.y) * ratio,
        lat: from.lat + (to.lat - from.lat) * ratio,
        lon: from.lon + (to.lon - from.lon) * ratio,
    }
}

/// Stitch contour segments into closed rings and return the one enclosing the
/// largest area, as `(lat, lon)` points with the first point repeated last.
///
/// Segment endpoints are snapped to a 10 cm grid and interned to dense ids, so
/// the walk is plain `Vec` indexing. Starting edges are tried in segment
/// order, which keeps the choice of ring deterministic.
fn largest_closed_ring(segments: &[ContourSegment]) -> Option<Vec<(f64, f64)>> {
    let mut ids: HashMap<ContourKey, usize> = HashMap::with_capacity(segments.len());
    let mut points: Vec<ContourPoint> = Vec::with_capacity(segments.len());
    let mut intern = |point: ContourPoint| {
        *ids.entry(contour_key(point)).or_insert_with(|| {
            points.push(point);
            points.len() - 1
        })
    };

    // Undirected, deduplicated edges between interned points.
    let mut edges: Vec<(usize, usize)> = segments
        .iter()
        .filter_map(|segment| {
            let (a, b) = (intern(segment.from), intern(segment.to));
            (a != b).then_some((a.min(b), a.max(b)))
        })
        .collect();
    {
        let mut seen = HashSet::with_capacity(edges.len());
        edges.retain(|edge| seen.insert(*edge));
    }

    let mut adjacency: Vec<Vec<(usize, usize)>> = vec![Vec::new(); points.len()];
    for (edge, &(a, b)) in edges.iter().enumerate() {
        adjacency[a].push((b, edge));
        adjacency[b].push((a, edge));
    }

    let mut used = vec![false; edges.len()];
    let mut ring: Vec<usize> = Vec::new();
    let mut best_ring: Option<Vec<(f64, f64)>> = None;
    let mut best_area = 0.0;

    for first_edge in 0..edges.len() {
        if used[first_edge] {
            continue;
        }
        used[first_edge] = true;
        let (start, mut current) = edges[first_edge];
        let mut previous = start;
        ring.clear();
        ring.extend([start, current]);

        while current != start {
            let next = adjacency[current]
                .iter()
                .find(|&&(neighbor, edge)| neighbor != previous && !used[edge]);
            let Some(&(neighbor, edge)) = next else {
                break;
            };
            used[edge] = true;
            ring.push(neighbor);
            previous = current;
            current = neighbor;
        }

        if current == start && ring.len() >= 4 {
            let ring_points: Vec<ContourPoint> = ring.iter().map(|&id| points[id]).collect();
            let area = projected_ring_area(&ring_points).abs();
            if area > best_area {
                best_area = area;
                best_ring = Some(ring_points.iter().map(|p| (p.lat, p.lon)).collect());
            }
        }
    }

    best_ring
}

fn contour_key(point: ContourPoint) -> ContourKey {
    (
        (point.x * CONTOUR_KEY_SCALE).round() as i64,
        (point.y * CONTOUR_KEY_SCALE).round() as i64,
    )
}

fn projected_ring_area(ring: &[ContourPoint]) -> f64 {
    if ring.len() < 4 {
        return 0.0;
    }

    ring.windows(2)
        .map(|pair| pair[0].x * pair[1].y - pair[1].x * pair[0].y)
        .sum::<f64>()
        * 0.5
}

/// Isochrones for every limit from one reachability search sized to the
/// largest limit: one search and one triangulation serve all limits.
pub fn isochrones_from_node(
    sg: &SpatialGraph,
    start_node: NodeIndex,
    time_limits: &[f64],
    network_type: NetworkType,
) -> Vec<Polygon> {
    let max_cost = time_limits.iter().copied().fold(0.0_f64, f64::max);
    let result = compute_reachability(sg, start_node, max_cost, network_type);
    build_isochrone_polygons(&sg.graph, &result, time_limits)
}

impl SpatialGraph {
    /// Build isochrone polygons for one or more time limits from a lat/lon origin.
    ///
    /// Each polygon encloses all nodes reachable within the corresponding time
    /// limit. The returned `Vec` is in the same order as `time_limits`.
    ///
    /// Returns `None` if no graph node is found near `(lat, lon)`.
    pub fn isochrones(
        &self,
        lat: f64,
        lon: f64,
        time_limits: Vec<f64>,
        network_type: NetworkType,
        max_snap_m: Option<f64>,
    ) -> Option<Vec<Polygon>> {
        let start_node = self.nearest_node_within(lat, lon, max_snap_m)?;
        Some(isochrones_from_node(
            self,
            start_node,
            &time_limits,
            network_type,
        ))
    }
}

#[cfg(feature = "extension-module")]
pub(crate) async fn calculate_isochrones_from_point(
    lat: f64,
    lon: f64,
    max_dist: Option<f64>,
    time_limits: Vec<f64>,
    network_type: overpass::NetworkType,
    retain_all: bool,
) -> Result<(Vec<Polygon>, SpatialGraph), OsmGraphError> {
    use crate::cache;

    // Auto-size bounding box if not provided.
    // Use max time limit * a generous speed + 20% buffer to ensure the
    // isochrone never saturates into a square at the bbox boundary.
    let max_speed_m_per_s = match network_type {
        NetworkType::Walk => 5.0 / 3.6,
        NetworkType::Bike => 25.0 / 3.6,
        NetworkType::Drive
        | NetworkType::DriveService
        | NetworkType::All
        | NetworkType::AllPrivate => 120.0 / 3.6,
    };
    let max_time = time_limits.iter().cloned().fold(0.0_f64, f64::max);
    let computed_dist = max_dist.unwrap_or(max_time * max_speed_m_per_s * 1.2);

    let polygon_coord_str = overpass::bbox_from_point(lat, lon, computed_dist);
    let query = overpass::create_overpass_query(&polygon_coord_str, network_type);

    let xml = if let Some(cached_xml) = cache::check_xml_cache(&query)? {
        cached_xml // in-memory hit
    } else if let Some(disk_xml) = cache::check_disk_xml_cache(&query) {
        cache::insert_into_xml_cache(query.clone(), disk_xml.clone())?; // promote to memory
        disk_xml // disk hit
    } else {
        let fetched = overpass::make_request(&overpass::overpass_url(), &query).await?;
        cache::write_disk_xml_cache(&query, &fetched); // persist to disk (best-effort)
        cache::insert_into_xml_cache(query.clone(), fetched.clone())?;
        fetched // network fetch
    };
    let parsed = graph::parse_xml(&xml)?;
    if parsed.nodes.is_empty() {
        return Err(OsmGraphError::EmptyGraph);
    }
    let bidirectional = matches!(network_type, NetworkType::Walk);
    let sg = SpatialGraph::new(graph::create_graph(
        parsed.nodes,
        parsed.ways,
        retain_all,
        bidirectional,
    ));

    let node_index = sg
        .nearest_node(lat, lon)
        .ok_or(OsmGraphError::NodeNotFound)?;
    let isochrones = isochrones_from_node(&sg, node_index, &time_limits, network_type);

    Ok((isochrones, sg))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Edge, NodeMap, XmlNode};
    use crate::reachability::compute_reachability;
    use geo::Area;
    use petgraph::graph::DiGraph;

    fn point(x: f64, y: f64) -> ContourPoint {
        ContourPoint {
            x,
            y,
            lat: y,
            lon: x,
        }
    }

    #[test]
    fn contour_segments_stitch_into_closed_ring() {
        let segments = vec![
            ContourSegment {
                from: point(0.0, 0.0),
                to: point(1.0, 0.0),
            },
            ContourSegment {
                from: point(1.0, 0.0),
                to: point(1.0, 1.0),
            },
            ContourSegment {
                from: point(1.0, 1.0),
                to: point(0.0, 1.0),
            },
            ContourSegment {
                from: point(0.0, 1.0),
                to: point(0.0, 0.0),
            },
        ];

        let ring = largest_closed_ring(&segments).expect("square should close");
        assert_eq!(ring.first(), ring.last());
        assert_eq!(ring.len(), 5);
    }

    fn make_node(id: i64, lat: f64, lon: f64) -> XmlNode {
        XmlNode {
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
        (SpatialGraph::new(graph), center)
    }

    #[test]
    fn empty_reachability_returns_empty_polygons_in_input_order() {
        let graph = RoadGraph::new();
        let result = ReachabilityResult {
            start: NodeIndex::new(0),
            max_cost: 0.0,
            times: NodeMap::with_node_count(0),
        };

        let polygons = build_isochrone_polygons(&graph, &result, &[60.0, 30.0]);

        assert_eq!(polygons.len(), 2);
        assert!(polygons
            .iter()
            .all(|polygon| polygon.exterior().0.is_empty()));
    }

    #[test]
    fn fewer_than_three_reachable_nodes_returns_empty_polygon() {
        let mut graph = DiGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.001, 0.0));
        graph.add_edge(a, b, make_way(10.0));
        let sg = SpatialGraph::new(graph);
        let result = compute_reachability(&sg, a, 10.0, NetworkType::Drive);

        let polygons = build_isochrone_polygons(&sg.graph, &result, &[10.0]);

        assert!(polygons[0].exterior().0.is_empty());
    }

    #[test]
    fn isochrone_output_order_matches_input_limits() {
        let (graph, start) = square_graph();

        let polygons = isochrones_from_node(&graph, start, &[20.0, 5.0], NetworkType::Drive);

        assert_eq!(polygons.len(), 2);
        assert!(polygons[0].unsigned_area() > polygons[1].unsigned_area());
    }

    #[test]
    fn increasing_time_limits_have_non_decreasing_area() {
        let (graph, start) = square_graph();

        let polygons = isochrones_from_node(&graph, start, &[5.0, 20.0], NetworkType::Drive);

        assert!(polygons[0].unsigned_area() <= polygons[1].unsigned_area());
    }
}
