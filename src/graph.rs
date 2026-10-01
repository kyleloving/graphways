use crate::overpass::NetworkType;
use crate::simplify::simplify_graph;
use crate::utils::{calculate_distance, calculate_travel_time};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

#[derive(Debug, Deserialize)]
pub struct XmlData {
    #[serde(rename = "node", default)]
    pub nodes: Vec<XmlNode>,
    #[serde(rename = "way", default)]
    pub ways: Vec<XmlWay>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct XmlNode {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "@lat")]
    pub lat: f64,
    #[serde(rename = "@lon")]
    pub lon: f64,
    #[serde(rename = "tag", default)]
    pub tags: Vec<XmlTag>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct XmlWay {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "nd", default)]
    pub nodes: Vec<XmlNodeRef>,
    #[serde(rename = "tag", default)]
    pub tags: Vec<XmlTag>,
    #[serde(default)]
    pub length: f64,
    #[serde(default)]
    pub speed_kph: f64,
    #[serde(default)]
    pub walk_travel_time: f64,
    #[serde(default)]
    pub bike_travel_time: f64,
    #[serde(default)]
    pub drive_travel_time: f64,
    /// Ordered route geometry as `(lat, lon)` points for this directed edge.
    ///
    /// Unsimplified edges contain their source and target coordinates.
    /// Simplified edges retain the intermediate shape points from the collapsed
    /// chain so route rendering follows the original road geometry.
    #[serde(default)]
    pub geometry: Vec<(f64, f64)>,
}

impl XmlWay {
    pub fn filter_useful_tags(mut self) -> Self {
        const USEFUL_TAGS: &[&str] = &["highway", "name", "ref", "bridge", "tunnel", "service"];
        // Linear search on 15-element static slice — no HashSet allocation needed.
        self.tags
            .retain(|tag| USEFUL_TAGS.contains(&tag.key.as_str()));
        self
    }

    /// Return the travel time (seconds) for the given network type.
    /// Centralises the walk / bike / drive dispatch so call sites don't
    /// repeat the same match expression.
    #[inline]
    pub fn travel_time(&self, network_type: NetworkType) -> f64 {
        match CostField::of(network_type) {
            CostField::Walk => self.walk_travel_time,
            CostField::Bike => self.bike_travel_time,
            CostField::Drive => self.drive_travel_time,
        }
    }

    /// This edge's route geometry, oriented from `source` to `target`.
    ///
    /// Falls back to the straight segment between the two nodes when the edge
    /// carries no shape points, and flips stored geometry that runs backwards.
    /// Borrows the stored points; nothing is allocated.
    pub fn oriented_geometry(&self, source: &XmlNode, target: &XmlNode) -> EdgeGeometry<'_> {
        let (start, end) = ((source.lat, source.lon), (target.lat, target.lon));
        let [first, .., last] = self.geometry.as_slice() else {
            return EdgeGeometry {
                points: GeometryPoints::Straight([start, end]),
                reversed: false,
            };
        };
        let span = |a: (f64, f64), b: (f64, f64)| calculate_distance(a.0, a.1, b.0, b.1);
        let reversed =
            span(*first, start) + span(*last, end) > span(*first, end) + span(*last, start);
        EdgeGeometry {
            points: GeometryPoints::Stored(&self.geometry),
            reversed,
        }
    }
}

/// Which precomputed travel-time field a [`NetworkType`] is costed with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CostField {
    Walk,
    Bike,
    Drive,
}

impl CostField {
    pub(crate) const ALL: [CostField; 3] = [CostField::Walk, CostField::Bike, CostField::Drive];

    #[inline]
    pub(crate) fn of(network_type: NetworkType) -> Self {
        match network_type {
            NetworkType::Walk => CostField::Walk,
            NetworkType::Bike => CostField::Bike,
            NetworkType::Drive
            | NetworkType::DriveService
            | NetworkType::All
            | NetworkType::AllPrivate => CostField::Drive,
        }
    }
}

/// Route geometry of one edge, oriented along the edge. See
/// [`XmlWay::oriented_geometry`].
#[derive(Debug, Clone, Copy)]
pub struct EdgeGeometry<'a> {
    points: GeometryPoints<'a>,
    reversed: bool,
}

#[derive(Debug, Clone, Copy)]
enum GeometryPoints<'a> {
    Stored(&'a [(f64, f64)]),
    Straight([(f64, f64); 2]),
}

impl EdgeGeometry<'_> {
    fn as_slice(&self) -> &[(f64, f64)] {
        match &self.points {
            GeometryPoints::Stored(points) => points,
            GeometryPoints::Straight(points) => points,
        }
    }

    /// `(lat, lon)` points from the edge's source to its target (at least 2).
    pub fn points(&self) -> impl DoubleEndedIterator<Item = (f64, f64)> + ExactSizeIterator + '_ {
        let points = self.as_slice();
        let last = points.len().saturating_sub(1);
        let reversed = self.reversed;
        (0..points.len()).map(move |i| points[if reversed { last - i } else { i }])
    }
}

/// Oriented geometry of `edge` in `graph`. Panics if `edge` is not in `graph`.
pub fn edge_geometry(graph: &DiGraph<XmlNode, XmlWay>, edge: EdgeIndex) -> EdgeGeometry<'_> {
    let (source, target) = graph
        .edge_endpoints(edge)
        .expect("edge index belongs to this graph");
    graph[edge].oriented_geometry(&graph[source], &graph[target])
}

#[derive(Debug, Deserialize, Clone)]
pub struct XmlNodeRef {
    #[serde(rename = "@ref")]
    pub node_id: i64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct XmlTag {
    #[serde(rename = "@k")]
    pub key: String,
    #[serde(rename = "@v")]
    pub value: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    Bidirectional,
    OneWayForward,
    OneWayReverse,
}

impl Direction {
    /// Edges to emit for each way segment, as "is this the reversed copy?"
    /// flags, primary (tagged) direction first. `force_bidirectional` adds
    /// the contraflow copy for profiles that ignore one-way restrictions.
    fn traversals(self, force_bidirectional: bool) -> &'static [bool] {
        const FORWARD: bool = false;
        const REVERSE: bool = true;
        match (self, force_bidirectional) {
            (Direction::Bidirectional, _) | (Direction::OneWayForward, true) => &[FORWARD, REVERSE],
            (Direction::OneWayForward, false) => &[FORWARD],
            (Direction::OneWayReverse, true) => &[REVERSE, FORWARD],
            (Direction::OneWayReverse, false) => &[REVERSE],
        }
    }
}

// Function to parse the XML response
pub fn parse_xml(xml_data: &str) -> Result<XmlData, quick_xml::DeError> {
    let root: XmlData = quick_xml::de::from_str(xml_data)?;
    Ok(root)
}

fn find_tag<'a>(tags: &'a [XmlTag], key: &str) -> Option<&'a XmlTag> {
    tags.iter().find(|tag| tag.key == key)
}

fn assess_path_directionality(path: &XmlWay) -> Direction {
    let oneway_tag = find_tag(&path.tags, "oneway");
    let junction_tag = find_tag(&path.tags, "junction");

    if oneway_tag.is_some_and(|tag| matches!(tag.value.as_str(), "-1" | "reverse")) {
        return Direction::OneWayReverse;
    }

    if oneway_tag.is_some_and(|tag| matches!(tag.value.as_str(), "yes" | "true" | "1")) {
        return Direction::OneWayForward;
    }

    // Roundabouts are considered one-way implicitly
    let is_roundabout = junction_tag.is_some_and(|tag| tag.value == "roundabout");
    if is_roundabout {
        Direction::OneWayForward
    } else {
        Direction::Bidirectional
    }
}

fn highway_speed_kph(highway: &str) -> Option<f64> {
    match highway {
        "motorway" => Some(110.0),
        "motorway_link" => Some(60.0),
        "trunk" => Some(90.0),
        "trunk_link" => Some(45.0),
        "primary" => Some(65.0),
        "primary_link" => Some(45.0),
        "secondary" => Some(55.0),
        "secondary_link" => Some(40.0),
        "tertiary" => Some(45.0),
        "tertiary_link" => Some(35.0),
        "unclassified" => Some(45.0),
        "residential" => Some(30.0),
        "living_street" => Some(10.0),
        "service" => Some(20.0),
        "track" => Some(20.0),
        "road" => Some(50.0),
        _ => None,
    }
}

fn way_speed_kph(way: &XmlWay) -> f64 {
    const FALLBACK_SPEED_KPH: f64 = 50.0;

    if let Some(maxspeed) =
        find_tag(&way.tags, "maxspeed").and_then(|tag| clean_maxspeed(&tag.value))
    {
        return maxspeed;
    }

    find_tag(&way.tags, "highway")
        .and_then(|tag| highway_speed_kph(&tag.value))
        .unwrap_or(FALLBACK_SPEED_KPH)
}

fn edge_way_from_template(
    template: &XmlWay,
    length: f64,
    speed_kph: f64,
    geometry: Vec<(f64, f64)>,
) -> XmlWay {
    XmlWay {
        id: template.id,
        nodes: Vec::new(),
        tags: template.tags.clone(),
        length,
        speed_kph,
        walk_travel_time: calculate_travel_time(length, 5.0),
        bike_travel_time: calculate_travel_time(length, 15.0),
        drive_travel_time: calculate_travel_time(length, speed_kph),
        geometry,
    }
}

/// Build a directed road graph from parsed OSM nodes and ways.
///
/// Every consecutive node pair of a way becomes one edge per traversable
/// direction. Way references to nodes missing from `nodes` (common in clipped
/// extracts) are skipped rather than panicking. Unless `retain_all` is set the
/// graph is then simplified: nearby intersection nodes are merged and
/// degree-two chains are collapsed into single edges.
pub fn create_graph(
    nodes: Vec<XmlNode>,
    ways: Vec<XmlWay>,
    retain_all: bool,
    bidirectional: bool,
) -> DiGraph<XmlNode, XmlWay> {
    let segment_count: usize = ways.iter().map(|w| w.nodes.len().saturating_sub(1)).sum();
    let mut graph = DiGraph::with_capacity(nodes.len(), segment_count * 2);
    let mut node_index_map = HashMap::with_capacity(nodes.len());

    for node in nodes {
        let id = node.id;
        node_index_map.insert(id, graph.add_node(node));
    }

    for mut way in ways {
        // Edge weights don't need the construction-only node list.
        let node_refs = std::mem::take(&mut way.nodes);
        let traversals = assess_path_directionality(&way).traversals(bidirectional);
        let speed_kph = way_speed_kph(&way);
        let way = way.filter_useful_tags();

        for pair in node_refs.windows(2) {
            let (Some(&a), Some(&b)) = (
                node_index_map.get(&pair[0].node_id),
                node_index_map.get(&pair[1].node_id),
            ) else {
                continue;
            };
            let (pa, pb) = (node_to_latlon(&graph, a), node_to_latlon(&graph, b));
            let length = calculate_distance(pa.0, pa.1, pb.0, pb.1);

            for &reversed in traversals {
                let (from, to, geometry) = if reversed {
                    (b, a, vec![pb, pa])
                } else {
                    (a, b, vec![pa, pb])
                };
                graph.add_edge(
                    from,
                    to,
                    edge_way_from_template(&way, length, speed_kph, geometry),
                );
            }
        }
    }

    if retain_all {
        graph
    } else {
        simplify_graph(graph)
    }
}

/// Parse an OSM `maxspeed` value ("50", "30 mph", "50;30") into km/h.
fn clean_maxspeed(maxspeed: &str) -> Option<f64> {
    const MPH_TO_KPH: f64 = 1.60934;
    let trimmed = maxspeed.trim();
    let numeric_len = trimmed
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(trimmed.len());
    let speed = trimmed[..numeric_len].parse::<f64>().ok()?;
    if speed <= 0.0 {
        return None;
    }

    let is_mph = trimmed
        .as_bytes()
        .windows(3)
        .any(|w| w.eq_ignore_ascii_case(b"mph"));
    Some(if is_mph { speed * MPH_TO_KPH } else { speed })
}

pub fn node_to_latlon(graph: &DiGraph<XmlNode, XmlWay>, node_index: NodeIndex) -> (f64, f64) {
    let node = &graph[node_index];
    (node.lat, node.lon)
}

/// R-tree entry pairing a node's projected coordinates with its NodeIndex.
#[derive(Clone, Copy)]
pub(crate) struct NodeEntry {
    pub(crate) point: [f64; 2],
    pub(crate) index: NodeIndex,
}

impl NodeEntry {
    pub(crate) fn new(node: &XmlNode, index: NodeIndex) -> Self {
        Self {
            point: spatial_index_point(node.lat, node.lon),
            index,
        }
    }
}

impl RTreeObject for NodeEntry {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.point)
    }
}

/// Local equirectangular projection to metres, good enough for nearest-node
/// queries and small-radius clustering.
pub(crate) fn spatial_index_point(lat: f64, lon: f64) -> [f64; 2] {
    const METERS_PER_DEGREE: f64 = 111_320.0;
    [
        lat * METERS_PER_DEGREE,
        lon * METERS_PER_DEGREE * lat.to_radians().cos(),
    ]
}

impl PointDistance for NodeEntry {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dlat = self.point[0] - point[0];
        let dlon = self.point[1] - point[1];
        dlat * dlat + dlon * dlon
    }
}

/// Per cost field, the largest `straight-line distance / travel time` over all
/// edges. Because great-circle distance obeys the triangle inequality,
/// `distance(n, goal) / max_speed` never exceeds the true remaining cost, and
/// the resulting A* heuristic is consistent even for edges whose endpoints
/// were moved by intersection consolidation.
fn max_straight_line_speeds(graph: &DiGraph<XmlNode, XmlWay>) -> [f64; 3] {
    let mut max_speed = [0.0_f64; 3];
    for edge in graph.edge_references() {
        let (a, b) = (&graph[edge.source()], &graph[edge.target()]);
        let distance = calculate_distance(a.lat, a.lon, b.lat, b.lon);
        if distance == 0.0 {
            continue;
        }
        for field in CostField::ALL {
            let time = match field {
                CostField::Walk => edge.weight().walk_travel_time,
                CostField::Bike => edge.weight().bike_travel_time,
                CostField::Drive => edge.weight().drive_travel_time,
            };
            if time.is_finite() && time >= 0.0 {
                let speed = if time > 0.0 {
                    distance / time
                } else {
                    f64::INFINITY
                };
                max_speed[field as usize] = max_speed[field as usize].max(speed);
            }
        }
    }
    // Pad by a hair so floating-point rounding can't make the bound inadmissible.
    max_speed.map(|speed| speed * (1.0 + 1e-9))
}

#[derive(Debug, Clone, Copy)]
pub struct SnapResult {
    pub input_lat: f64,
    pub input_lon: f64,
    pub node_index: NodeIndex,
    pub node_id: i64,
    pub node_lat: f64,
    pub node_lon: f64,
    pub distance_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SnappedPoi {
    pub poi_id: i64,
    pub snap: SnapResult,
}

/// A graph bundled with a spatial index for O(log n) nearest-node queries
/// and an optional pre-computed POI snap map for O(1) POI filtering.
///
/// Build once via `SpatialGraph::new`, reuse for all queries. All inner
/// fields are reference-counted so cloning is O(1).
#[derive(Clone)]
pub struct SpatialGraph {
    pub graph: Arc<DiGraph<XmlNode, XmlWay>>,
    tree: Arc<RTree<NodeEntry>>,
    /// POI OSM node id → snapped graph node diagnostics, computed once at startup via
    /// `snap_pois`. `None` until called; `Some` map used by POI filtering
    /// for O(1) lookup instead of an R-tree query on every request.
    pub poi_snaps: Option<Arc<HashMap<i64, SnappedPoi>>>,
    /// Upper bound on straight-line speed (m/s) over any edge, per cost
    /// field, computed on first use by routing. See
    /// [`SpatialGraph::max_straight_line_speed`].
    max_speed_mps: Arc<OnceLock<[f64; 3]>>,
}

impl SpatialGraph {
    pub fn new(graph: DiGraph<XmlNode, XmlWay>) -> Self {
        let entries: Vec<NodeEntry> = graph
            .node_indices()
            .map(|i| NodeEntry::new(&graph[i], i))
            .collect();
        let tree = Arc::new(RTree::bulk_load(entries));
        Self {
            graph: Arc::new(graph),
            tree,
            poi_snaps: None,
            max_speed_mps: Arc::default(),
        }
    }

    /// The fastest straight-line speed (m/s) any edge allows for
    /// `network_type`: `straight_line_distance / max_straight_line_speed`
    /// never exceeds the true travel time between two nodes, which makes it
    /// an admissible and consistent A* heuristic. Computed once, lazily, so
    /// graphs that never route don't pay for it.
    pub(crate) fn max_straight_line_speed(&self, network_type: NetworkType) -> f64 {
        self.max_speed_mps
            .get_or_init(|| max_straight_line_speeds(&self.graph))
            [CostField::of(network_type) as usize]
    }

    /// Copy the subgraph induced by the nodes for which `keep` returns true.
    ///
    /// Node and edge order follow the parent graph, so results are
    /// deterministic. POI snaps are not carried over.
    pub fn induced_subgraph(&self, mut keep: impl FnMut(NodeIndex) -> bool) -> SpatialGraph {
        let mut remap = vec![NodeIndex::end(); self.graph.node_count()];
        let mut subgraph = DiGraph::new();
        for index in self.graph.node_indices() {
            if keep(index) {
                remap[index.index()] = subgraph.add_node(self.graph[index].clone());
            }
        }
        for edge in self.graph.edge_references() {
            let (source, target) = (remap[edge.source().index()], remap[edge.target().index()]);
            if source != NodeIndex::end() && target != NodeIndex::end() {
                subgraph.add_edge(source, target, edge.weight().clone());
            }
        }
        SpatialGraph::new(subgraph)
    }

    pub(crate) fn from_parsed_osm(
        data: XmlData,
        network_type: NetworkType,
        retain_all: bool,
    ) -> Self {
        let bidirectional = matches!(network_type, NetworkType::Walk);
        let graph = create_graph(data.nodes, data.ways, retain_all, bidirectional);
        Self::new(graph)
    }

    /// Parse an OSM XML response and build a [`SpatialGraph`].
    pub fn from_osm(
        xml: &str,
        network_type: NetworkType,
        retain_all: Option<bool>,
    ) -> Result<Self, quick_xml::DeError> {
        let data = parse_xml(xml)?;
        Ok(Self::from_parsed_osm(
            data,
            network_type,
            retain_all.unwrap_or(false),
        ))
    }

    /// Pre-snap a set of POI nodes to their nearest graph nodes, storing the
    /// result for O(1) lookup at request time.
    ///
    /// `pois` is the POI list returned by [`crate::pbf::read_pbf`]. Call once
    /// at startup after `new`.
    pub fn snap_pois(&mut self, pois: &[crate::poi::Poi]) {
        let snaps: HashMap<i64, SnappedPoi> = pois
            .iter()
            .filter_map(|poi| {
                self.snap_point(poi.lat, poi.lon).map(|snap| {
                    (
                        poi.id,
                        SnappedPoi {
                            poi_id: poi.id,
                            snap,
                        },
                    )
                })
            })
            .collect();
        self.poi_snaps = Some(Arc::new(snaps));
    }

    pub fn nearest_node(&self, lat: f64, lon: f64) -> Option<NodeIndex> {
        self.tree
            .nearest_neighbor(&spatial_index_point(lat, lon))
            .map(|e| e.index)
    }

    pub fn snap_point(&self, lat: f64, lon: f64) -> Option<SnapResult> {
        self.nearest_node(lat, lon).map(|node_index| {
            let node = &self.graph[node_index];
            SnapResult {
                input_lat: lat,
                input_lon: lon,
                node_index,
                node_id: node.id,
                node_lat: node.lat,
                node_lon: node.lon,
                distance_m: calculate_distance(lat, lon, node.lat, node.lon),
            }
        })
    }

    /// Snap a coordinate to the nearest graph node, optionally rejecting snaps
    /// farther than `max_distance_m`.
    pub fn snap_point_within(
        &self,
        lat: f64,
        lon: f64,
        max_distance_m: Option<f64>,
    ) -> Option<SnapResult> {
        let snap = self.snap_point(lat, lon)?;
        match max_distance_m {
            Some(max_distance_m) if snap.distance_m > max_distance_m => None,
            _ => Some(snap),
        }
    }

    pub fn nearest_node_within(
        &self,
        lat: f64,
        lon: f64,
        max_distance_m: Option<f64>,
    ) -> Option<NodeIndex> {
        self.snap_point_within(lat, lon, max_distance_m)
            .map(|snap| snap.node_index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use petgraph::visit::EdgeRef;

    fn make_node(id: i64, lat: f64, lon: f64) -> XmlNode {
        XmlNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn make_way_raw(node_ids: Vec<i64>, tags: Vec<(&str, &str)>) -> XmlWay {
        XmlWay {
            id: 1,
            nodes: node_ids
                .into_iter()
                .map(|id| XmlNodeRef { node_id: id })
                .collect(),
            tags: tags
                .into_iter()
                .map(|(k, v)| XmlTag {
                    key: k.into(),
                    value: v.into(),
                })
                .collect(),
            length: 0.0,
            speed_kph: 0.0,
            walk_travel_time: 0.0,
            bike_travel_time: 0.0,
            drive_travel_time: 0.0,
            geometry: Vec::new(),
        }
    }

    fn edge_id_pairs(graph: &DiGraph<XmlNode, XmlWay>) -> Vec<(i64, i64)> {
        let mut pairs: Vec<(i64, i64)> = graph
            .edge_references()
            .map(|edge| (graph[edge.source()].id, graph[edge.target()].id))
            .collect();
        pairs.sort_unstable();
        pairs
    }

    #[test]
    fn test_graph_respects_maxspeed_tag() {
        let nodes = [make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("maxspeed", "30")],
        );
        let graph = create_graph(
            vec![nodes[0].clone(), nodes[1].clone()],
            vec![way],
            true,
            false,
        );
        assert_eq!(graph.edge_weights().next().unwrap().speed_kph, 30.0);
    }

    #[test]
    fn test_graph_parses_mph_maxspeed_tag() {
        let nodes = [make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("maxspeed", "30 mph")],
        );
        let graph = create_graph(
            vec![nodes[0].clone(), nodes[1].clone()],
            vec![way],
            true,
            false,
        );
        let speed = graph.edge_weights().next().unwrap().speed_kph;
        assert!((speed - 48.2802).abs() < 1e-4);
    }

    #[test]
    fn test_graph_falls_back_when_maxspeed_is_non_numeric() {
        let nodes = [make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("maxspeed", "signals")],
        );
        let graph = create_graph(
            vec![nodes[0].clone(), nodes[1].clone()],
            vec![way],
            true,
            false,
        );
        assert_eq!(graph.edge_weights().next().unwrap().speed_kph, 30.0);
    }

    #[test]
    fn test_oneway_produces_single_edge() {
        let nodes = [make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("oneway", "yes")],
        );
        let graph = create_graph(
            vec![nodes[0].clone(), nodes[1].clone()],
            vec![way],
            true,
            false,
        );
        assert_eq!(graph.edge_count(), 1);
    }

    #[test]
    fn test_oneway_yes_points_in_way_order() {
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("oneway", "yes")],
        );

        let graph = create_graph(nodes, vec![way], true, false);

        assert_eq!(edge_id_pairs(&graph), vec![(1, 2)]);
    }

    #[test]
    fn test_oneway_reverse_points_against_way_order() {
        for value in ["-1", "reverse"] {
            let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
            let way = make_way_raw(
                vec![1, 2],
                vec![("highway", "residential"), ("oneway", value)],
            );

            let graph = create_graph(nodes, vec![way], true, false);

            assert_eq!(edge_id_pairs(&graph), vec![(2, 1)], "oneway={value}");
        }
    }

    #[test]
    fn test_oneway_truthy_values_point_in_way_order() {
        for value in ["true", "1"] {
            let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
            let way = make_way_raw(
                vec![1, 2],
                vec![("highway", "residential"), ("oneway", value)],
            );

            let graph = create_graph(nodes, vec![way], true, false);

            assert_eq!(edge_id_pairs(&graph), vec![(1, 2)], "oneway={value}");
        }
    }

    #[test]
    fn test_roundabout_is_oneway_forward() {
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("junction", "roundabout")],
        );

        let graph = create_graph(nodes, vec![way], true, false);

        assert_eq!(edge_id_pairs(&graph), vec![(1, 2)]);
    }

    #[test]
    fn test_bidirectional_profile_overrides_oneway() {
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(
            vec![1, 2],
            vec![("highway", "residential"), ("oneway", "yes")],
        );

        let graph = create_graph(nodes, vec![way], true, true);

        assert_eq!(edge_id_pairs(&graph), vec![(1, 2), (2, 1)]);
    }

    #[test]
    fn test_bidirectional_produces_two_edges() {
        let nodes = [make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(vec![1, 2], vec![("highway", "residential")]);
        let graph = create_graph(
            vec![nodes[0].clone(), nodes[1].clone()],
            vec![way],
            true,
            false,
        );
        assert_eq!(graph.edge_count(), 2);
    }

    #[test]
    fn dangling_node_refs_are_skipped_instead_of_panicking() {
        // Clipped extracts reference nodes outside the extract (id 99 here).
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0)];
        let way = make_way_raw(vec![99, 1, 2], vec![("highway", "residential")]);

        let graph = create_graph(nodes, vec![way], true, false);

        assert_eq!(edge_id_pairs(&graph), vec![(1, 2), (2, 1)]);
    }

    #[test]
    fn oriented_geometry_follows_edge_direction() {
        let (a, b) = (make_node(1, 0.0, 0.0), make_node(2, 0.001, 0.0));
        let mut way = make_way_raw(vec![], vec![]);

        let straight: Vec<_> = way.oriented_geometry(&a, &b).points().collect();
        assert_eq!(straight, vec![(0.0, 0.0), (0.001, 0.0)]);

        // Stored backwards relative to a → b: must be flipped.
        way.geometry = vec![(0.001, 0.0), (0.0005, 0.0001), (0.0, 0.0)];
        let flipped: Vec<_> = way.oriented_geometry(&a, &b).points().collect();
        assert_eq!(flipped, vec![(0.0, 0.0), (0.0005, 0.0001), (0.001, 0.0)]);
        let reverse: Vec<_> = way.oriented_geometry(&b, &a).points().collect();
        assert_eq!(reverse, way.geometry);
    }

    #[test]
    fn induced_subgraph_keeps_order_and_internal_edges() {
        let nodes = vec![
            make_node(1, 0.0, 0.0),
            make_node(2, 0.001, 0.0),
            make_node(3, 0.002, 0.0),
        ];
        let way = make_way_raw(vec![1, 2, 3], vec![("highway", "residential")]);
        let sg = SpatialGraph::new(create_graph(nodes, vec![way], true, false));

        let sub = sg.induced_subgraph(|idx| sg.graph[idx].id != 3);

        let ids: Vec<i64> = sub.graph.node_weights().map(|n| n.id).collect();
        assert_eq!(ids, vec![1, 2]);
        assert_eq!(edge_id_pairs(&sub.graph), vec![(1, 2), (2, 1)]);
    }

    #[test]
    fn test_nearest_node_finds_closest() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        graph.add_node(make_node(2, 52.0, 13.0));
        let sg = SpatialGraph::new(graph);
        let idx = sg.nearest_node(48.001, 11.001).unwrap();
        assert_eq!(sg.graph[idx].id, 1);
    }

    #[test]
    fn test_snap_point_returns_diagnostics() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        let sg = SpatialGraph::new(graph);

        let snap = sg.snap_point(48.001, 11.001).unwrap();

        assert_eq!(snap.node_id, 1);
        assert_eq!(snap.node_lat, 48.0);
        assert_eq!(snap.node_lon, 11.0);
        assert!(snap.distance_m > 0.0);
    }

    #[test]
    fn test_snap_point_within_rejects_far_snap() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        let sg = SpatialGraph::new(graph);

        assert!(sg.snap_point_within(48.001, 11.001, Some(500.0)).is_some());
        assert!(sg.snap_point_within(48.001, 11.001, Some(1.0)).is_none());
    }

    #[test]
    fn test_parse_xml_minimal_osm_fixture() {
        let xml = r#"
            <osm version="0.6">
              <node id="1" lat="48.0" lon="11.0" />
              <node id="2" lat="48.001" lon="11.0">
                <tag k="amenity" v="cafe" />
              </node>
              <way id="10">
                <nd ref="1" />
                <nd ref="2" />
                <tag k="highway" v="residential" />
                <tag k="name" v="Fixture Street" />
              </way>
            </osm>
        "#;

        let parsed = parse_xml(xml).unwrap();

        assert_eq!(parsed.nodes.len(), 2);
        assert_eq!(parsed.ways.len(), 1);
        assert_eq!(parsed.ways[0].nodes.len(), 2);
        assert!(parsed.ways[0]
            .tags
            .iter()
            .any(|tag| tag.key == "highway" && tag.value == "residential"));
        assert_eq!(parsed.nodes[1].tags[0].key, "amenity");
    }

    #[test]
    fn test_parse_xml_malformed_input_errors() {
        let err = parse_xml("<osm><node id=\"1\"></osm>").unwrap_err();
        assert!(!err.to_string().is_empty());
    }
}
