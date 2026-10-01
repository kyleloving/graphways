//! Core graph types: the OSM parse shapes, the [`Edge`] weight stored on
//! every road segment, graph construction, and [`SpatialGraph`], which bundles
//! a road graph with the indexes every query needs.

use crate::ch::ContractionHierarchy;
use crate::overpass::NetworkType;
use crate::search::{SearchIndex, SlotCosts};
use crate::simplify::simplify_graph;
use crate::utils::{calculate_distance, calculate_travel_time};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use serde::Deserialize;
use std::collections::HashMap;
use std::ops::Index;
use std::sync::{Arc, OnceLock};

// ---------------------------------------------------------------------------
// OSM input shapes (Overpass XML and PBF both produce these)
// ---------------------------------------------------------------------------

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

/// An OSM way as parsed from XML or PBF: the input to [`create_graph`].
#[derive(Debug, Deserialize, Clone)]
pub struct XmlWay {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "nd", default)]
    pub nodes: Vec<XmlNodeRef>,
    #[serde(rename = "tag", default)]
    pub tags: Vec<XmlTag>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct XmlNodeRef {
    #[serde(rename = "@ref")]
    pub node_id: i64,
}

#[derive(Debug, Deserialize, Clone, PartialEq, Eq)]
pub struct XmlTag {
    #[serde(rename = "@k")]
    pub key: String,
    #[serde(rename = "@v")]
    pub value: String,
}

pub fn parse_xml(xml_data: &str) -> Result<XmlData, quick_xml::DeError> {
    quick_xml::de::from_str(xml_data)
}

// ---------------------------------------------------------------------------
// Road graph
// ---------------------------------------------------------------------------

/// The road network: OSM nodes connected by directed [`Edge`]s.
pub type RoadGraph = DiGraph<XmlNode, Edge>;

/// One directed road segment, or a collapsed chain of segments, in a
/// [`RoadGraph`].
#[derive(Debug, Clone, Default)]
pub struct Edge {
    /// OSM id of the way this edge was cut from (the first way of a chain).
    pub way_id: i64,
    /// Routing-relevant tags of that way, shared by every edge cut from it.
    pub tags: Arc<[XmlTag]>,
    /// Length in metres.
    pub length: f64,
    pub speed_kph: f64,
    /// Travel times in seconds per mode.
    pub walk_travel_time: f64,
    pub bike_travel_time: f64,
    pub drive_travel_time: f64,
    /// Intermediate shape as `(lat, lon)` points. Empty means a straight
    /// segment between the endpoints, which is what unsimplified edges use.
    /// Read it through [`Edge::oriented_geometry`], which handles both cases
    /// and the orientation.
    pub geometry: Vec<(f64, f64)>,
}

impl Edge {
    /// A straight edge whose travel times follow from `length` at walking
    /// pace (5 km/h), cycling pace (15 km/h) and `speed_kph` for driving.
    pub fn from_length(way_id: i64, tags: Arc<[XmlTag]>, length: f64, speed_kph: f64) -> Self {
        Edge {
            way_id,
            tags,
            length,
            speed_kph,
            walk_travel_time: calculate_travel_time(length, 5.0),
            bike_travel_time: calculate_travel_time(length, 15.0),
            drive_travel_time: calculate_travel_time(length, speed_kph),
            geometry: Vec::new(),
        }
    }

    /// Travel time in seconds for `network_type`.
    #[inline]
    pub fn travel_time(&self, network_type: NetworkType) -> f64 {
        self.cost(CostField::of(network_type))
    }

    #[inline]
    pub(crate) fn cost(&self, field: CostField) -> f64 {
        match field {
            CostField::Walk => self.walk_travel_time,
            CostField::Bike => self.bike_travel_time,
            CostField::Drive => self.drive_travel_time,
        }
    }

    /// Value of the tag named `key`, if present.
    pub fn tag(&self, key: &str) -> Option<&str> {
        find_tag(&self.tags, key).map(|tag| tag.value.as_str())
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
    Walk = 0,
    Bike = 1,
    Drive = 2,
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
/// [`Edge::oriented_geometry`].
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
pub fn edge_geometry(graph: &RoadGraph, edge: EdgeIndex) -> EdgeGeometry<'_> {
    let (source, target) = graph
        .edge_endpoints(edge)
        .expect("edge index belongs to this graph");
    graph[edge].oriented_geometry(&graph[source], &graph[target])
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

fn find_tag<'a>(tags: &'a [XmlTag], key: &str) -> Option<&'a XmlTag> {
    tags.iter().find(|tag| tag.key == key)
}

fn assess_path_directionality(tags: &[XmlTag]) -> Direction {
    let oneway = find_tag(tags, "oneway").map(|tag| tag.value.as_str());
    match oneway {
        Some("-1" | "reverse") => Direction::OneWayReverse,
        Some("yes" | "true" | "1") => Direction::OneWayForward,
        // Roundabouts are one-way implicitly.
        _ if find_tag(tags, "junction").is_some_and(|tag| tag.value == "roundabout") => {
            Direction::OneWayForward
        }
        _ => Direction::Bidirectional,
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

fn way_speed_kph(tags: &[XmlTag]) -> f64 {
    const FALLBACK_SPEED_KPH: f64 = 50.0;
    find_tag(tags, "maxspeed")
        .and_then(|tag| clean_maxspeed(&tag.value))
        .or_else(|| find_tag(tags, "highway").and_then(|tag| highway_speed_kph(&tag.value)))
        .unwrap_or(FALLBACK_SPEED_KPH)
}

/// The tags edges keep; everything else is dropped at build time.
fn useful_tags(mut tags: Vec<XmlTag>) -> Arc<[XmlTag]> {
    const USEFUL_TAGS: &[&str] = &["highway", "name", "ref", "bridge", "tunnel", "service"];
    tags.retain(|tag| USEFUL_TAGS.contains(&tag.key.as_str()));
    tags.into()
}

/// Build a directed road graph from parsed OSM nodes and ways.
///
/// Every consecutive node pair of a way becomes one edge per traversable
/// direction; all edges of a way share one copy of its tags. Way references
/// to nodes missing from `nodes` (common in clipped extracts) are skipped
/// rather than panicking. Unless `retain_all` is set the graph is then
/// simplified: nearby intersection nodes are merged and degree-two chains are
/// collapsed into single edges.
pub fn create_graph(
    nodes: Vec<XmlNode>,
    ways: Vec<XmlWay>,
    retain_all: bool,
    bidirectional: bool,
) -> RoadGraph {
    let segment_count: usize = ways.iter().map(|w| w.nodes.len().saturating_sub(1)).sum();
    let mut graph = DiGraph::with_capacity(nodes.len(), segment_count * 2);
    let mut node_index_map = HashMap::with_capacity(nodes.len());

    for node in nodes {
        let id = node.id;
        node_index_map.insert(id, graph.add_node(node));
    }

    for way in ways {
        let traversals = assess_path_directionality(&way.tags).traversals(bidirectional);
        let speed_kph = way_speed_kph(&way.tags);
        let tags = useful_tags(way.tags);

        for pair in way.nodes.windows(2) {
            let (Some(&a), Some(&b)) = (
                node_index_map.get(&pair[0].node_id),
                node_index_map.get(&pair[1].node_id),
            ) else {
                continue;
            };
            let (pa, pb) = (node_to_latlon(&graph, a), node_to_latlon(&graph, b));
            let length = calculate_distance(pa.0, pa.1, pb.0, pb.1);
            for &reversed in traversals {
                let (from, to) = if reversed { (b, a) } else { (a, b) };
                let edge = Edge::from_length(way.id, Arc::clone(&tags), length, speed_kph);
                graph.add_edge(from, to, edge);
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

pub fn node_to_latlon(graph: &RoadGraph, node_index: NodeIndex) -> (f64, f64) {
    let node = &graph[node_index];
    (node.lat, node.lon)
}

// ---------------------------------------------------------------------------
// Spatial index entries
// ---------------------------------------------------------------------------

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

impl PointDistance for NodeEntry {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dlat = self.point[0] - point[0];
        let dlon = self.point[1] - point[1];
        dlat * dlat + dlon * dlon
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

// ---------------------------------------------------------------------------
// NodeMap
// ---------------------------------------------------------------------------

/// A map from graph nodes to values with O(1) lookup and insertion-ordered
/// iteration, backed by a dense slot table instead of hashing.
///
/// Search results use it: reachability entries are in settle order, i.e.
/// sorted by travel time.
#[derive(Debug, Clone)]
pub struct NodeMap<T> {
    slots: Vec<u32>,
    entries: Vec<(NodeIndex, T)>,
}

const NO_SLOT: u32 = u32::MAX;

impl<T> NodeMap<T> {
    /// An empty map able to hold any node of a graph with `node_count` nodes.
    pub fn with_node_count(node_count: usize) -> Self {
        Self {
            slots: vec![NO_SLOT; node_count],
            entries: Vec::new(),
        }
    }

    /// Insert or replace the value for `node`, returning the previous value.
    /// Panics if `node` is outside the graph the map was sized for.
    pub fn insert(&mut self, node: NodeIndex, value: T) -> Option<T> {
        let slot = &mut self.slots[node.index()];
        if *slot == NO_SLOT {
            *slot = self.entries.len() as u32;
            self.entries.push((node, value));
            None
        } else {
            Some(std::mem::replace(
                &mut self.entries[*slot as usize].1,
                value,
            ))
        }
    }

    #[inline]
    pub fn get(&self, node: NodeIndex) -> Option<&T> {
        match self.slots.get(node.index()) {
            Some(&slot) if slot != NO_SLOT => Some(&self.entries[slot as usize].1),
            _ => None,
        }
    }

    #[inline]
    pub fn contains_key(&self, node: NodeIndex) -> bool {
        self.get(node).is_some()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// `(node, value)` pairs in insertion order.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (&NodeIndex, &T)> + '_ {
        self.entries.iter().map(|(node, value)| (node, value))
    }

    pub fn keys(&self) -> impl ExactSizeIterator<Item = NodeIndex> + '_ {
        self.entries.iter().map(|(node, _)| *node)
    }

    pub fn values(&self) -> impl ExactSizeIterator<Item = &T> + '_ {
        self.entries.iter().map(|(_, value)| value)
    }

    /// The entries as a slice, in insertion order.
    pub fn as_slice(&self) -> &[(NodeIndex, T)] {
        &self.entries
    }
}

impl<T> Index<NodeIndex> for NodeMap<T> {
    type Output = T;

    fn index(&self, node: NodeIndex) -> &T {
        self.get(node).expect("node is not in this NodeMap")
    }
}

impl<'a, T> IntoIterator for &'a NodeMap<T> {
    type Item = (&'a NodeIndex, &'a T);
    type IntoIter = std::iter::Map<
        std::slice::Iter<'a, (NodeIndex, T)>,
        fn(&'a (NodeIndex, T)) -> (&'a NodeIndex, &'a T),
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.entries.iter().map(|(node, value)| (node, value))
    }
}

// ---------------------------------------------------------------------------
// SpatialGraph
// ---------------------------------------------------------------------------

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

/// A road graph bundled with the indexes queries need: an R-tree for
/// nearest-node lookups, a compact adjacency for searches, and (after
/// [`SpatialGraph::prepare_routing`]) a contraction hierarchy per mode.
///
/// Build once via [`SpatialGraph::new`], reuse for all queries. Cloning is
/// O(1): clones share the graph and every index, including ones built later.
#[derive(Clone)]
pub struct SpatialGraph {
    pub graph: Arc<RoadGraph>,
    /// POI OSM node id → snapped graph node diagnostics, computed once at startup via
    /// `snap_pois`. `None` until called; `Some` map used by POI filtering
    /// for O(1) lookup instead of an R-tree query on every request.
    pub poi_snaps: Option<Arc<HashMap<i64, SnappedPoi>>>,
    index: Arc<GraphIndex>,
}

/// Indexes derived from the graph. The cheap ones are built eagerly, the rest
/// on first use and then shared by every clone.
struct GraphIndex {
    tree: RTree<NodeEntry>,
    search: SearchIndex,
    node_ids: OnceLock<Vec<(i64, NodeIndex)>>,
    max_speed_mps: OnceLock<[f64; 3]>,
    hierarchies: [OnceLock<ContractionHierarchy>; 3],
}

impl SpatialGraph {
    pub fn new(graph: RoadGraph) -> Self {
        let tree = RTree::bulk_load(
            graph
                .node_indices()
                .map(|i| NodeEntry::new(&graph[i], i))
                .collect(),
        );
        let search = SearchIndex::new(&graph);
        Self {
            graph: Arc::new(graph),
            poi_snaps: None,
            index: Arc::new(GraphIndex {
                tree,
                search,
                node_ids: OnceLock::new(),
                max_speed_mps: OnceLock::new(),
                hierarchies: Default::default(),
            }),
        }
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

    /// Compact adjacency used by every search.
    pub(crate) fn search_index(&self) -> &SearchIndex {
        &self.index.search
    }

    /// Per-slot edge costs for `field`, built on first use.
    pub(crate) fn slot_costs(&self, field: CostField) -> &SlotCosts {
        self.index
            .search
            .costs(field as usize, |edge| self.graph[edge].cost(field))
    }

    pub(crate) fn hierarchy_slot(&self, field: CostField) -> &OnceLock<ContractionHierarchy> {
        &self.index.hierarchies[field as usize]
    }

    /// The fastest straight-line speed (m/s) any edge allows for `field`:
    /// `straight_line_distance / speed` never exceeds the true travel time
    /// between two nodes, which makes it an admissible and consistent A*
    /// heuristic. Computed once, on first use.
    pub(crate) fn max_straight_line_speed(&self, field: CostField) -> f64 {
        self.index
            .max_speed_mps
            .get_or_init(|| max_straight_line_speeds(&self.graph))[field as usize]
    }

    /// The graph node with OSM id `node_id`, if any. O(log n) after a
    /// one-time index build.
    pub fn node_index(&self, node_id: i64) -> Option<NodeIndex> {
        let ids = self.index.node_ids.get_or_init(|| {
            let mut ids: Vec<(i64, NodeIndex)> = self
                .graph
                .node_indices()
                .map(|idx| (self.graph[idx].id, idx))
                .collect();
            ids.sort_unstable();
            ids
        });
        let pos = ids.partition_point(|&(id, _)| id < node_id);
        ids.get(pos)
            .filter(|&&(id, _)| id == node_id)
            .map(|&(_, idx)| idx)
    }

    /// Copy the subgraph induced by the nodes for which `keep` returns true.
    ///
    /// Node and edge order follow the parent graph, so results are
    /// deterministic. POI snaps and prepared hierarchies are not carried over.
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

    /// Pre-snap a set of POI nodes to their nearest graph nodes, storing the
    /// result for O(1) lookup at request time.
    ///
    /// `pois` is the POI list returned by [`crate::pbf::read_pbf`]. Call once
    /// at startup after `new`.
    pub fn snap_pois(&mut self, pois: &[crate::poi::Poi]) {
        let snaps: HashMap<i64, SnappedPoi> = pois
            .iter()
            .filter_map(|poi| {
                let snap = self.snap_point(poi.lat, poi.lon)?;
                Some((
                    poi.id,
                    SnappedPoi {
                        poi_id: poi.id,
                        snap,
                    },
                ))
            })
            .collect();
        self.poi_snaps = Some(Arc::new(snaps));
    }

    pub fn nearest_node(&self, lat: f64, lon: f64) -> Option<NodeIndex> {
        self.index
            .tree
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

/// Per cost field, the largest `straight-line distance / travel time` over all
/// edges. Because great-circle distance obeys the triangle inequality,
/// `distance(n, goal) / max_speed` never exceeds the true remaining cost.
fn max_straight_line_speeds(graph: &RoadGraph) -> [f64; 3] {
    let mut max_speed = [0.0_f64; 3];
    for edge in graph.edge_references() {
        let (a, b) = (&graph[edge.source()], &graph[edge.target()]);
        let distance = calculate_distance(a.lat, a.lon, b.lat, b.lon);
        if distance == 0.0 {
            continue;
        }
        for field in CostField::ALL {
            let time = edge.weight().cost(field);
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
        }
    }

    fn edge_id_pairs(graph: &RoadGraph) -> Vec<(i64, i64)> {
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
        let mut way = Edge::default();

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
    fn edges_of_one_way_share_a_single_tag_allocation() {
        let nodes = vec![
            make_node(1, 0.0, 0.0),
            make_node(2, 0.001, 0.0),
            make_node(3, 0.002, 0.0),
        ];
        let way = make_way_raw(
            vec![1, 2, 3],
            vec![
                ("highway", "residential"),
                ("name", "Elm"),
                ("surface", "x"),
            ],
        );

        let graph = create_graph(nodes, vec![way], true, false);

        let edges: Vec<&Edge> = graph.edge_weights().collect();
        assert_eq!(edges.len(), 4);
        assert!(edges.iter().all(|e| Arc::ptr_eq(&e.tags, &edges[0].tags)));
        assert_eq!(edges[0].tag("name"), Some("Elm"));
        assert_eq!(
            edges[0].tag("surface"),
            None,
            "non-routing tags are dropped"
        );
        assert!(
            edges[0].geometry.is_empty(),
            "straight edges store no points"
        );
    }

    #[test]
    fn node_map_behaves_like_an_ordered_map() {
        let mut map = NodeMap::with_node_count(5);
        assert!(map.is_empty());
        assert_eq!(map.insert(NodeIndex::new(3), "c"), None);
        assert_eq!(map.insert(NodeIndex::new(1), "a"), None);
        assert_eq!(map.insert(NodeIndex::new(3), "C"), Some("c"));

        assert_eq!(map.len(), 2);
        assert_eq!(map.get(NodeIndex::new(3)), Some(&"C"));
        assert_eq!(map[NodeIndex::new(1)], "a");
        assert!(!map.contains_key(NodeIndex::new(0)));
        assert!(
            !map.contains_key(NodeIndex::new(99)),
            "out of range is absent"
        );
        let keys: Vec<usize> = map.keys().map(|k| k.index()).collect();
        assert_eq!(keys, vec![3, 1], "insertion order");
    }

    #[test]
    fn node_index_finds_nodes_by_osm_id() {
        let nodes = vec![make_node(42, 0.0, 0.0), make_node(7, 0.001, 0.0)];
        let way = make_way_raw(vec![42, 7], vec![("highway", "residential")]);
        let sg = SpatialGraph::new(create_graph(nodes, vec![way], true, false));

        let idx = sg.node_index(7).unwrap();
        assert_eq!(sg.graph[idx].id, 7);
        assert_eq!(sg.node_index(8), None);
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
