//! Core graph types: the OSM parse shapes, the [`Edge`] weight stored on
//! every road segment, graph construction, and [`SpatialGraph`], which bundles
//! a road graph with the indexes every query needs.

use crate::ch::ContractionHierarchy;
use crate::error::OsmGraphError;
use crate::overpass::NetworkType;
use crate::profile::{BuildOptions, Profile};
use crate::restrictions;
use crate::search::{SearchIndex, SlotCosts};
use crate::simplify::simplify_graph;
use crate::utils::{calculate_distance, calculate_travel_time};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rstar::primitives::{GeomWithData, Line};
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::ops::Index;
use std::sync::{Arc, OnceLock};

// ---------------------------------------------------------------------------
// OSM input shapes (Overpass XML and PBF both produce these)
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize, Default)]
pub struct OsmData {
    #[serde(rename = "node", default)]
    pub nodes: Vec<OsmNode>,
    #[serde(rename = "way", default)]
    pub ways: Vec<OsmWay>,
    /// Relations; only turn restrictions are used.
    #[serde(rename = "relation", default)]
    pub relations: Vec<OsmRelation>,
}

/// An OSM relation (only `type=restriction` ones are read).
#[derive(Debug, Deserialize, Clone, Default)]
pub struct OsmRelation {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "member", default)]
    pub members: Vec<OsmMember>,
    #[serde(rename = "tag", default)]
    pub tags: Vec<OsmTag>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct OsmMember {
    /// `node`, `way` or `relation`.
    #[serde(rename = "@type")]
    pub kind: String,
    #[serde(rename = "@ref")]
    pub reference: i64,
    #[serde(rename = "@role", default)]
    pub role: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OsmNode {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "@lat")]
    pub lat: f64,
    #[serde(rename = "@lon")]
    pub lon: f64,
    #[serde(rename = "tag", default)]
    pub tags: Vec<OsmTag>,
}

/// An OSM way as parsed from XML or PBF: the input to [`create_graph`].
#[derive(Debug, Deserialize, Clone)]
pub struct OsmWay {
    #[serde(rename = "@id")]
    pub id: i64,
    #[serde(rename = "nd", default)]
    pub nodes: Vec<OsmNodeRef>,
    #[serde(rename = "tag", default)]
    pub tags: Vec<OsmTag>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct OsmNodeRef {
    #[serde(rename = "@ref")]
    pub node_id: i64,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Eq)]
pub struct OsmTag {
    #[serde(rename = "@k")]
    pub key: String,
    #[serde(rename = "@v")]
    pub value: String,
}

/// Former names of the OSM input types.
#[deprecated(since = "0.5.0", note = "renamed to OsmData")]
pub type XmlData = OsmData;
#[deprecated(since = "0.5.0", note = "renamed to OsmNode")]
pub type XmlNode = OsmNode;
#[deprecated(since = "0.5.0", note = "renamed to OsmWay")]
pub type XmlWay = OsmWay;
#[deprecated(since = "0.5.0", note = "renamed to OsmTag")]
pub type XmlTag = OsmTag;
#[deprecated(since = "0.5.0", note = "renamed to OsmNodeRef")]
pub type XmlNodeRef = OsmNodeRef;

pub fn parse_xml(xml_data: &str) -> Result<OsmData, quick_xml::DeError> {
    quick_xml::de::from_str(xml_data)
}

// ---------------------------------------------------------------------------
// Road graph
// ---------------------------------------------------------------------------

/// The road network: OSM nodes connected by directed [`Edge`]s.
pub type RoadGraph = DiGraph<OsmNode, Edge>;

/// One directed road segment, or a collapsed chain of segments, in a
/// [`RoadGraph`].
#[derive(Debug, Clone, Default)]
pub struct Edge {
    /// OSM id of the way this edge was cut from (the first way of a chain).
    pub way_id: i64,
    /// OSM id of the way of the edge's last segment: the same as `way_id`
    /// unless simplification joined several ways into one edge.
    pub last_way_id: i64,
    /// Routing-relevant tags of that way, shared by every edge cut from it.
    pub tags: Arc<[OsmTag]>,
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
    /// A straight edge whose travel times follow from `length` at the
    /// default walking (5 km/h) and cycling (15 km/h) paces and `speed_kph`
    /// for driving.
    pub fn from_length(way_id: i64, tags: Arc<[OsmTag]>, length: f64, speed_kph: f64) -> Self {
        Self::with_profile(way_id, tags, length, speed_kph, &Profile::default())
    }

    /// Like [`Edge::from_length`] with walking and cycling paces from `profile`.
    pub fn with_profile(
        way_id: i64,
        tags: Arc<[OsmTag]>,
        length: f64,
        speed_kph: f64,
        profile: &Profile,
    ) -> Self {
        let mut edge = Edge {
            way_id,
            last_way_id: way_id,
            tags,
            speed_kph,
            ..Edge::default()
        };
        edge.set_length(length, profile);
        edge
    }

    /// Set the length and recompute all three travel times from the edge's
    /// driving speed and `profile`'s walking and cycling paces.
    pub(crate) fn set_length(&mut self, length: f64, profile: &Profile) {
        self.length = length;
        self.walk_travel_time = calculate_travel_time(length, profile.walk_speed_kph);
        self.bike_travel_time = calculate_travel_time(length, profile.bike_speed_kph);
        self.drive_travel_time = calculate_travel_time(length, self.speed_kph);
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
    pub fn oriented_geometry(&self, source: &OsmNode, target: &OsmNode) -> EdgeGeometry<'_> {
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

/// A stretch of one edge between two fractions of its length, `0 <= from <=
/// to <= 1`, in the edge's direction. Routes are sequences of pieces: whole
/// edges in the middle, partial ones where a route starts or ends mid-road.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Piece {
    pub edge: EdgeIndex,
    pub from: f64,
    pub to: f64,
}

impl Piece {
    pub fn whole(edge: EdgeIndex) -> Self {
        Piece {
            edge,
            from: 0.0,
            to: 1.0,
        }
    }

    /// Share of the edge this piece covers.
    pub fn share(&self) -> f64 {
        (self.to - self.from).max(0.0)
    }

    /// `(lat, lon)` points of this stretch: the interpolated start, every
    /// shape point strictly inside, and the interpolated end.
    pub fn points(&self, graph: &RoadGraph) -> Vec<(f64, f64)> {
        let geometry = edge_geometry(graph, self.edge);
        if self.from <= 0.0 && self.to >= 1.0 {
            return geometry.points().collect();
        }
        let points: Vec<(f64, f64)> = geometry.points().collect();
        let mut cumulative = Vec::with_capacity(points.len());
        let mut total = 0.0;
        cumulative.push(0.0);
        for pair in points.windows(2) {
            total += calculate_distance(pair[0].0, pair[0].1, pair[1].0, pair[1].1);
            cumulative.push(total);
        }
        let at = |distance: f64| -> (f64, f64) {
            let i = cumulative
                .partition_point(|&c| c <= distance)
                .clamp(1, points.len() - 1);
            let span = cumulative[i] - cumulative[i - 1];
            let s = if span > 0.0 {
                ((distance - cumulative[i - 1]) / span).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (a, b) = (points[i - 1], points[i]);
            (a.0 + (b.0 - a.0) * s, a.1 + (b.1 - a.1) * s)
        };
        let (start, end) = (self.from * total, self.to * total);
        let mut out = vec![at(start)];
        out.extend(
            points
                .iter()
                .zip(&cumulative)
                .filter(|&(_, &c)| c > start && c < end)
                .map(|(&p, _)| p),
        );
        out.push(at(end));
        out
    }
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

fn find_tag<'a>(tags: &'a [OsmTag], key: &str) -> Option<&'a OsmTag> {
    tags.iter().find(|tag| tag.key == key)
}

fn assess_path_directionality(tags: &[OsmTag]) -> Direction {
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

fn way_speed_kph(tags: &[OsmTag], profile: &Profile) -> f64 {
    profile
        .use_maxspeed
        .then(|| find_tag(tags, "maxspeed").and_then(|tag| clean_maxspeed(&tag.value)))
        .flatten()
        .unwrap_or_else(|| {
            profile.drive_speed_kph(find_tag(tags, "highway").map(|t| t.value.as_str()))
        })
}

/// The tags edges keep; everything else is dropped at build time.
fn useful_tags(mut tags: Vec<OsmTag>) -> Arc<[OsmTag]> {
    const USEFUL_TAGS: &[&str] = &["highway", "name", "ref", "bridge", "tunnel", "service"];
    tags.retain(|tag| USEFUL_TAGS.contains(&tag.key.as_str()));
    tags.into()
}

/// Build a directed road graph from parsed OSM nodes and ways with the
/// default [`Profile`]. See [`create_graph_with`].
pub fn create_graph(
    nodes: Vec<OsmNode>,
    ways: Vec<OsmWay>,
    retain_all: bool,
    bidirectional: bool,
) -> RoadGraph {
    create_graph_with(
        nodes,
        ways,
        bidirectional,
        &BuildOptions::retain_all(retain_all),
    )
}

/// Build a directed road graph from parsed OSM nodes and ways.
///
/// Every consecutive node pair of a way becomes one edge per traversable
/// direction (both directions when `bidirectional`, as for walking); all
/// edges of a way share one copy of its tags. Way references to nodes missing
/// from `nodes` (common in clipped extracts) are skipped rather than
/// panicking. Unless `options.retain_all` is set the graph is then
/// simplified: nearby intersection nodes are merged and degree-two chains are
/// collapsed into single edges.
pub fn create_graph_with(
    nodes: Vec<OsmNode>,
    ways: Vec<OsmWay>,
    bidirectional: bool,
    options: &BuildOptions,
) -> RoadGraph {
    build_graph(nodes, ways, bidirectional, options, &HashSet::new())
}

/// [`create_graph_with`], keeping the nodes with OSM ids in `protected`
/// intact through simplification (turn-restriction via nodes).
fn build_graph(
    nodes: Vec<OsmNode>,
    ways: Vec<OsmWay>,
    bidirectional: bool,
    options: &BuildOptions,
    protected: &HashSet<i64>,
) -> RoadGraph {
    let profile = &options.profile;
    let segment_count: usize = ways.iter().map(|w| w.nodes.len().saturating_sub(1)).sum();
    let mut graph = DiGraph::with_capacity(nodes.len(), segment_count * 2);
    let mut node_index_map = HashMap::with_capacity(nodes.len());

    // Traffic signals delay drivers entering their node: in both directions,
    // or only along/against the way per `traffic_signals:direction`.
    let mut signals: HashMap<i64, SignalDirection> = HashMap::new();
    for node in nodes {
        let id = node.id;
        if profile.traffic_signal_s > 0.0 {
            if let Some(direction) = signal_direction(&node.tags) {
                signals.insert(id, direction);
            }
        }
        node_index_map.insert(id, graph.add_node(node));
    }

    for way in ways {
        let traversals = assess_path_directionality(&way.tags).traversals(bidirectional);
        let speed_kph = way_speed_kph(&way.tags, profile);
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
                let mut edge =
                    Edge::with_profile(way.id, Arc::clone(&tags), length, speed_kph, profile);
                let entered = if reversed { &pair[0] } else { &pair[1] };
                let delayed = match signals.get(&entered.node_id) {
                    Some(SignalDirection::Both) => true,
                    Some(SignalDirection::Forward) => !reversed,
                    Some(SignalDirection::Backward) => reversed,
                    None => false,
                };
                if delayed && edge.drive_travel_time.is_finite() {
                    edge.drive_travel_time += profile.traffic_signal_s;
                }
                graph.add_edge(from, to, edge);
            }
        }
    }

    if options.retain_all {
        graph
    } else {
        simplify_graph(graph, profile, protected)
    }
}

#[derive(Clone, Copy)]
enum SignalDirection {
    Both,
    Forward,
    Backward,
}

/// Whether a node is a traffic signal for drivers, and for which direction
/// along its way.
fn signal_direction(tags: &[OsmTag]) -> Option<SignalDirection> {
    if find_tag(tags, "highway").map(|t| t.value.as_str()) != Some("traffic_signals") {
        return None;
    }
    Some(
        match find_tag(tags, "traffic_signals:direction").map(|t| t.value.as_str()) {
            Some("forward") => SignalDirection::Forward,
            Some("backward") => SignalDirection::Backward,
            _ => SignalDirection::Both,
        },
    )
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
    pub(crate) fn new(node: &OsmNode, index: NodeIndex) -> Self {
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

    /// The entries by value, in insertion order.
    pub fn into_entries(self) -> Vec<(NodeIndex, T)> {
        self.entries
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

/// A WGS84 coordinate. Every method that takes a location accepts
/// `impl Into<LatLon>`, so `(lat, lon)` tuples work too, in that order.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// The point itself, if both coordinates are finite.
    fn finite(self) -> Option<Self> {
        (self.lat.is_finite() && self.lon.is_finite()).then_some(self)
    }

    /// Great-circle distance in metres.
    pub fn distance_m(self, other: LatLon) -> f64 {
        calculate_distance(self.lat, self.lon, other.lat, other.lon)
    }
}

impl From<(f64, f64)> for LatLon {
    fn from((lat, lon): (f64, f64)) -> Self {
        Self { lat, lon }
    }
}

impl From<&OsmNode> for LatLon {
    fn from(node: &OsmNode) -> Self {
        Self::new(node.lat, node.lon)
    }
}

/// Where a coordinate lands on the road network.
///
/// Points snap to the closest point on any road, not to the closest
/// intersection, so a query from the middle of a long block starts in the
/// middle of that block. The nearest end of the snapped edge is reported too.
#[derive(Debug, Clone, Copy)]
pub struct SnapResult {
    pub input_lat: f64,
    pub input_lon: f64,
    /// The point on the road the input was snapped to.
    pub snapped_lat: f64,
    pub snapped_lon: f64,
    /// Straight-line distance in metres from the input to the road.
    pub distance_m: f64,
    /// The edge snapped onto, or `None` when the graph has no edges near
    /// enough to matter and the point snapped to a node instead.
    pub edge: Option<EdgeIndex>,
    /// Position along `edge` as a share of its length, from its source (0)
    /// to its target (1).
    pub fraction: f64,
    /// The endpoint of `edge` nearest the snapped point.
    pub node_index: NodeIndex,
    pub node_id: i64,
    pub node_lat: f64,
    pub node_lon: f64,
}

impl SnapResult {
    /// The snapped point on the road.
    pub fn snapped(&self) -> LatLon {
        LatLon::new(self.snapped_lat, self.snapped_lon)
    }
}

/// A way onto (or off) the network from a snapped point: reach `node` at
/// `cost`, travelling `piece` of the snapped edge (`None` when the point is
/// the node itself).
#[derive(Debug, Clone, Copy)]
pub(crate) struct Anchor {
    pub(crate) node: NodeIndex,
    pub(crate) cost: f64,
    pub(crate) piece: Option<Piece>,
}

/// Search roots for a set of anchors: `(state, cost, anchor index)`.
pub(crate) type Roots = Vec<(u32, f64, usize)>;

/// `(state, cost)` pairs for seeding a search.
pub(crate) fn seeds(roots: &Roots) -> Vec<(u32, f64)> {
    roots
        .iter()
        .map(|&(state, cost, _)| (state, cost))
        .collect()
}

/// The cheapest anchor rooted at search state `state`, if any.
pub(crate) fn anchor_at<'a>(
    anchors: &'a [Anchor],
    roots: &Roots,
    state: NodeIndex,
) -> Option<&'a Anchor> {
    roots
        .iter()
        .filter(|&&(s, _, _)| s as usize == state.index())
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|&(_, _, i)| &anchors[i])
}

/// R-tree entry for one straight segment of an edge's geometry, in the
/// local metric projection: `(edge index, segment index)`.
type SegmentEntry = GeomWithData<Line<[f64; 2]>, (u32, u32)>;

#[derive(Debug, Clone, Copy)]
pub struct SnappedPoi {
    pub poi_id: i64,
    pub snap: SnapResult,
}

/// Which end of a query a snapped point is, for error messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Role {
    Origin,
    Destination,
}

/// A road graph for one [`NetworkType`], bundled with the indexes queries
/// need: an R-tree for nearest-node lookups, a compact adjacency for
/// searches, and (after [`SpatialGraph::prepare_routing`]) a contraction
/// hierarchy.
///
/// Every query uses the travel times of the graph's network type. Build once,
/// reuse for all queries. Cloning is O(1): clones share the graph and every
/// index, including ones built later.
#[derive(Clone)]
pub struct SpatialGraph {
    pub graph: Arc<RoadGraph>,
    /// POI OSM node id → snapped graph node diagnostics, computed once at startup via
    /// `snap_pois`. `None` until called; `Some` map used by POI filtering
    /// for O(1) lookup instead of an R-tree query on every request.
    pub poi_snaps: Option<Arc<HashMap<i64, SnappedPoi>>>,
    network_type: NetworkType,
    index: Arc<GraphIndex>,
}

/// Indexes derived from the graph. The cheap ones are built eagerly, the rest
/// on first use and then shared by every clone.
struct GraphIndex {
    forbidden_turns: Vec<(EdgeIndex, EdgeIndex)>,
    turn_costs: Vec<(EdgeIndex, EdgeIndex, f64)>,
    tree: RTree<NodeEntry>,
    segments: RTree<SegmentEntry>,
    search: SearchIndex,
    node_ids: OnceLock<Vec<(i64, NodeIndex)>>,
    max_speed_mps: OnceLock<f64>,
    hierarchy: OnceLock<ContractionHierarchy>,
}

impl SpatialGraph {
    /// Wrap a road graph whose edges are costed for `network_type`.
    pub fn new(graph: RoadGraph, network_type: NetworkType) -> Self {
        Self::with_forbidden_turns(graph, network_type, Vec::new())
    }

    /// Wrap a road graph in which every search must avoid the given
    /// `(into junction, out of junction)` edge pairs, e.g. from
    /// [`crate::restrictions::forbidden_turns`].
    pub fn with_forbidden_turns(
        graph: RoadGraph,
        network_type: NetworkType,
        forbidden_turns: Vec<(EdgeIndex, EdgeIndex)>,
    ) -> Self {
        Self::with_turns(graph, network_type, forbidden_turns, Vec::new())
    }

    /// Wrap a road graph with banned turns and with turn costs: each
    /// `(into junction, out of junction, seconds)` in `turn_costs` adds that
    /// many seconds to the turn (e.g. from [`crate::profile::TurnCosts`]).
    /// Turn costs add to whatever edge cost a query uses, including the
    /// closures of the `_with` queries.
    pub fn with_turns(
        graph: RoadGraph,
        network_type: NetworkType,
        mut forbidden_turns: Vec<(EdgeIndex, EdgeIndex)>,
        mut turn_costs: Vec<(EdgeIndex, EdgeIndex, f64)>,
    ) -> Self {
        forbidden_turns.sort_unstable();
        forbidden_turns.dedup();
        turn_costs.retain(|&(_, _, cost)| cost.is_finite() && cost > 0.0);
        turn_costs.sort_unstable_by_key(|&(a, b, _)| (a, b));
        turn_costs.dedup_by_key(|&mut (a, b, _)| (a, b));
        // Transit stops and vehicles are not places to start or end a trip:
        // points snap to the street network only.
        let tree = RTree::bulk_load(
            graph
                .node_indices()
                .filter(|&i| !crate::transit::is_transit(&graph[i].tags))
                .map(|i| NodeEntry::new(&graph[i], i))
                .collect(),
        );
        let search = SearchIndex::new(&graph, &forbidden_turns, &turn_costs);
        let segments = RTree::bulk_load(segment_entries(&graph));
        Self {
            graph: Arc::new(graph),
            poi_snaps: None,
            network_type,
            index: Arc::new(GraphIndex {
                forbidden_turns,
                turn_costs,
                tree,
                segments,
                search,
                node_ids: OnceLock::new(),
                max_speed_mps: OnceLock::new(),
                hierarchy: OnceLock::new(),
            }),
        }
    }

    /// Build a graph from parsed OSM data with the default profile.
    pub fn from_osm_data(data: OsmData, network_type: NetworkType, retain_all: bool) -> Self {
        Self::from_osm_data_with(data, network_type, &BuildOptions::retain_all(retain_all))
    }

    /// Build a graph from parsed OSM data with [`create_graph_with`].
    pub fn from_osm_data_with(
        data: OsmData,
        network_type: NetworkType,
        options: &BuildOptions,
    ) -> Self {
        let bidirectional = matches!(network_type, NetworkType::Walk);
        let restrictions = if restrictions::applies_to(network_type) {
            restrictions::parse_restrictions(&data.relations)
        } else {
            Vec::new()
        };
        let protected = restrictions::via_nodes(&restrictions);
        let graph = build_graph(data.nodes, data.ways, bidirectional, options, &protected);
        let turns = restrictions::forbidden_turns(&graph, &restrictions);
        let turn_costs = if restrictions::applies_to(network_type) {
            crate::turns::turn_costs(&graph, &options.profile.turn_costs)
        } else {
            Vec::new()
        };
        Self::with_turns(graph, network_type, turns, turn_costs)
    }

    /// Parse an OSM XML document (e.g. an Overpass response) and build a
    /// graph with the default profile.
    pub fn from_osm(
        xml: &str,
        network_type: NetworkType,
        retain_all: bool,
    ) -> Result<Self, OsmGraphError> {
        Self::from_osm_with(xml, network_type, &BuildOptions::retain_all(retain_all))
    }

    /// Parse an OSM XML document and build a graph with custom options.
    pub fn from_osm_with(
        xml: &str,
        network_type: NetworkType,
        options: &BuildOptions,
    ) -> Result<Self, OsmGraphError> {
        Ok(Self::from_osm_data_with(
            parse_xml(xml)?,
            network_type,
            options,
        ))
    }

    /// The network type whose travel times every query uses.
    pub fn network_type(&self) -> NetworkType {
        self.network_type
    }

    /// Turns every search avoids, as `(into junction, out of junction)`
    /// edge pairs.
    pub fn forbidden_turns(&self) -> &[(EdgeIndex, EdgeIndex)] {
        &self.index.forbidden_turns
    }

    /// Seconds added to turns, as `(into junction, out of junction, cost)`.
    pub fn turn_costs(&self) -> &[(EdgeIndex, EdgeIndex, f64)] {
        &self.index.turn_costs
    }

    /// The cost of turning from edge `into` onto edge `out` (0 if free).
    pub fn turn_cost(&self, into: EdgeIndex, out: EdgeIndex) -> f64 {
        let costs = &self.index.turn_costs;
        costs
            .binary_search_by_key(&(into, out), |&(a, b, _)| (a, b))
            .map_or(0.0, |i| costs[i].2)
    }

    pub(crate) fn cost_field(&self) -> CostField {
        CostField::of(self.network_type)
    }

    /// Compact adjacency used by every search.
    pub(crate) fn search_index(&self) -> &SearchIndex {
        &self.index.search
    }

    /// Per-slot edge costs for this graph's network type, built on first use.
    pub(crate) fn slot_costs(&self) -> &SlotCosts {
        let field = self.cost_field();
        self.index
            .search
            .costs(field as usize, |edge| self.graph[edge].cost(field))
    }

    pub(crate) fn hierarchy_slot(&self) -> &OnceLock<ContractionHierarchy> {
        &self.index.hierarchy
    }

    /// The fastest straight-line speed (m/s) any edge allows:
    /// `straight_line_distance / speed` never exceeds the true travel time
    /// between two nodes, which makes it an admissible and consistent A*
    /// heuristic. Computed once, on first use.
    pub(crate) fn max_straight_line_speed(&self) -> f64 {
        *self
            .index
            .max_speed_mps
            .get_or_init(|| max_straight_line_speed(&self.graph, self.cost_field()))
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
        let mut edge_remap = vec![EdgeIndex::end(); self.graph.edge_count()];
        let mut subgraph = DiGraph::new();
        for index in self.graph.node_indices() {
            if keep(index) {
                remap[index.index()] = subgraph.add_node(self.graph[index].clone());
            }
        }
        for edge in self.graph.edge_references() {
            let (source, target) = (remap[edge.source().index()], remap[edge.target().index()]);
            if source != NodeIndex::end() && target != NodeIndex::end() {
                edge_remap[edge.id().index()] =
                    subgraph.add_edge(source, target, edge.weight().clone());
            }
        }
        let kept = |a: EdgeIndex, b: EdgeIndex| {
            let (a, b) = (edge_remap[a.index()], edge_remap[b.index()]);
            (a != EdgeIndex::end() && b != EdgeIndex::end()).then_some((a, b))
        };
        let turns = self
            .forbidden_turns()
            .iter()
            .filter_map(|&(a, b)| kept(a, b))
            .collect();
        let turn_costs = self
            .turn_costs()
            .iter()
            .filter_map(|&(a, b, cost)| kept(a, b).map(|(a, b)| (a, b, cost)))
            .collect();
        SpatialGraph::with_turns(subgraph, self.network_type, turns, turn_costs)
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
                let snap = self.snap_point((poi.lat, poi.lon))?;
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

    /// The node nearest `point`; `None` for an empty graph or a non-finite
    /// coordinate.
    pub fn nearest_node(&self, point: impl Into<LatLon>) -> Option<NodeIndex> {
        let point = point.into().finite()?;
        self.index
            .tree
            .nearest_neighbor(&spatial_index_point(point.lat, point.lon))
            .map(|e| e.index)
    }

    /// Snap a coordinate to the closest point on any road. Falls back to the
    /// nearest node in a graph without edges; `None` for an empty graph or a
    /// non-finite coordinate.
    pub fn snap_point(&self, point: impl Into<LatLon>) -> Option<SnapResult> {
        let point = point.into().finite()?;
        let query = spatial_index_point(point.lat, point.lon);
        // The index projection is only locally uniform, so refine the few
        // closest candidates in a projection centred on the query point.
        let local = |p: (f64, f64)| -> [f64; 2] {
            const METERS_PER_DEGREE: f64 = 111_320.0;
            [
                (p.1 - point.lon) * METERS_PER_DEGREE * point.lat.to_radians().cos(),
                (p.0 - point.lat) * METERS_PER_DEGREE,
            ]
        };
        let best = self
            .index
            .segments
            .nearest_neighbor_iter(&query)
            .take(4)
            .map(|entry| {
                let (edge, segment) =
                    (EdgeIndex::new(entry.data.0 as usize), entry.data.1 as usize);
                let points: Vec<(f64, f64)> = edge_geometry(&self.graph, edge).points().collect();
                let (a, b) = (local(points[segment]), local(points[segment + 1]));
                let d = [b[0] - a[0], b[1] - a[1]];
                let length_2 = d[0] * d[0] + d[1] * d[1];
                let s = if length_2 > 0.0 {
                    (-(a[0] * d[0] + a[1] * d[1]) / length_2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let foot = [a[0] + s * d[0], a[1] + s * d[1]];
                (
                    foot[0] * foot[0] + foot[1] * foot[1],
                    edge,
                    segment,
                    s,
                    points,
                )
            })
            .min_by(|x, y| x.0.total_cmp(&y.0));
        let Some((_, edge, segment, s, points)) = best else {
            return self
                .nearest_node(point)
                .map(|node| self.snap_to_node_from(point, node));
        };

        let (a, b) = (points[segment], points[segment + 1]);
        let snapped = LatLon::new(a.0 + (b.0 - a.0) * s, a.1 + (b.1 - a.1) * s);
        let span = |p: (f64, f64), q: (f64, f64)| calculate_distance(p.0, p.1, q.0, q.1);
        let total: f64 = points.windows(2).map(|w| span(w[0], w[1])).sum();
        let before: f64 = points[..=segment]
            .windows(2)
            .map(|w| span(w[0], w[1]))
            .sum();
        let fraction = if total > 0.0 {
            ((before + s * span(a, b)) / total).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let (source, target) = self.graph.edge_endpoints(edge).expect("indexed edge");
        let node_index = if fraction <= 0.5 { source } else { target };
        let node = &self.graph[node_index];
        Some(SnapResult {
            input_lat: point.lat,
            input_lon: point.lon,
            snapped_lat: snapped.lat,
            snapped_lon: snapped.lon,
            distance_m: point.distance_m(snapped),
            edge: Some(edge),
            fraction,
            node_index,
            node_id: node.id,
            node_lat: node.lat,
            node_lon: node.lon,
        })
    }

    /// A snap exactly at graph node `node`, for node-based queries.
    pub fn snap_to_node(&self, node: NodeIndex) -> SnapResult {
        self.snap_to_node_from((&self.graph[node]).into(), node)
    }

    fn snap_to_node_from(&self, point: LatLon, node_index: NodeIndex) -> SnapResult {
        let node = &self.graph[node_index];
        SnapResult {
            input_lat: point.lat,
            input_lon: point.lon,
            snapped_lat: node.lat,
            snapped_lon: node.lon,
            distance_m: point.distance_m(node.into()),
            edge: None,
            fraction: 0.0,
            node_index,
            node_id: node.id,
            node_lat: node.lat,
            node_lon: node.lon,
        }
    }

    /// The edge running the opposite way along the same road as `edge`, if
    /// one exists.
    pub(crate) fn twin(&self, edge: EdgeIndex) -> Option<EdgeIndex> {
        let (u, v) = self.graph.edge_endpoints(edge)?;
        let length = self.graph[edge].length;
        self.graph
            .edges_connecting(v, u)
            .filter(|r| (r.weight().length - length).abs() <= 1.0 + 0.01 * length)
            .min_by(|a, b| {
                (a.weight().length - length)
                    .abs()
                    .total_cmp(&(b.weight().length - length).abs())
            })
            .map(|r| r.id())
    }

    /// Search states a search leaving through `anchors` starts from: the
    /// state reached along the anchor's partial edge, or the node itself.
    pub(crate) fn departure_roots(&self, anchors: &[Anchor]) -> Roots {
        let index = self.search_index();
        anchors
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let state = match a.piece {
                    Some(piece) => index.head_state(piece.edge, a.node),
                    None => a.node.index() as u32,
                };
                (state, a.cost, i)
            })
            .collect()
    }

    /// Search states a search arriving through `anchors` may end in: those
    /// allowed to take the anchor's partial edge, or any state of the node.
    pub(crate) fn arrival_roots(&self, anchors: &[Anchor]) -> Roots {
        let index = self.search_index();
        anchors
            .iter()
            .enumerate()
            .flat_map(|(i, a)| {
                let states: Vec<(u32, f64)> = match a.piece {
                    Some(piece) => index.tail_states(piece.edge, a.node).collect(),
                    None => index.states_of(a.node).map(|state| (state, 0.0)).collect(),
                };
                states
                    .into_iter()
                    .map(move |(state, turn)| (state, a.cost + turn, i))
            })
            .collect()
    }

    /// Coordinates of the graph node a search state belongs to.
    pub(crate) fn state_point(&self, state: u32) -> LatLon {
        let node = self.search_index().node_of(state);
        (&self.graph[NodeIndex::new(node as usize)]).into()
    }

    /// Ways onto the network from `snap`, priced by `cost`.
    pub(crate) fn departures(
        &self,
        snap: &SnapResult,
        cost: &mut dyn FnMut(EdgeIndex) -> f64,
    ) -> Vec<Anchor> {
        self.anchors(snap, cost, true)
    }

    /// Ways from the network to `snap`, priced by `cost`.
    pub(crate) fn arrivals(
        &self,
        snap: &SnapResult,
        cost: &mut dyn FnMut(EdgeIndex) -> f64,
    ) -> Vec<Anchor> {
        self.anchors(snap, cost, false)
    }

    fn anchors(
        &self,
        snap: &SnapResult,
        cost: &mut dyn FnMut(EdgeIndex) -> f64,
        depart: bool,
    ) -> Vec<Anchor> {
        let Some(edge) = snap.edge else {
            return vec![Anchor {
                node: snap.node_index,
                cost: 0.0,
                piece: None,
            }];
        };
        let (u, v) = self.graph.edge_endpoints(edge).expect("snapped edge");
        let t = snap.fraction;
        let mut anchors = Vec::with_capacity(3);
        let mut push = |node, edge, from: f64, to: f64, cost: f64| {
            if cost.is_finite() && cost >= 0.0 {
                let piece = Piece { edge, from, to };
                anchors.push(Anchor {
                    node,
                    cost: cost * piece.share(),
                    piece: Some(piece),
                });
            }
        };
        let forward = cost(edge);
        if depart {
            push(v, edge, t, 1.0, forward);
        } else {
            push(u, edge, 0.0, t, forward);
        }
        if let Some(twin) = self.twin(edge) {
            let backward = cost(twin);
            if depart {
                push(u, twin, 1.0 - t, 1.0, backward);
            } else {
                push(v, twin, 0.0, 1.0 - t, backward);
            }
        }
        for (node, at_node) in [(u, t <= 0.0), (v, t >= 1.0)] {
            if at_node {
                anchors.push(Anchor {
                    node,
                    cost: 0.0,
                    piece: None,
                });
            }
        }
        anchors
    }

    /// The cheapest way from `origin` to `destination` along the single road
    /// both lie on, without passing through a node, if there is one.
    pub(crate) fn direct_piece(
        &self,
        origin: &SnapResult,
        destination: &SnapResult,
        cost: &mut dyn FnMut(EdgeIndex) -> f64,
    ) -> Option<(f64, Piece)> {
        let (eo, ed) = (origin.edge?, destination.edge?);
        let twin = self.twin(eo);
        let mut best: Option<(f64, Piece)> = None;
        for (edge, flipped) in [(Some(eo), false), (twin, true)] {
            let Some(edge) = edge else { continue };
            let from = if flipped {
                1.0 - origin.fraction
            } else {
                origin.fraction
            };
            let to = if ed == edge {
                destination.fraction
            } else if self.twin(ed) == Some(edge) {
                1.0 - destination.fraction
            } else {
                continue;
            };
            let edge_cost = cost(edge);
            if to >= from && edge_cost.is_finite() && edge_cost >= 0.0 {
                let piece = Piece { edge, from, to };
                let total = edge_cost * piece.share();
                if best.is_none_or(|(b, _)| total < b) {
                    best = Some((total, piece));
                }
            }
        }
        best
    }

    /// Snap a coordinate onto the graph, rejecting snaps farther than
    /// `max_distance_m` when given.
    pub fn snap_point_within(
        &self,
        point: impl Into<LatLon>,
        max_distance_m: Option<f64>,
    ) -> Option<SnapResult> {
        self.snap_point(point)
            .filter(|snap| max_distance_m.is_none_or(|max| snap.distance_m <= max))
    }

    /// Snap a query endpoint, turning failures into the matching error.
    pub(crate) fn snap_endpoint(
        &self,
        point: LatLon,
        role: Role,
        max_snap_m: Option<f64>,
    ) -> Result<SnapResult, OsmGraphError> {
        let snap = self.snap_point(point).ok_or(match role {
            Role::Origin => OsmGraphError::OriginNodeNotFound,
            Role::Destination => OsmGraphError::DestinationNodeNotFound,
        })?;
        match max_snap_m {
            Some(max_distance_m) if snap.distance_m > max_distance_m => {
                Err(OsmGraphError::SnapDistanceExceeded {
                    role: match role {
                        Role::Origin => "origin",
                        Role::Destination => "destination",
                    },
                    distance_m: snap.distance_m,
                    max_distance_m,
                })
            }
            _ => Ok(snap),
        }
    }
}

/// Segments of every edge for the snapping index. Of two edges running
/// opposite ways along one road only one is indexed; snapping finds the
/// other through [`SpatialGraph::twin`].
fn segment_entries(graph: &RoadGraph) -> Vec<SegmentEntry> {
    let mut entries = Vec::new();
    for edge in graph.edge_references() {
        let (u, v) = (edge.source(), edge.target());
        let has_indexed_twin = u > v
            && graph.edges_connecting(v, u).any(|r| {
                (r.weight().length - edge.weight().length).abs()
                    <= 1.0 + 0.01 * edge.weight().length
            });
        if has_indexed_twin || u == v || crate::transit::is_transit(&edge.weight().tags) {
            continue;
        }
        let geometry = edge.weight().oriented_geometry(&graph[u], &graph[v]);
        let projected: Vec<[f64; 2]> = geometry
            .points()
            .map(|(lat, lon)| spatial_index_point(lat, lon))
            .collect();
        for (i, pair) in projected.windows(2).enumerate() {
            entries.push(GeomWithData::new(
                Line::new(pair[0], pair[1]),
                (edge.id().index() as u32, i as u32),
            ));
        }
    }
    entries
}

/// The largest `straight-line distance / travel time` over all edges for
/// `field`. Because great-circle distance obeys the triangle inequality,
/// `distance(n, goal) / max_speed` never exceeds the true remaining cost.
fn max_straight_line_speed(graph: &RoadGraph, field: CostField) -> f64 {
    let mut max_speed = 0.0_f64;
    for edge in graph.edge_references() {
        let (a, b) = (&graph[edge.source()], &graph[edge.target()]);
        // Along-road length too: routes may start or end part-way along an
        // edge, and the bound must hold for those partial stretches.
        let distance = calculate_distance(a.lat, a.lon, b.lat, b.lon).max(edge.weight().length);
        let time = edge.weight().cost(field);
        if distance > 0.0 && time.is_finite() && time >= 0.0 {
            let speed = if time > 0.0 {
                distance / time
            } else {
                f64::INFINITY
            };
            max_speed = max_speed.max(speed);
        }
    }
    // Pad by a hair so floating-point rounding can't make the bound inadmissible.
    max_speed * (1.0 + 1e-9)
}

#[cfg(test)]
mod tests {
    use super::*;
    use petgraph::visit::EdgeRef;

    fn make_node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode {
            id,
            lat,
            lon,
            tags: vec![],
        }
    }

    fn make_way_raw(node_ids: Vec<i64>, tags: Vec<(&str, &str)>) -> OsmWay {
        OsmWay {
            id: 1,
            nodes: node_ids
                .into_iter()
                .map(|id| OsmNodeRef { node_id: id })
                .collect(),
            tags: tags
                .into_iter()
                .map(|(k, v)| OsmTag {
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
        let sg = SpatialGraph::new(
            create_graph(nodes, vec![way], true, false),
            NetworkType::Drive,
        );

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
        let sg = SpatialGraph::new(
            create_graph(nodes, vec![way], true, false),
            NetworkType::Drive,
        );

        let idx = sg.node_index(7).unwrap();
        assert_eq!(sg.graph[idx].id, 7);
        assert_eq!(sg.node_index(8), None);
    }

    #[test]
    fn snapping_lands_on_the_nearest_road_not_the_nearest_node() {
        // A 1 km straight road with nodes only at its ends.
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.0, 0.009)];
        let way = make_way_raw(vec![1, 2], vec![("highway", "residential")]);
        let sg = SpatialGraph::new(
            create_graph(nodes, vec![way], true, false),
            NetworkType::Walk,
        );

        let snap = sg.snap_point((0.0001, 0.003)).unwrap();

        assert!(snap.edge.is_some());
        assert!((snap.distance_m - 11.1).abs() < 0.2, "{}", snap.distance_m);
        assert!(
            (snap.fraction - 1.0 / 3.0).abs() < 1e-6,
            "{}",
            snap.fraction
        );
        assert!((snap.snapped_lon - 0.003).abs() < 1e-9 && snap.snapped_lat.abs() < 1e-9);
        assert_eq!(snap.node_id, 1, "nearest end of the snapped edge");
    }

    #[test]
    fn piece_points_cut_the_geometry_at_fractions() {
        let mut graph = RoadGraph::new();
        let a = graph.add_node(make_node(1, 0.0, 0.0));
        let b = graph.add_node(make_node(2, 0.002, 0.0));
        let edge = graph.add_edge(a, b, Edge::default());

        let points = Piece {
            edge,
            from: 0.25,
            to: 0.75,
        }
        .points(&graph);

        assert_eq!(points.len(), 2);
        assert!((points[0].0 - 0.0005).abs() < 1e-12);
        assert!((points[1].0 - 0.0015).abs() < 1e-12);
        assert_eq!(
            Piece::whole(edge).points(&graph),
            vec![(0.0, 0.0), (0.002, 0.0)]
        );
    }

    #[test]
    fn profile_sets_speeds_and_overrides_road_classes() {
        use crate::profile::{BuildOptions, Profile};
        let nodes = vec![make_node(1, 0.0, 0.0), make_node(2, 0.009, 0.0)];
        let way = make_way_raw(vec![1, 2], vec![("highway", "primary"), ("maxspeed", "70")]);
        let options = BuildOptions {
            retain_all: true,
            profile: Profile {
                walk_speed_kph: 4.0,
                use_maxspeed: false,
                ..Profile::default().with_drive_speed("primary", 40.0)
            },
        };

        let graph = create_graph_with(nodes, vec![way], false, &options);

        let edge = graph.edge_weights().next().unwrap();
        assert_eq!(edge.speed_kph, 40.0, "maxspeed ignored, class speed used");
        assert!((edge.walk_travel_time - edge.length / (4.0 / 3.6)).abs() < 1e-9);
        assert!((edge.bike_travel_time - edge.length / (15.0 / 3.6)).abs() < 1e-9);
    }

    #[test]
    fn test_nearest_node_finds_closest() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        graph.add_node(make_node(2, 52.0, 13.0));
        let sg = SpatialGraph::new(graph, NetworkType::Drive);
        let idx = sg.nearest_node((48.001, 11.001)).unwrap();
        assert_eq!(sg.graph[idx].id, 1);
    }

    #[test]
    fn test_snap_point_returns_diagnostics() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        let sg = SpatialGraph::new(graph, NetworkType::Drive);

        let snap = sg.snap_point((48.001, 11.001)).unwrap();

        assert_eq!(snap.node_id, 1);
        assert_eq!(snap.node_lat, 48.0);
        assert_eq!(snap.node_lon, 11.0);
        assert!(snap.distance_m > 0.0);
    }

    #[test]
    fn test_snap_point_within_rejects_far_snap() {
        let mut graph = DiGraph::new();
        graph.add_node(make_node(1, 48.0, 11.0));
        let sg = SpatialGraph::new(graph, NetworkType::Drive);

        assert!(sg
            .snap_point_within((48.001, 11.001), Some(500.0))
            .is_some());
        assert!(sg.snap_point_within((48.001, 11.001), Some(1.0)).is_none());
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
