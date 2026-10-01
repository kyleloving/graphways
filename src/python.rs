//! Python bindings, compiled only with the `extension-module` feature
//! (maturin enables it when building the wheel).

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

use geojson::{Feature, JsonObject};
use petgraph::graph::{EdgeReference, NodeIndex};
use petgraph::visit::EdgeRef;
use pyo3::exceptions::{PyLookupError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyList};

use crate::graph::{Edge, NodeMap, SnapResult, SpatialGraph, XmlNode};
use crate::overpass::NetworkType;
use crate::{cache, feasibility, geocoding, isochrone, poi, reachability, routing, utils};

static TOKIO_RT: OnceLock<tokio::runtime::Runtime> = OnceLock::new();

fn tokio_rt() -> &'static tokio::runtime::Runtime {
    TOKIO_RT.get_or_init(|| tokio::runtime::Runtime::new().expect("failed to create tokio runtime"))
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

fn parse_network_type(s: &str) -> PyResult<NetworkType> {
    match s.trim().to_ascii_lowercase().as_str() {
        "drive" => Ok(NetworkType::Drive),
        "driveservice" | "drive_service" | "drive-service" => Ok(NetworkType::DriveService),
        "walk" | "walking" => Ok(NetworkType::Walk),
        "bike" | "biking" | "bicycle" => Ok(NetworkType::Bike),
        "all" => Ok(NetworkType::All),
        "allprivate" | "all_private" | "all-private" => Ok(NetworkType::AllPrivate),
        _ => Err(PyValueError::new_err(format!(
            "Invalid network '{s}'. Expected one of: drive, drive_service, walk, bike, all, all_private"
        ))),
    }
}

fn no_origin_node() -> PyErr {
    PyValueError::new_err("No graph node found within max_snap_m of the origin coordinates")
}

/// Convert minute budgets to seconds, run `compute`, and pair each polygon
/// with the limit it was requested for.
fn isochrone_results(
    minutes: Vec<f64>,
    compute: impl FnOnce(Vec<f64>) -> Option<Vec<geo::Polygon>>,
) -> PyResult<Vec<PyIsochroneResult>> {
    let time_limits = minutes.iter().map(|m| m * 60.0).collect();
    let polygons = compute(time_limits).ok_or_else(no_origin_node)?;
    Ok(minutes
        .into_iter()
        .zip(polygons)
        .map(|(minutes, polygon)| PyIsochroneResult { minutes, polygon })
        .collect())
}

/// `(id, lat, lon)` of the node in `nodes` closest to `(lat, lon)`.
fn nearest_of(
    sg: &SpatialGraph,
    nodes: impl Iterator<Item = NodeIndex>,
    lat: f64,
    lon: f64,
) -> Option<(i64, f64, f64)> {
    nodes
        .map(|idx| &sg.graph[idx])
        .map(|node| {
            (
                utils::calculate_distance(lat, lon, node.lat, node.lon),
                node,
            )
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, node)| (node.id, node.lat, node.lon))
}

/// Edges of `sg` whose endpoints both carry a label, with those labels.
/// Visits only the labelled nodes' edges, not the whole graph.
fn labeled_edges<'a, L>(
    sg: &'a SpatialGraph,
    labels: &'a NodeMap<L>,
) -> impl Iterator<Item = (EdgeReference<'a, Edge>, &'a L, &'a L)> + 'a {
    labels.iter().flat_map(move |(&source, source_label)| {
        sg.graph
            .edges(source)
            .filter_map(move |edge| Some((edge, source_label, labels.get(edge.target())?)))
    })
}

// ---------------------------------------------------------------------------
// GeoJSON helpers
// ---------------------------------------------------------------------------

fn props<const N: usize>(entries: [(&str, geojson::JsonValue); N]) -> JsonObject {
    entries
        .into_iter()
        .map(|(key, value)| (key.to_owned(), value))
        .collect()
}

fn feature(value: geojson::Value, properties: JsonObject) -> Feature {
    Feature {
        geometry: Some(geojson::Geometry::new(value)),
        properties: Some(properties),
        ..Default::default()
    }
}

fn feature_collection(features: impl IntoIterator<Item = Feature>) -> String {
    geojson::GeoJson::FeatureCollection(geojson::FeatureCollection {
        features: features.into_iter().collect(),
        bbox: None,
        foreign_members: None,
    })
    .to_string()
}

fn point(node: &XmlNode) -> geojson::Value {
    geojson::Value::Point(vec![node.lon, node.lat])
}

fn edge_line(sg: &SpatialGraph, edge: EdgeReference<'_, Edge>) -> geojson::Value {
    let geometry = edge
        .weight()
        .oriented_geometry(&sg.graph[edge.source()], &sg.graph[edge.target()]);
    geojson::Value::LineString(geometry.points().map(|(lat, lon)| vec![lon, lat]).collect())
}

/// An edge's road class, length, speed and per-mode travel times.
fn edge_properties(way: &Edge) -> JsonObject {
    props([
        ("highway", way.tag("highway").unwrap_or("unknown").into()),
        ("length_m", way.length.into()),
        ("speed_kph", way.speed_kph.into()),
        ("drive_time_s", way.drive_travel_time.into()),
        ("walk_time_s", way.walk_travel_time.into()),
        ("bike_time_s", way.bike_travel_time.into()),
    ])
}

/// `source_node_id` / `target_node_id` properties of an edge.
fn endpoint_ids(sg: &SpatialGraph, edge: EdgeReference<'_, Edge>) -> JsonObject {
    props([
        ("source_node_id", sg.graph[edge.source()].id.into()),
        ("target_node_id", sg.graph[edge.target()].id.into()),
    ])
}

fn snap_json(snap: SnapResult) -> geojson::JsonValue {
    geojson::JsonValue::Object(props([
        ("input_lat", snap.input_lat.into()),
        ("input_lon", snap.input_lon.into()),
        ("node_id", snap.node_id.into()),
        ("node_lat", snap.node_lat.into()),
        ("node_lon", snap.node_lon.into()),
        ("distance_m", snap.distance_m.into()),
    ]))
}

fn route_to_geojson(r: &routing::Route) -> String {
    let coords = r
        .coordinates
        .iter()
        .map(|&(lat, lon)| vec![lon, lat])
        .collect();
    let properties = props([
        ("distance_m", r.distance_m.into()),
        ("duration_s", r.duration_s.into()),
        ("origin_snap", snap_json(r.origin_snap)),
        ("destination_snap", snap_json(r.destination_snap)),
        (
            "cumulative_times_s",
            geojson::JsonValue::Array(r.cumulative_times_s.iter().map(|&t| t.into()).collect()),
        ),
    ]);
    geojson::GeoJson::Feature(feature(geojson::Value::LineString(coords), properties)).to_string()
}

fn snap_to_dict(py: Python<'_>, snap: SnapResult) -> PyResult<&PyDict> {
    let dict = PyDict::new(py);
    dict.set_item("input_lat", snap.input_lat)?;
    dict.set_item("input_lon", snap.input_lon)?;
    dict.set_item("node_id", snap.node_id)?;
    dict.set_item("node_lat", snap.node_lat)?;
    dict.set_item("node_lon", snap.node_lon)?;
    dict.set_item("distance_m", snap.distance_m)?;
    Ok(dict)
}

// ---------------------------------------------------------------------------
// Result classes
// ---------------------------------------------------------------------------

#[pyclass(name = "SnapResult")]
#[derive(Clone, Copy)]
struct PySnapResult {
    snap: SnapResult,
}

#[pymethods]
impl PySnapResult {
    #[getter]
    fn input_lat(&self) -> f64 {
        self.snap.input_lat
    }

    #[getter]
    fn input_lon(&self) -> f64 {
        self.snap.input_lon
    }

    #[getter]
    fn node_id(&self) -> i64 {
        self.snap.node_id
    }

    #[getter]
    fn node_lat(&self) -> f64 {
        self.snap.node_lat
    }

    #[getter]
    fn node_lon(&self) -> f64 {
        self.snap.node_lon
    }

    #[getter]
    fn distance_m(&self) -> f64 {
        self.snap.distance_m
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        snap_to_dict(py, self.snap)
    }

    fn __repr__(&self) -> String {
        format!(
            "SnapResult(node_id={}, distance_m={:.1})",
            self.snap.node_id, self.snap.distance_m
        )
    }
}

#[pyclass(name = "RouteResult")]
#[derive(Clone)]
struct PyRouteResult {
    route: routing::Route,
}

#[pymethods]
impl PyRouteResult {
    #[getter]
    fn coordinates(&self) -> Vec<(f64, f64)> {
        self.route.coordinates.clone()
    }

    #[getter]
    fn cumulative_times_s(&self) -> Vec<f64> {
        self.route.cumulative_times_s.clone()
    }

    #[getter]
    fn distance_m(&self) -> f64 {
        self.route.distance_m
    }

    #[getter]
    fn duration_s(&self) -> f64 {
        self.route.duration_s
    }

    #[getter]
    fn origin_snap(&self) -> PySnapResult {
        PySnapResult {
            snap: self.route.origin_snap,
        }
    }

    #[getter]
    fn destination_snap(&self) -> PySnapResult {
        PySnapResult {
            snap: self.route.destination_snap,
        }
    }

    fn to_geojson(&self) -> String {
        route_to_geojson(&self.route)
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        let dict = PyDict::new(py);
        dict.set_item("coordinates", &self.route.coordinates)?;
        dict.set_item("cumulative_times_s", &self.route.cumulative_times_s)?;
        dict.set_item("distance_m", self.route.distance_m)?;
        dict.set_item("duration_s", self.route.duration_s)?;
        dict.set_item("origin_snap", snap_to_dict(py, self.route.origin_snap)?)?;
        dict.set_item(
            "destination_snap",
            snap_to_dict(py, self.route.destination_snap)?,
        )?;
        Ok(dict)
    }

    fn __repr__(&self) -> String {
        format!(
            "RouteResult(distance_m={:.0}, duration_s={:.0}, points={})",
            self.route.distance_m,
            self.route.duration_s,
            self.route.coordinates.len()
        )
    }
}

#[pyclass(name = "IsochroneResult")]
#[derive(Clone)]
struct PyIsochroneResult {
    minutes: f64,
    polygon: geo::Polygon<f64>,
}

#[pymethods]
impl PyIsochroneResult {
    #[getter]
    fn minutes(&self) -> f64 {
        self.minutes
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        let dict = PyDict::new(py);
        dict.set_item("minutes", self.minutes)?;
        dict.set_item("geojson", self.to_geojson())?;
        Ok(dict)
    }

    fn to_geojson(&self) -> String {
        utils::polygon_to_geojson_string(&self.polygon)
    }

    fn __repr__(&self) -> String {
        format!("IsochroneResult(minutes={:.1})", self.minutes)
    }
}

fn poi_to_dict<'py>(py: Python<'py>, poi: &poi::Poi) -> PyResult<&'py PyDict> {
    let dict = PyDict::new(py);
    dict.set_item("id", poi.id)?;
    dict.set_item("lat", poi.lat)?;
    dict.set_item("lon", poi.lon)?;
    dict.set_item("tags", &poi.tags)?;
    Ok(dict)
}

#[pyclass(name = "Poi")]
#[derive(Clone)]
struct PyPoi {
    poi: poi::Poi,
}

#[pymethods]
impl PyPoi {
    #[getter]
    fn id(&self) -> i64 {
        self.poi.id
    }

    #[getter]
    fn lat(&self) -> f64 {
        self.poi.lat
    }

    #[getter]
    fn lon(&self) -> f64 {
        self.poi.lon
    }

    #[getter]
    fn tags(&self) -> HashMap<String, String> {
        self.poi.tags.clone()
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        poi_to_dict(py, &self.poi)
    }

    fn __repr__(&self) -> String {
        let name = self.poi.tags.get("name").map_or("unnamed", String::as_str);
        format!("Poi(id={}, name={:?})", self.poi.id, name)
    }
}

#[pyclass(name = "PoiCollection")]
#[derive(Clone)]
struct PyPoiCollection {
    pois: Vec<poi::Poi>,
}

#[pymethods]
impl PyPoiCollection {
    #[getter]
    fn count(&self) -> usize {
        self.pois.len()
    }

    #[getter]
    fn pois(&self) -> Vec<PyPoi> {
        self.pois.iter().cloned().map(|poi| PyPoi { poi }).collect()
    }

    fn to_geojson(&self) -> String {
        poi::pois_to_geojson(&self.pois)
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<&'py PyDict> {
        let dict = PyDict::new(py);
        let items = PyList::empty(py);
        for poi in &self.pois {
            items.append(poi_to_dict(py, poi)?)?;
        }
        dict.set_item("pois", items)?;
        dict.set_item("count", self.pois.len())?;
        Ok(dict)
    }

    fn __len__(&self) -> usize {
        self.pois.len()
    }

    fn __repr__(&self) -> String {
        format!("PoiCollection(count={})", self.pois.len())
    }
}

// ---------------------------------------------------------------------------
// SpatialGraph
// ---------------------------------------------------------------------------

/// A road-network graph loaded from OpenStreetMap.
///
/// Construct one with `SpatialGraph.from_place(...)`, `SpatialGraph.from_pbf(...)`,
/// or `SpatialGraph.from_osm(...)`, then reuse it for queries over the same area.
#[pyclass(name = "SpatialGraph")]
struct PyGraph {
    sg: SpatialGraph,
    network_type: NetworkType,
    routing_requested: AtomicBool,
}

impl PyGraph {
    fn new(sg: SpatialGraph, network_type: NetworkType) -> Self {
        Self {
            sg,
            network_type,
            routing_requested: AtomicBool::new(false),
        }
    }

    /// Start building the routing index on a background thread, once.
    /// Routes are answered with A* until it is ready, so no call waits on it.
    fn prepare_routing_in_background(&self) {
        if !self.routing_requested.swap(true, Ordering::Relaxed) {
            let (sg, network_type) = (self.sg.clone(), self.network_type);
            std::thread::spawn(move || sg.prepare_routing(network_type));
        }
    }
}

#[pymethods]
impl PyGraph {
    #[staticmethod]
    #[pyo3(signature = (path, network, retain_all = false))]
    fn from_pbf(path: String, network: String, retain_all: bool) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let sg = SpatialGraph::from_pbf(path, network_type, Some(retain_all))?;
        Ok(Self::new(sg, network_type))
    }

    #[staticmethod]
    #[pyo3(signature = (xml, network, retain_all = false))]
    fn from_osm(xml: String, network: String, retain_all: bool) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let sg = SpatialGraph::from_osm(&xml, network_type, Some(retain_all))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self::new(sg, network_type))
    }

    #[staticmethod]
    #[pyo3(signature = (place, network, max_dist = None, retain_all = false))]
    fn from_place(
        place: String,
        network: String,
        max_dist: Option<f64>,
        retain_all: bool,
    ) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let (lat, lon) = tokio_rt().block_on(geocoding::geocode(&place))?;
        let (_, sg) = tokio_rt().block_on(isochrone::calculate_isochrones_from_point(
            lat,
            lon,
            Some(max_dist.unwrap_or(5_000.0)),
            vec![],
            network_type,
            retain_all,
        ))?;
        Ok(Self::new(sg, network_type))
    }

    fn node_count(&self) -> usize {
        self.sg.graph.node_count()
    }

    fn edge_count(&self) -> usize {
        self.sg.graph.edge_count()
    }

    fn nearest_node(&self, lat: f64, lon: f64) -> Option<(i64, f64, f64)> {
        self.sg.nearest_node(lat, lon).map(|idx| {
            let n = &self.sg.graph[idx];
            (n.id, n.lat, n.lon)
        })
    }

    fn snap_point(&self, lat: f64, lon: f64) -> Option<PySnapResult> {
        self.sg
            .snap_point(lat, lon)
            .map(|snap| PySnapResult { snap })
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn isochrone(
        &self,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(minutes, |limits| {
            self.sg
                .isochrones(origin.0, origin.1, limits, self.network_type, max_snap_m)
        })
    }

    /// Build the routing index for this graph's mode now and wait for it.
    ///
    /// Optional: the first `route()` call starts the same build in the
    /// background and routes with A* meanwhile. Call this when you want
    /// every route to get the fast path from the start.
    fn prepare_routing(&self, py: Python<'_>) {
        self.routing_requested.store(true, Ordering::Relaxed);
        py.allow_threads(|| self.sg.prepare_routing(self.network_type));
    }

    /// Whether the routing index is built (see `prepare_routing`).
    fn is_routing_prepared(&self) -> bool {
        self.sg.is_routing_prepared(self.network_type)
    }

    #[pyo3(signature = (origin, destination, max_snap_m = Some(100.0)))]
    fn route(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
    ) -> PyResult<PyRouteResult> {
        self.prepare_routing_in_background();
        let route = py.allow_threads(|| {
            self.sg.route(
                origin.0,
                origin.1,
                destination.0,
                destination.1,
                self.network_type,
                max_snap_m,
            )
        })?;
        Ok(PyRouteResult { route })
    }

    fn fetch_pois(&self, isochrone: &PyAny) -> PyResult<PyPoiCollection> {
        let isochrone_geojson = if let Ok(s) = isochrone.extract::<String>() {
            s
        } else if let Ok(iso) = isochrone.extract::<PyRef<PyIsochroneResult>>() {
            iso.to_geojson()
        } else {
            return Err(PyTypeError::new_err(
                "fetch_pois expects an IsochroneResult or GeoJSON string",
            ));
        };
        let polygon = poi::parse_isochrone(&isochrone_geojson)?;
        let pois = tokio_rt().block_on(poi::fetch_pois_within(&polygon))?;
        Ok(PyPoiCollection { pois })
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn reachable(
        &self,
        origin: (f64, f64),
        minutes: f64,
        max_snap_m: Option<f64>,
    ) -> PyResult<PyReachableGraph> {
        let inner = self
            .sg
            .reachable_graph(
                origin.0,
                origin.1,
                minutes * 60.0,
                self.network_type,
                max_snap_m,
            )
            .ok_or_else(no_origin_node)?;
        Ok(PyReachableGraph { inner })
    }

    #[pyo3(signature = (
        origin,
        destination,
        max_minutes,
        stop_minutes = 0.0,
        buffer_minutes = 0.0,
        max_snap_m = Some(100.0),
    ))]
    fn prism(
        &self,
        origin: (f64, f64),
        destination: (f64, f64),
        max_minutes: f64,
        stop_minutes: f64,
        buffer_minutes: f64,
        max_snap_m: Option<f64>,
    ) -> PyResult<PyPrismGraph> {
        let max_time_s = max_minutes * 60.0;
        let stop_time_s = stop_minutes * 60.0;
        let buffer_s = buffer_minutes * 60.0;
        let traversal_budget = max_time_s - stop_time_s - buffer_s;
        if !traversal_budget.is_finite() || traversal_budget < 0.0 {
            return Err(PyValueError::new_err(
                "max_minutes must be at least stop_minutes + buffer_minutes",
            ));
        }

        let inner = self
            .sg
            .prism(
                origin.0,
                origin.1,
                destination.0,
                destination.1,
                traversal_budget,
                self.network_type,
                max_snap_m,
            )
            .ok_or_else(|| {
                PyValueError::new_err("No node found near the origin or destination coordinates")
            })?
            .map_err(|e| match e {
                feasibility::InfeasibleReason::BudgetTooTight { .. } => {
                    PyValueError::new_err(e.to_string())
                }
                feasibility::InfeasibleReason::NoPathExists => {
                    PyLookupError::new_err(e.to_string())
                }
            })?;

        Ok(PyPrismGraph {
            inner,
            max_time_s,
            stop_time_s,
            buffer_s,
        })
    }

    fn nodes_geojson(&self) -> String {
        feature_collection(self.sg.graph.node_weights().map(|n| {
            feature(
                point(n),
                props([
                    ("id", n.id.into()),
                    ("lat", n.lat.into()),
                    ("lon", n.lon.into()),
                ]),
            )
        }))
    }

    fn edges_geojson(&self) -> String {
        feature_collection(
            self.sg
                .graph
                .edge_references()
                .map(|edge| feature(edge_line(&self.sg, edge), edge_properties(edge.weight()))),
        )
    }

    fn __repr__(&self) -> String {
        format!(
            "SpatialGraph(nodes={}, edges={}, network_type={:?})",
            self.sg.graph.node_count(),
            self.sg.graph.edge_count(),
            self.network_type,
        )
    }
}

// ---------------------------------------------------------------------------
// ReachableGraph
// ---------------------------------------------------------------------------

#[pyclass(name = "ReachableGraph")]
struct PyReachableGraph {
    inner: reachability::ReachableGraph,
}

impl PyReachableGraph {
    fn sg(&self) -> &SpatialGraph {
        &self.inner.graph
    }

    fn times(&self) -> &NodeMap<f64> {
        &self.inner.result.times
    }
}

#[pymethods]
impl PyReachableGraph {
    #[getter]
    fn max_time_s(&self) -> f64 {
        self.inner.result.max_cost
    }

    /// Number of reachable nodes.
    fn node_count(&self) -> usize {
        self.inner.node_count()
    }

    /// Number of directed edges between reachable nodes.
    fn edge_count(&self) -> usize {
        self.inner.edge_count()
    }

    fn contains_node(&self, node_id: i64) -> bool {
        self.inner.contains_node_id(node_id)
    }

    fn nearest_node(&self, lat: f64, lon: f64) -> Option<(i64, f64, f64)> {
        nearest_of(self.sg(), self.times().keys(), lat, lon)
    }

    fn travel_time_to_node_id(&self, node_id: i64) -> Option<f64> {
        self.inner.travel_time_to_node_id(node_id)
    }

    fn nodes<'py>(&self, py: Python<'py>) -> PyResult<&'py PyList> {
        let items = PyList::empty(py);
        for (&idx, &travel_time_s) in self.times() {
            let node = &self.sg().graph[idx];
            let dict = PyDict::new(py);
            dict.set_item("node_id", node.id)?;
            dict.set_item("lat", node.lat)?;
            dict.set_item("lon", node.lon)?;
            dict.set_item("travel_time_s", travel_time_s)?;
            items.append(dict)?;
        }
        Ok(items)
    }

    fn nodes_geojson(&self) -> String {
        feature_collection(self.times().iter().map(|(&idx, &travel_time_s)| {
            let node = &self.sg().graph[idx];
            feature(
                point(node),
                props([
                    ("node_id", node.id.into()),
                    ("lat", node.lat.into()),
                    ("lon", node.lon.into()),
                    ("travel_time_s", travel_time_s.into()),
                ]),
            )
        }))
    }

    fn edges_geojson(&self) -> String {
        feature_collection(labeled_edges(self.sg(), self.times()).map(
            |(edge, &source_time, &target_time)| {
                let mut properties = edge_properties(edge.weight());
                properties.extend(endpoint_ids(self.sg(), edge));
                properties.insert("source_time_s".into(), source_time.into());
                properties.insert("target_time_s".into(), target_time.into());
                feature(edge_line(self.sg(), edge), properties)
            },
        ))
    }

    fn to_geojson(&self) -> String {
        let nodes = self.times().iter().map(|(&idx, &travel_time_s)| {
            let node = &self.sg().graph[idx];
            feature(
                point(node),
                props([
                    ("kind", "node".into()),
                    ("node_id", node.id.into()),
                    ("travel_time_s", travel_time_s.into()),
                ]),
            )
        });
        let edges =
            labeled_edges(self.sg(), self.times()).map(|(edge, &source_time, &target_time)| {
                feature(
                    edge_line(self.sg(), edge),
                    props([
                        ("kind", "edge".into()),
                        ("source_time_s", source_time.into()),
                        ("target_time_s", target_time.into()),
                        ("length_m", edge.weight().length.into()),
                    ])
                    .into_iter()
                    .chain(endpoint_ids(self.sg(), edge))
                    .collect(),
                )
            });
        feature_collection(nodes.chain(edges))
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn isochrone(
        &self,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(minutes, |limits| {
            self.inner
                .isochrones(origin.0, origin.1, limits, max_snap_m)
        })
    }

    #[pyo3(signature = (origin, destination, max_snap_m = Some(100.0)))]
    fn route(
        &self,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
    ) -> PyResult<PyRouteResult> {
        let route =
            self.inner
                .route(origin.0, origin.1, destination.0, destination.1, max_snap_m)?;
        Ok(PyRouteResult { route })
    }

    fn __repr__(&self) -> String {
        format!(
            "ReachableGraph(nodes={}, edges={}, max_time_s={:.0})",
            self.node_count(),
            self.edge_count(),
            self.inner.result.max_cost,
        )
    }
}

// ---------------------------------------------------------------------------
// PrismGraph
// ---------------------------------------------------------------------------

#[pyclass(name = "PrismGraph")]
struct PyPrismGraph {
    inner: feasibility::PrismGraph,
    max_time_s: f64,
    stop_time_s: f64,
    buffer_s: f64,
}

impl PyPrismGraph {
    fn sg(&self) -> &SpatialGraph {
        &self.inner.graph
    }

    fn feasible(&self) -> &NodeMap<feasibility::FeasibleNode> {
        &self.inner.result.feasible
    }
}

#[pymethods]
impl PyPrismGraph {
    #[getter]
    fn max_time_s(&self) -> f64 {
        self.max_time_s
    }

    #[getter]
    fn traversal_budget_s(&self) -> f64 {
        self.inner.result.available_time
    }

    #[getter]
    fn stop_time_s(&self) -> f64 {
        self.stop_time_s
    }

    #[getter]
    fn buffer_s(&self) -> f64 {
        self.buffer_s
    }

    #[getter]
    fn direct_time_s(&self) -> f64 {
        self.inner.result.direct_time
    }

    fn node_count(&self) -> usize {
        self.inner.node_count()
    }

    fn edge_count(&self) -> usize {
        self.inner.edge_count()
    }

    fn contains_node(&self, node_id: i64) -> bool {
        self.inner.contains_node_id(node_id)
    }

    fn nearest_node(&self, lat: f64, lon: f64) -> Option<(i64, f64, f64)> {
        nearest_of(self.sg(), self.feasible().keys(), lat, lon)
    }

    fn slack_at_node_id(&self, node_id: i64) -> Option<f64> {
        self.inner.slack_at_node_id(node_id)
    }

    fn nodes<'py>(&self, py: Python<'py>) -> PyResult<&'py PyList> {
        let items = PyList::empty(py);
        for (&idx, reach) in self.feasible() {
            let node = &self.sg().graph[idx];
            let dict = PyDict::new(py);
            dict.set_item("node_id", node.id)?;
            dict.set_item("lat", node.lat)?;
            dict.set_item("lon", node.lon)?;
            dict.set_item("inbound_time_s", reach.inbound_time)?;
            dict.set_item("outbound_time_s", reach.outbound_time)?;
            dict.set_item("slack_s", reach.slack)?;
            items.append(dict)?;
        }
        Ok(items)
    }

    fn nodes_geojson(&self) -> String {
        feature_collection(self.feasible().iter().map(|(&idx, reach)| {
            let node = &self.sg().graph[idx];
            feature(
                point(node),
                props([
                    ("node_id", node.id.into()),
                    ("lat", node.lat.into()),
                    ("lon", node.lon.into()),
                    ("inbound_time_s", reach.inbound_time.into()),
                    ("outbound_time_s", reach.outbound_time.into()),
                    ("slack_s", reach.slack.into()),
                ]),
            )
        }))
    }

    fn edges_geojson(&self) -> String {
        feature_collection(labeled_edges(self.sg(), self.feasible()).map(
            |(edge, source, target)| {
                let mut properties = edge_properties(edge.weight());
                properties.extend(endpoint_ids(self.sg(), edge));
                properties.insert("source_slack_s".into(), source.slack.into());
                properties.insert("target_slack_s".into(), target.slack.into());
                feature(edge_line(self.sg(), edge), properties)
            },
        ))
    }

    fn to_geojson(&self) -> String {
        let nodes = self.feasible().iter().map(|(&idx, reach)| {
            let node = &self.sg().graph[idx];
            feature(
                point(node),
                props([
                    ("kind", "node".into()),
                    ("node_id", node.id.into()),
                    ("inbound_time_s", reach.inbound_time.into()),
                    ("outbound_time_s", reach.outbound_time.into()),
                    ("slack_s", reach.slack.into()),
                ]),
            )
        });
        let edges = labeled_edges(self.sg(), self.feasible()).map(|(edge, source, target)| {
            feature(
                edge_line(self.sg(), edge),
                props([
                    ("kind", "edge".into()),
                    ("source_slack_s", source.slack.into()),
                    ("target_slack_s", target.slack.into()),
                    ("length_m", edge.weight().length.into()),
                ])
                .into_iter()
                .chain(endpoint_ids(self.sg(), edge))
                .collect(),
            )
        });
        feature_collection(nodes.chain(edges))
    }

    #[pyo3(signature = (min_slack_s = 0.0))]
    fn slack_polygon(&self, min_slack_s: f64) -> Option<String> {
        feasibility::build_feasibility_polygon(&self.sg().graph, &self.inner.result, min_slack_s)
            .map(|p| utils::polygon_to_geojson_string(&p))
    }

    fn slack_polygons(&self, min_slack_values: Vec<f64>) -> Vec<Option<String>> {
        min_slack_values
            .into_iter()
            .map(|min_slack| self.slack_polygon(min_slack))
            .collect()
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn isochrone(
        &self,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(minutes, |limits| {
            self.inner
                .isochrones(origin.0, origin.1, limits, max_snap_m)
        })
    }

    #[pyo3(signature = (origin, destination, max_snap_m = Some(100.0)))]
    fn route(
        &self,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
    ) -> PyResult<PyRouteResult> {
        let route =
            self.inner
                .route(origin.0, origin.1, destination.0, destination.1, max_snap_m)?;
        Ok(PyRouteResult { route })
    }

    fn __repr__(&self) -> String {
        format!(
            "PrismGraph(nodes={}, edges={}, direct_time_s={:.0}, max_time_s={:.0})",
            self.node_count(),
            self.edge_count(),
            self.inner.result.direct_time,
            self.max_time_s,
        )
    }
}

// ---------------------------------------------------------------------------
// Module-level functions
// ---------------------------------------------------------------------------

#[pyfunction]
fn geocode(place: String) -> PyResult<(f64, f64)> {
    Ok(tokio_rt().block_on(geocoding::geocode(&place))?)
}

#[pyfunction]
fn clear_cache() -> PyResult<()> {
    cache::clear_cache()?;
    cache::clear_disk_cache()?;
    Ok(())
}

#[pyfunction]
fn cache_dir() -> String {
    cache::disk_cache_dir().to_string_lossy().into_owned()
}

#[pymodule]
fn graphways(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyGraph>()?;
    m.add_class::<PyReachableGraph>()?;
    m.add_class::<PyPrismGraph>()?;
    m.add_class::<PySnapResult>()?;
    m.add_class::<PyRouteResult>()?;
    m.add_class::<PyIsochroneResult>()?;
    m.add_class::<PyPoi>()?;
    m.add_class::<PyPoiCollection>()?;
    m.add_function(wrap_pyfunction!(geocode, m)?)?;
    m.add_function(wrap_pyfunction!(clear_cache, m)?)?;
    m.add_function(wrap_pyfunction!(cache_dir, m)?)?;
    Ok(())
}
