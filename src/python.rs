//! Python bindings, compiled only with the `extension-module` feature
//! (maturin enables it when building the wheel).
//!
//! Every call that does real work (loading, routing, searches, GeoJSON
//! export) releases the GIL, so threaded Python servers stay responsive.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

use geojson::{Feature, JsonObject};
use petgraph::graph::{EdgeReference, NodeIndex};
use petgraph::visit::EdgeRef;
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use crate::error::OsmGraphError;
use crate::graph::{Edge, NodeMap, OsmNode, SnapResult, SpatialGraph};
use crate::overpass::NetworkType;
use crate::profile::{BuildOptions, Profile};
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

const PROFILE_KEYS: &[&str] = &[
    "walk_speed_kph",
    "bike_speed_kph",
    "drive_speeds_kph",
    "default_drive_speed_kph",
    "use_maxspeed",
    "merge_distance_m",
];

/// Build options from `retain_all` and the optional speed-profile keyword
/// arguments shared by every graph constructor. Speeds in
/// `drive_speeds_kph` override the defaults class by class.
fn build_options(retain_all: bool, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<BuildOptions> {
    let mut profile = Profile::default();
    for (key, value) in kwargs.into_iter().flat_map(|kwargs| kwargs.iter()) {
        let key: String = key.extract()?;
        if value.is_none() {
            continue;
        }
        match key.as_str() {
            "walk_speed_kph" => profile.walk_speed_kph = value.extract()?,
            "bike_speed_kph" => profile.bike_speed_kph = value.extract()?,
            "drive_speeds_kph" => profile
                .drive_speeds_kph
                .extend(value.extract::<HashMap<String, f64>>()?),
            "default_drive_speed_kph" => profile.default_drive_speed_kph = value.extract()?,
            "use_maxspeed" => profile.use_maxspeed = value.extract()?,
            "merge_distance_m" => profile.merge_distance_m = value.extract()?,
            _ => {
                return Err(PyTypeError::new_err(format!(
                    "unexpected keyword argument '{key}'; profile options are: {}",
                    PROFILE_KEYS.join(", ")
                )))
            }
        }
    }
    Ok(BuildOptions {
        retain_all,
        profile,
    })
}

/// Snapping failures for area queries (isochrones, reachability, prisms)
/// surface as `ValueError`, as they always have; routes keep `LookupError`.
fn snap_as_value_error(e: OsmGraphError) -> PyErr {
    match e {
        OsmGraphError::OriginNodeNotFound
        | OsmGraphError::DestinationNodeNotFound
        | OsmGraphError::SnapDistanceExceeded { .. } => PyValueError::new_err(e.to_string()),
        other => other.into(),
    }
}

/// Convert minute budgets to seconds, run `compute`, and pair each area with
/// the limit it was requested for.
fn isochrone_results(
    py: Python<'_>,
    minutes: Vec<f64>,
    compute: impl FnOnce(&[f64]) -> Result<Vec<geo::MultiPolygon>, OsmGraphError> + Send,
) -> PyResult<Vec<PyIsochroneResult>> {
    let limits: Vec<f64> = minutes.iter().map(|m| m * 60.0).collect();
    let areas = py
        .detach(|| compute(&limits))
        .map_err(snap_as_value_error)?;
    Ok(minutes
        .into_iter()
        .zip(areas)
        .map(|(minutes, area)| PyIsochroneResult { minutes, area })
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

/// Parse a GeoJSON string into a Python object (dict), for `__geo_interface__`.
fn json_to_python<'py>(py: Python<'py>, json: &str) -> PyResult<Bound<'py, PyAny>> {
    py.import("json")?.call_method1("loads", (json,))
}

/// `shapely.geometry.shape(obj)`, with a clear message if shapely is missing.
fn to_shapely<'py>(obj: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let geometry = obj.py().import("shapely.geometry").map_err(|_| {
        pyo3::exceptions::PyImportError::new_err("to_shapely() needs shapely: pip install shapely")
    })?;
    geometry.call_method1("shape", (obj.getattr("__geo_interface__")?,))
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

fn point(node: &OsmNode) -> geojson::Value {
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
        ("snapped_lat", snap.snapped_lat.into()),
        ("snapped_lon", snap.snapped_lon.into()),
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

fn snap_to_dict(py: Python<'_>, snap: SnapResult) -> PyResult<Bound<'_, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("input_lat", snap.input_lat)?;
    dict.set_item("input_lon", snap.input_lon)?;
    dict.set_item("snapped_lat", snap.snapped_lat)?;
    dict.set_item("snapped_lon", snap.snapped_lon)?;
    dict.set_item("node_id", snap.node_id)?;
    dict.set_item("node_lat", snap.node_lat)?;
    dict.set_item("node_lon", snap.node_lon)?;
    dict.set_item("distance_m", snap.distance_m)?;
    Ok(dict)
}

// ---------------------------------------------------------------------------
// Result classes
// ---------------------------------------------------------------------------

/// Where a coordinate landed on the road network.
#[pyclass(name = "SnapResult", frozen, skip_from_py_object)]
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

    /// Latitude of the point on the road the input snapped to.
    #[getter]
    fn snapped_lat(&self) -> f64 {
        self.snap.snapped_lat
    }

    /// Longitude of the point on the road the input snapped to.
    #[getter]
    fn snapped_lon(&self) -> f64 {
        self.snap.snapped_lon
    }

    /// OSM id of the nearest end of the road segment snapped to.
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

    /// Distance in metres from the input to the road.
    #[getter]
    fn distance_m(&self) -> f64 {
        self.snap.distance_m
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        snap_to_dict(py, self.snap)
    }

    fn __repr__(&self) -> String {
        format!(
            "SnapResult(node_id={}, distance_m={:.1})",
            self.snap.node_id, self.snap.distance_m
        )
    }
}

#[pyclass(name = "RouteResult", frozen, skip_from_py_object)]
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

    /// GeoJSON Feature as a dict; lets shapely, geopandas and others
    /// consume the route directly.
    #[getter]
    fn __geo_interface__<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        json_to_python(py, &self.to_geojson())
    }

    /// The route as a shapely LineString (requires shapely).
    fn to_shapely<'py>(slf: &Bound<'py, Self>) -> PyResult<Bound<'py, PyAny>> {
        to_shapely(slf.as_any())
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let dict = PyDict::new(py);
        dict.set_item("coordinates", self.route.coordinates.clone())?;
        dict.set_item("cumulative_times_s", self.route.cumulative_times_s.clone())?;
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

#[pyclass(name = "IsochroneResult", frozen, skip_from_py_object)]
#[derive(Clone)]
struct PyIsochroneResult {
    minutes: f64,
    area: geo::MultiPolygon<f64>,
}

#[pymethods]
impl PyIsochroneResult {
    #[getter]
    fn minutes(&self) -> f64 {
        self.minutes
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let dict = PyDict::new(py);
        dict.set_item("minutes", self.minutes)?;
        dict.set_item("geojson", self.to_geojson())?;
        Ok(dict)
    }

    /// GeoJSON MultiPolygon geometry: one or more parts, each possibly with
    /// holes where nothing is reachable in time.
    fn to_geojson(&self) -> String {
        utils::multipolygon_to_geojson_string(&self.area)
    }

    #[getter]
    fn __geo_interface__<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        json_to_python(py, &self.to_geojson())
    }

    /// The area as a shapely MultiPolygon (requires shapely).
    fn to_shapely<'py>(slf: &Bound<'py, Self>) -> PyResult<Bound<'py, PyAny>> {
        to_shapely(slf.as_any())
    }

    fn __repr__(&self) -> String {
        format!(
            "IsochroneResult(minutes={:.1}, parts={})",
            self.minutes,
            self.area.0.len()
        )
    }
}

fn poi_to_dict<'py>(py: Python<'py>, poi: &poi::Poi) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("id", poi.id)?;
    dict.set_item("lat", poi.lat)?;
    dict.set_item("lon", poi.lon)?;
    dict.set_item("tags", poi.tags.clone())?;
    Ok(dict)
}

#[pyclass(name = "Poi", frozen, skip_from_py_object)]
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

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        poi_to_dict(py, &self.poi)
    }

    fn __repr__(&self) -> String {
        let name = self.poi.tags.get("name").map_or("unnamed", String::as_str);
        format!("Poi(id={}, name={:?})", self.poi.id, name)
    }
}

#[pyclass(name = "PoiCollection", frozen, skip_from_py_object)]
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

    #[getter]
    fn __geo_interface__<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        json_to_python(py, &self.to_geojson())
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
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
#[pyclass(name = "SpatialGraph", frozen, skip_from_py_object)]
struct PyGraph {
    sg: SpatialGraph,
    routing_requested: AtomicBool,
}

impl PyGraph {
    fn new(sg: SpatialGraph) -> Self {
        Self {
            routing_requested: AtomicBool::new(sg.is_routing_prepared()),
            sg,
        }
    }

    /// Start building the routing index on a background thread, once.
    /// Routes are answered with A* until it is ready, so no call waits on it.
    fn prepare_routing_in_background(&self) {
        if !self.routing_requested.swap(true, Ordering::Relaxed) {
            let sg = self.sg.clone();
            std::thread::spawn(move || sg.prepare_routing());
        }
    }
}

#[pymethods]
impl PyGraph {
    #[staticmethod]
    #[pyo3(signature = (path, network, retain_all = false, **profile))]
    fn from_pbf(
        py: Python<'_>,
        path: String,
        network: String,
        retain_all: bool,
        profile: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let options = build_options(retain_all, profile)?;
        let sg = py.detach(|| SpatialGraph::from_pbf_with(path, network_type, &options))?;
        Ok(Self::new(sg))
    }

    #[staticmethod]
    #[pyo3(signature = (xml, network, retain_all = false, **profile))]
    fn from_osm(
        py: Python<'_>,
        xml: String,
        network: String,
        retain_all: bool,
        profile: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let options = build_options(retain_all, profile)?;
        let sg = py
            .detach(|| SpatialGraph::from_osm_with(&xml, network_type, &options))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self::new(sg))
    }

    #[staticmethod]
    #[pyo3(signature = (place, network, max_dist = None, retain_all = false, **profile))]
    fn from_place(
        py: Python<'_>,
        place: String,
        network: String,
        max_dist: Option<f64>,
        retain_all: bool,
        profile: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        let network_type = parse_network_type(&network)?;
        let options = build_options(retain_all, profile)?;
        let sg = py.detach(|| -> Result<SpatialGraph, OsmGraphError> {
            let (lat, lon) = tokio_rt().block_on(geocoding::geocode(&place))?;
            let (_, sg) = tokio_rt().block_on(isochrone::calculate_isochrones_from_point(
                lat,
                lon,
                Some(max_dist.unwrap_or(5_000.0)),
                vec![],
                network_type,
                &options,
            ))?;
            Ok(sg)
        })?;
        Ok(Self::new(sg))
    }

    /// Load a graph written by `save()`.
    ///
    /// Much faster than rebuilding from OpenStreetMap, and the routing
    /// index comes back ready if it was built before saving.
    #[staticmethod]
    fn load(py: Python<'_>, path: std::path::PathBuf) -> PyResult<Self> {
        let sg = py.detach(|| SpatialGraph::load(path))?;
        Ok(Self::new(sg))
    }

    /// Write the graph to `path` for a fast `SpatialGraph.load()` later.
    ///
    /// By default the routing index is built first (if it isn't already) so
    /// the loaded graph routes at full speed straight away. Pass
    /// `prepare_routing=False` for a smaller file that builds it on demand.
    #[pyo3(signature = (path, prepare_routing = true))]
    fn save(
        &self,
        py: Python<'_>,
        path: std::path::PathBuf,
        prepare_routing: bool,
    ) -> PyResult<()> {
        if prepare_routing {
            self.routing_requested.store(true, Ordering::Relaxed);
        }
        py.detach(|| {
            if prepare_routing {
                self.sg.prepare_routing();
            }
            self.sg.save(path)
        })?;
        Ok(())
    }

    fn node_count(&self) -> usize {
        self.sg.graph.node_count()
    }

    fn edge_count(&self) -> usize {
        self.sg.graph.edge_count()
    }

    fn nearest_node(&self, lat: f64, lon: f64) -> Option<(i64, f64, f64)> {
        self.sg.nearest_node((lat, lon)).map(|idx| {
            let n = &self.sg.graph[idx];
            (n.id, n.lat, n.lon)
        })
    }

    /// Snap a coordinate to the nearest point on any road.
    fn snap_point(&self, lat: f64, lon: f64) -> Option<PySnapResult> {
        self.sg
            .snap_point((lat, lon))
            .map(|snap| PySnapResult { snap })
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn isochrone(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(py, minutes, |limits| {
            self.sg.isochrones(origin, limits, max_snap_m)
        })
    }

    /// Build the routing index now and wait for it.
    ///
    /// Optional: the first `route()` call starts the same build in the
    /// background and routes with A* meanwhile. Call this when you want
    /// every route to get the fast path from the start.
    fn prepare_routing(&self, py: Python<'_>) {
        self.routing_requested.store(true, Ordering::Relaxed);
        py.detach(|| self.sg.prepare_routing());
    }

    /// Whether the routing index is built (see `prepare_routing`).
    fn is_routing_prepared(&self) -> bool {
        self.sg.is_routing_prepared()
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
        let route = py.detach(|| self.sg.route(origin, destination, max_snap_m))?;
        Ok(PyRouteResult { route })
    }

    fn fetch_pois(
        &self,
        py: Python<'_>,
        isochrone: &Bound<'_, PyAny>,
    ) -> PyResult<PyPoiCollection> {
        let geojson = if let Ok(s) = isochrone.extract::<String>() {
            s
        } else if let Ok(iso) = isochrone.cast::<PyIsochroneResult>() {
            iso.get().to_geojson()
        } else {
            return Err(PyTypeError::new_err(
                "fetch_pois expects an IsochroneResult or GeoJSON string",
            ));
        };
        let area = poi::parse_area(&geojson)?;
        let pois = py.detach(|| tokio_rt().block_on(poi::fetch_pois_within(&area)))?;
        Ok(PyPoiCollection { pois })
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn reachable(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        minutes: f64,
        max_snap_m: Option<f64>,
    ) -> PyResult<PyReachableGraph> {
        let inner = py
            .detach(|| self.sg.reachable_graph(origin, minutes * 60.0, max_snap_m))
            .map_err(snap_as_value_error)?;
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
    #[allow(clippy::too_many_arguments)]
    fn prism(
        &self,
        py: Python<'_>,
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

        let inner = py
            .detach(|| {
                self.sg
                    .prism(origin, destination, traversal_budget, max_snap_m)
            })
            .map_err(snap_as_value_error)?;
        Ok(PyPrismGraph {
            inner,
            max_time_s,
            stop_time_s,
            buffer_s,
        })
    }

    fn nodes_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
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
        })
    }

    fn edges_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
            feature_collection(
                self.sg
                    .graph
                    .edge_references()
                    .map(|edge| feature(edge_line(&self.sg, edge), edge_properties(edge.weight()))),
            )
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "SpatialGraph(nodes={}, edges={}, network_type={:?})",
            self.sg.graph.node_count(),
            self.sg.graph.edge_count(),
            self.sg.network_type(),
        )
    }
}

// ---------------------------------------------------------------------------
// ReachableGraph
// ---------------------------------------------------------------------------

#[pyclass(name = "ReachableGraph", frozen, skip_from_py_object)]
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

    fn nodes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
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

    fn nodes_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
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
        })
    }

    fn edges_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
            feature_collection(labeled_edges(self.sg(), self.times()).map(
                |(edge, &source_time, &target_time)| {
                    let mut properties = edge_properties(edge.weight());
                    properties.extend(endpoint_ids(self.sg(), edge));
                    properties.insert("source_time_s".into(), source_time.into());
                    properties.insert("target_time_s".into(), target_time.into());
                    feature(edge_line(self.sg(), edge), properties)
                },
            ))
        })
    }

    fn to_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
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
        })
    }

    #[pyo3(signature = (origin, minutes, max_snap_m = Some(100.0)))]
    fn isochrone(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(py, minutes, |limits| {
            self.inner.isochrones(origin, limits, max_snap_m)
        })
    }

    #[pyo3(signature = (origin, destination, max_snap_m = Some(100.0)))]
    fn route(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
    ) -> PyResult<PyRouteResult> {
        let route = py.detach(|| self.inner.route(origin, destination, max_snap_m))?;
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

#[pyclass(name = "PrismGraph", frozen, skip_from_py_object)]
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

    fn nodes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
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

    fn nodes_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
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
        })
    }

    fn edges_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
            feature_collection(labeled_edges(self.sg(), self.feasible()).map(
                |(edge, source, target)| {
                    let mut properties = edge_properties(edge.weight());
                    properties.extend(endpoint_ids(self.sg(), edge));
                    properties.insert("source_slack_s".into(), source.slack.into());
                    properties.insert("target_slack_s".into(), target.slack.into());
                    feature(edge_line(self.sg(), edge), properties)
                },
            ))
        })
    }

    fn to_geojson(&self, py: Python<'_>) -> String {
        py.detach(|| {
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
        })
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
        py: Python<'_>,
        origin: (f64, f64),
        minutes: Vec<f64>,
        max_snap_m: Option<f64>,
    ) -> PyResult<Vec<PyIsochroneResult>> {
        isochrone_results(py, minutes, |limits| {
            self.inner.isochrones(origin, limits, max_snap_m)
        })
    }

    #[pyo3(signature = (origin, destination, max_snap_m = Some(100.0)))]
    fn route(
        &self,
        py: Python<'_>,
        origin: (f64, f64),
        destination: (f64, f64),
        max_snap_m: Option<f64>,
    ) -> PyResult<PyRouteResult> {
        let route = py.detach(|| self.inner.route(origin, destination, max_snap_m))?;
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
fn geocode(py: Python<'_>, place: String) -> PyResult<(f64, f64)> {
    Ok(py.detach(|| tokio_rt().block_on(geocoding::geocode(&place)))?)
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
fn graphways(m: &Bound<'_, PyModule>) -> PyResult<()> {
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
