# Rust API

The Rust API exposes the same core graph operations without Python overhead.

Full generated API documentation is published alongside this site:
**[Rust API docs ->](https://docs.rs/graphways/)**

---

## Quick example

```rust
use graphways::graph::SpatialGraph;
use graphways::overpass::NetworkType;

fn main() -> Result<(), graphways::error::OsmGraphError> {
    let graph = SpatialGraph::from_pbf(
        "data/district-of-columbia-latest.osm.pbf",
        NetworkType::Walk,
        None,
    )?;

    let reachable = graph.reachable_graph(
        38.9097,
        -77.0432,
        15.0 * 60.0,
        NetworkType::Walk,
        Some(100.0),
    );

    println!(
        "{} nodes, reachable graph exists: {}",
        graph.graph.node_count(),
        reachable.is_some()
    );
    Ok(())
}
```

---

## Key types

### `NetworkType`

```rust
pub enum NetworkType {
    Drive,
    DriveService,
    Walk,
    Bike,
    All,
    AllPrivate,
}
```

Controls which OSM highway tags are included in the graph.  See the [Quickstart](quickstart.md#choosing-a-network-type) for details.

---

### `SpatialGraph`

```rust
pub struct SpatialGraph {
    pub graph: Arc<RoadGraph>, // RoadGraph = DiGraph<OsmNode, Edge>
    pub poi_snaps: Option<Arc<HashMap<i64, SnappedPoi>>>,
    // internal indexes omitted
}
```

A petgraph graph bundled with the indexes queries need: an R-tree for
nearest-node lookups, a compact array-based adjacency for searches, and (after
`prepare_routing`) a contraction hierarchy per mode. Everything is shared, so
cloning a `SpatialGraph` is O(1) and clones see indexes built later.

```rust
// Nearest-node lookup -- O(log n)
let node_idx = sg.nearest_node(lat, lon)?;

// Look a node up by OSM id -- O(log n)
let node_idx = sg.node_index(osm_id)?;

// Direct petgraph access
let node_count = sg.graph.node_count();
let edge_count = sg.graph.edge_count();
```

For local PBF workflows, construct the reusable graph directly:

```rust
use graphways::graph::SpatialGraph;
use graphways::overpass::NetworkType;

let graph = SpatialGraph::from_pbf(
    "data/district-of-columbia-latest.osm.pbf",
    NetworkType::Walk,
    None,
)?;
```

For OSM XML, use the sibling constructor:

```rust
let graph = SpatialGraph::from_osm(xml, NetworkType::Walk, None)?;
```

---

### `OsmNode`

```rust
pub struct OsmNode {
    pub id: i64,
    pub lat: f64,
    pub lon: f64,
    pub tags: Vec<OsmTag>,
}
```

---

### `Edge`

The weight of every edge in a `RoadGraph`.

```rust
pub struct Edge {
    pub way_id: i64,
    pub tags: Arc<[OsmTag]>,       // shared by every edge cut from the same way
    pub length: f64,               // meters
    pub speed_kph: f64,
    pub walk_travel_time: f64,     // seconds
    pub bike_travel_time: f64,     // seconds
    pub drive_travel_time: f64,    // seconds
    pub geometry: Vec<(f64, f64)>, // (lat, lon) shape points; empty = straight segment
}
```

Use `edge.travel_time(network_type)`, `edge.tag("highway")` and
`edge.oriented_geometry(&source, &target)` rather than reading the raw fields;
the last one handles straight edges and stored geometry that runs backwards.

### `OsmWay`

The parsed OSM way that `create_graph` consumes:

```rust
pub struct OsmWay {
    pub id: i64,
    pub nodes: Vec<OsmNodeRef>,
    pub tags: Vec<OsmTag>,
}
```

---

## Core functions

### `routing::route`

```rust
pub fn route(
    sg: &SpatialGraph,
    origin_lat: f64,
    origin_lon: f64,
    dest_lat: f64,
    dest_lon: f64,
    network_type: NetworkType,
    max_snap_m: Option<f64>,
) -> Result<Route, OsmGraphError>
```

Exactly optimal point-to-point routing. Pass `max_snap_m` to reject endpoints
that snap too far from the graph.

Call `SpatialGraph::prepare_routing(network_type)` once to build a contraction
hierarchy for that mode (sub-second for a city's drive graph, a few seconds
for a dense walking graph, using all cores). After that, routes for the mode
take well under a millisecond. Without it, routing uses A\* with a
straight-line lower bound. `SpatialGraph::route_with` routes under a custom
edge-cost closure instead of the built-in travel times.

```rust
graph.prepare_routing(NetworkType::Walk);
let route = graph.route(48.137, 11.575, 48.150, 11.560, NetworkType::Walk, Some(100.0))?;
```

Returns a `Route`:

```rust
pub struct Route {
    pub coordinates: Vec<(f64, f64)>,   // (lat, lon) per waypoint
    pub cumulative_times_s: Vec<f64>,   // parallel to coordinates
    pub distance_m: f64,
    pub duration_s: f64,
    pub origin_snap: SnapResult,
    pub destination_snap: SnapResult,
}
```

---

### `reachability::compute_reachability`

```rust
pub fn compute_reachability(
    sg: &SpatialGraph,
    start: NodeIndex,
    max_cost: f64,
    network_type: NetworkType,
) -> ReachabilityResult
```

One-to-many travel times from `start` within `max_cost` seconds. The result's
`times` is a `NodeMap<f64>`: O(1) lookup by `NodeIndex`, iterated in increasing
order of travel time. `compute_reachability_with` takes a cost closure instead.
`feasibility::compute_feasibility` / `compute_feasibility_with` follow the same
shape for origin-destination prisms.

---

### `overpass::bbox_from_point`

```rust
pub fn bbox_from_point(lat: f64, lon: f64, dist: f64) -> String
```

Construct a `south,west,north,east` bounding-box string for an Overpass API query.

---

### `overpass::make_request`

```rust
pub async fn make_request(url: &str, query: &str) -> Result<String, reqwest::Error>
```

POST a query to an Overpass API endpoint and return the raw XML response.
Transient `429` / `5xx` responses are retried. The default endpoint helpers
respect `GRAPHWAYS_OVERPASS_URL`, `GRAPHWAYS_NOMINATIM_URL`, and
`GRAPHWAYS_USER_AGENT`.

---

## Error type

```rust
pub enum OsmGraphError {
    Network(reqwest::Error),
    XmlParse(quick_xml::DeError),
    EmptyGraph,
    NodeNotFound,
    OriginNodeNotFound,
    DestinationNodeNotFound,
    SnapDistanceExceeded { role: &'static str, distance_m: f64, max_distance_m: f64 },
    PathNotFound,
    LockPoisoned,
    GeocodingFailed(String),
    InvalidInput(String),
    Io(std::io::Error),
    PbfError(String),
}
```

---

## Migrating from 0.4

- The edge weight type is now `Edge` (was `OsmWay`, which is now only the
  parse input). `Edge::way_id` replaces `id`; `tags` is a shared `Arc<[OsmTag]>`;
  straight edges have empty `geometry`, so read shapes through
  `Edge::oriented_geometry`.
- `RoadGraph` names `DiGraph<OsmNode, Edge>`.
- `compute_reachability(_with)` and `compute_feasibility(_with)` take
  `&SpatialGraph` instead of a bare graph.
- `ReachabilityResult::distances: HashMap` is now `times: NodeMap<f64>`, and
  `FeasibilityResult::feasible` is a `NodeMap<FeasibleNode>`. `NodeMap::get`
  takes a `NodeIndex` by value.
- `isochrone::calculate_isochrones_concurrently(Arc<DiGraph>, ..)` is now
  `isochrone::isochrones_from_node(&SpatialGraph, start, &limits, network)`.
- Simplification refits edges touching a merged intersection to the merged
  node positions, so paths no longer cross a junction for free. Expect walk
  and drive times about 2-3% longer than 0.4 on dense city graphs.
