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
    let graph = SpatialGraph::from_pbf("data/munich.osm.pbf", NetworkType::Walk, false)?;
    let marienplatz = (48.1374, 11.5755);

    // Areas reachable within 5, 10 and 15 minutes, as multipolygons.
    let isochrones = graph.isochrones(marienplatz, &[300.0, 600.0, 900.0], Some(100.0))?;

    // Exact routes; sub-millisecond once routing is prepared.
    graph.prepare_routing();
    let route = graph.route(marienplatz, (48.1500, 11.5600), Some(100.0))?;
    println!("{:.0} s, {:.0} m", route.duration_s, route.distance_m);

    // Save the prepared graph; loading it takes a fraction of the build time.
    graph.save("munich-walk.graph")?;
    let graph = SpatialGraph::load("munich-walk.graph")?;
    assert!(graph.is_routing_prepared());
    println!("{} isochrones", isochrones.len());
    Ok(())
}
```

Coordinates are `(lat, lon)` tuples or [`LatLon`](#latlon) values anywhere a
method takes `impl Into<LatLon>`. Polygons follow the `geo` convention
(`x` = longitude, `y` = latitude), which is also GeoJSON's order.

---

## Cargo features

| Feature | Default | What it adds |
| --- | --- | --- |
| `network` | yes | Downloading from Overpass and Nominatim (`SpatialGraph::from_place`, `from_point`, `geocoding`, POI fetching). Pulls in `reqwest` and `tokio`. |
| `extension-module` | no | The Python bindings (maturin enables it). Implies `network`. |

Loading local PBF or XML files needs neither:

```toml
graphways = { version = "0.5", default-features = false }
```

---

## Building a graph

```rust
use graphways::graph::SpatialGraph;
use graphways::overpass::NetworkType;
use graphways::profile::{BuildOptions, Profile};

// Local PBF extract (fastest; POIs are pre-snapped too).
let graph = SpatialGraph::from_pbf("area.osm.pbf", NetworkType::Drive, false)?;

// OSM XML you already have.
let graph = SpatialGraph::from_osm(&xml, NetworkType::Walk, false)?;

// Downloaded on demand (async, `network` feature); cached in memory and on disk.
let graph = SpatialGraph::from_place("Munich", 5_000.0, NetworkType::Bike, &BuildOptions::default()).await?;
let graph = SpatialGraph::from_point((48.137, 11.575), 2_000.0, NetworkType::Walk, &BuildOptions::default()).await?;

// Your own petgraph, optionally with banned turns.
let graph = SpatialGraph::new(road_graph, NetworkType::Drive);
```

The `_with` variants (`from_pbf_with`, `from_osm_with`, `from_osm_data_with`)
take [`BuildOptions`](#speed-profiles) with a custom speed profile.
`retain_all = true` keeps every OSM node instead of simplifying the graph.

Driving graphs (`Drive`, `DriveService`) read `type=restriction` relations
and forbid the banned turns in every search. `restrictions::parse_restrictions`
and `restrictions::forbidden_turns` expose the same steps for graphs you build
yourself; pass the result to `SpatialGraph::with_forbidden_turns`.

### Speed profiles

```rust
let profile = Profile {
    walk_speed_kph: 3.5,
    ..Profile::default()
}
.with_drive_speed("residential", 20.0);
let options = BuildOptions { retain_all: false, profile };
let graph = SpatialGraph::from_pbf_with("area.osm.pbf", NetworkType::Walk, &options)?;
```

`Profile` holds walking and cycling speeds, a driving speed per `highway=*`
class (with a fallback), whether `maxspeed` tags override them, the distance
under which intersections merge during simplification, the delay at traffic
signals, and `turn_costs: TurnCosts` for driving.

`TurnCosts` follows OSRM's car profile: nearly free straight on, about 2 s for
a right and 5 s for a left turn in right-hand traffic, plus 20 s for a
U-turn; `left_hand_traffic` mirrors it and `TurnCosts::none()` turns it off.
Driving graphs built from OSM data get these costs automatically; for your
own graphs pass `(into, out, seconds)` triples to
`SpatialGraph::with_turns`. Turn costs are added to whatever edge cost a
query uses, including custom cost closures.

### Saving and loading

```rust
graph.prepare_routing();          // optional: the routing index is saved too
graph.save("area.graph")?;
let graph = SpatialGraph::load("area.graph")?;
```

The file holds the road graph, turn restrictions, POI snaps and, if built,
the routing index. On the Munich walking graph, building and preparing takes
about 8 s and loading the saved file under 1 s. Files carry a format version;
an incompatible or damaged file fails with `OsmGraphError::InvalidGraphFile`.

---

## Queries

| Method | Returns |
| --- | --- |
| `snap_point(point)` | `Option<SnapResult>`: the nearest point on any road |
| `nearest_node(point)` | `Option<NodeIndex>` |
| `node_index(osm_id)` | `Option<NodeIndex>` |
| `route(origin, destination, max_snap_m)` | `Result<Route, _>` |
| `route_with(origin, destination, max_snap_m, cost)` | `Result<Route, _>` under a custom edge cost |
| `travel_time_matrix(origins, destinations, max_snap_m)` | `TravelTimeMatrix` |
| `travel_time_matrix_with(origins, destinations, max_snap_m, cost)` | `TravelTimeMatrix` under a custom edge cost |
| `accessibility(origins, opportunities, weights, &decays, max_snap_m)` | `Result<Vec<Option<Vec<f64>>>, _>` |
| `nearest_destinations(origins, destinations, k, max_snap_m)` | `Vec<Vec<NearbyDestination>>` |
| `reachability(origin, max_time, max_snap_m)` | `Result<ReachabilityResult, _>` |
| `reachable_graph(origin, max_time, max_snap_m)` | `Result<ReachableGraph, _>` |
| `isochrones(origin, &limits, max_snap_m)` | `Result<Vec<MultiPolygon>, _>` |
| `prism(origin, destination, available_time, max_snap_m)` | `Result<PrismGraph, _>` |
| `with_transit(gtfs, &TransitOptions)` | `Result<(SpatialGraph, TransitSummary), _>`: walking plus public transport |
| `reachable_pois(origin, max_time)` | async, `network` feature |

Times are in seconds for the graph's network type. Every query point snaps to
the closest point on a road, so searches start and end part-way along edges.
`max_snap_m` rejects points farther than that from any road with
`OsmGraphError::SnapDistanceExceeded`.

### Routing

`prepare_routing()` builds a contraction hierarchy (about 1.5 s for a city's
driving graph and 5 s for a dense walking graph, on all cores). Routes then
take well under a millisecond. Without it, `route` uses A\* with a
straight-line lower bound; both are exact. The hierarchy is shared by every
clone of the graph.

```rust
pub struct Route {
    pub coordinates: Vec<(f64, f64)>,   // (lat, lon) per waypoint
    pub cumulative_times_s: Vec<f64>,   // parallel to coordinates
    pub distance_m: f64,
    pub duration_s: f64,
    pub pieces: Vec<Piece>,             // stretches of road travelled, in order
    pub origin_snap: SnapResult,
    pub destination_snap: SnapResult,
}
```

`route_with` takes a closure `Fn(EdgeInfo) -> f64` (live traffic, penalties);
negative, NaN or infinite costs make an edge impassable.

### Travel-time matrices

```rust
let matrix = graph.travel_time_matrix(&homes, &clinics, Some(250.0));
let seconds: Option<f64> = matrix.durations_s[i][j];
let metres: Option<f64> = matrix.distances_m[i][j]; // length of that fastest route
```

With routing prepared, a matrix costs one small search per point (Munich,
1000 x 1000: about 80 ms driving, 150 ms walking). Unprepared graphs and
custom costs run one Dijkstra search per point on the smaller side. Points
too far from any road get a `None` snap and `None` times instead of failing
the whole matrix.

### Accessibility

```rust
use graphways::accessibility::Decay;

let decays = [Decay::Step { cutoff_s: 900.0 }, Decay::Exponential { half_life_s: 600.0 }];
let scores = graph.accessibility(&homes, &jobs, Some(&job_counts), &decays, Some(250.0))?;
// scores[i] is None for an unsnapped origin, else one score per decay.
let nearest = graph.nearest_destinations(&homes, &clinics, 3, Some(250.0));
```

`accessibility` sums `weight × decay(travel time)` over opportunities for
each origin (`Decay::Step`, `Linear`, `Exponential`, `Gaussian`).
`nearest_destinations` returns each origin's `k` fastest destinations with
their index, duration and route length. Both reduce each origin's row as it
is computed, so memory grows with the number of points, not with origins ×
destinations; prepare routing first for more than a handful of origins.

### Public transport

```rust
use graphways::transit::TransitOptions;

let walk = SpatialGraph::from_pbf("munich.osm.pbf", NetworkType::Walk, false)?;
let options = TransitOptions::new("2026-10-06", "07:00", "09:00")?;
let (transit, summary) = walk.with_transit("mvg_gtfs.zip", &options)?;
let isochrones = transit.isochrones((48.137, 11.575), &[900.0, 1800.0], Some(100.0))?;
```

`with_transit` adds a GTFS feed (zip or directory) to a walking graph as a
frequency-based network for one time window: ride edges carry average
running times, boarding costs `wait_factor` (0.5) times the headway, and
changing lines means walking and waiting again. Every query works on the
result unchanged; points still snap only to streets. On Munich the model's
travel times match r5's median over the window with a mean absolute error
of 4.3% (see `benchmarks/transit`). See the `transit` module docs for the
model and its limits.

### Reachability, isochrones and prisms

`reachability` returns every node reachable within the budget:

```rust
pub struct ReachabilityResult {
    pub origin: LatLon,
    pub max_cost: f64,
    pub times: NodeMap<f64>,   // O(1) lookup by NodeIndex, iterated by increasing time
}
```

`ReachabilityResult::time_to(&graph, &snap)` gives the time to any snapped
point, finishing part-way along its road. `reachability::compute_reachability`
and `compute_reachability_with` take an already snapped origin (and, for the
latter, a cost closure). `isochrone::build_isochrone_polygons` turns a result
into polygons for several limits.

Isochrones are `geo::MultiPolygon`s: several parts where the reachable area
is split, and holes for unreachable pockets (rings smaller than
`isochrone::MIN_RING_AREA_M2` are dropped).

`prism` returns the nodes on some origin → node → destination trip within the
time budget, each labelled with inbound time, outbound time and slack; it
fails with `OsmGraphError::Infeasible` when the trip itself does not fit.
`feasibility::compute_feasibility(_with)` is the lower-level form.

---

## Key types

### `SpatialGraph`

```rust
pub struct SpatialGraph {
    pub graph: Arc<RoadGraph>, // RoadGraph = DiGraph<OsmNode, Edge>
    pub poi_snaps: Option<Arc<HashMap<i64, SnappedPoi>>>,
    // network type, turn restrictions and search indexes omitted
}
```

A petgraph graph bundled with what queries need: R-trees of nodes and road
segments, a compact array-based adjacency, and (after `prepare_routing`) a
contraction hierarchy. Everything is shared, so cloning is O(1).
`network_type()` and `forbidden_turns()` report how it was built.

### `LatLon`

```rust
pub struct LatLon { pub lat: f64, pub lon: f64 }
```

Converts from `(f64, f64)` (lat, lon). `distance_m` gives great-circle metres.

### `SnapResult`

```rust
pub struct SnapResult {
    pub input_lat: f64,
    pub input_lon: f64,
    pub snapped_lat: f64,          // the point on the road
    pub snapped_lon: f64,
    pub distance_m: f64,           // input to road
    pub edge: Option<EdgeIndex>,   // the edge snapped onto
    pub fraction: f64,             // position along it, 0 = source, 1 = target
    pub node_index: NodeIndex,     // the edge's nearer endpoint
    pub node_id: i64,
    pub node_lat: f64,
    pub node_lon: f64,
}
```

### `Edge`

The weight of every edge in a `RoadGraph`.

```rust
pub struct Edge {
    pub way_id: i64,
    pub last_way_id: i64,          // way of the last segment (simplified chains)
    pub tags: Arc<[OsmTag]>,       // shared by every edge cut from the same way
    pub length: f64,               // metres
    pub speed_kph: f64,
    pub walk_travel_time: f64,     // seconds
    pub bike_travel_time: f64,
    pub drive_travel_time: f64,
    pub geometry: Vec<(f64, f64)>, // (lat, lon) shape points; empty = straight segment
}
```

Use `edge.travel_time(network_type)`, `edge.tag("highway")` and
`edge.oriented_geometry(&source, &target)` rather than reading the raw fields;
the last one handles straight edges and stored geometry that runs backwards.
`Edge::from_length` and `Edge::with_profile` build edges for your own graphs.

### OSM input types

`OsmData { nodes, ways, relations }` is what `parse_xml` and `pbf::read_pbf`
produce and `SpatialGraph::from_osm_data` consumes, built from `OsmNode`,
`OsmWay`, `OsmRelation`, `OsmMember`, `OsmNodeRef` and `OsmTag`.

---

## Error type

```rust
pub enum OsmGraphError {
    Network(reqwest::Error),       // `network` feature only
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
    Infeasible(InfeasibleReason),
    InvalidGraphFile(String),
}
```

---

## Migrating from 0.4

**Graph construction**

- The network type is fixed when a graph is built and no longer passed to
  queries: `SpatialGraph::new(graph, network_type)`, and `network_type()`
  reads it back. `routing::route(&sg, ...)` is now the method `sg.route(...)`.
- `from_pbf(path, network, retain_all: bool)` and `from_osm(xml, network,
  retain_all: bool)` take a plain `bool`; the `_with` variants take
  `&BuildOptions` for custom speeds.
- The parse types are `OsmData`, `OsmNode`, `OsmWay`, `OsmTag` and
  `OsmNodeRef` (the old `Xml*` names remain as deprecated aliases).
  `OsmData` gained `relations`.
- The edge weight type is `Edge` (was `OsmWay`, now only the parse input).
  `Edge::way_id` replaces `id`; `tags` is a shared `Arc<[OsmTag]>`; straight
  edges have empty `geometry`, so read shapes through
  `Edge::oriented_geometry`. `RoadGraph` names `DiGraph<OsmNode, Edge>`.
- Simplification refits edges touching a merged intersection to the merged
  node positions, so paths no longer cross a junction for free, and it keeps
  parallel roads and loops between the same junctions distinct. Expect walk
  and drive times a few percent longer than 0.4 on dense city graphs.

**Queries**

- Query points are `impl Into<LatLon>`, e.g. `(lat, lon)` tuples, instead of
  separate `lat, lon` arguments: `graph.route(origin, destination, max_snap_m)`,
  `graph.nearest_node(point)`, `graph.snap_point(point)`.
- Points snap to the nearest point on a road rather than the nearest node.
  `SnapResult` gained `snapped_lat`, `snapped_lon`, `edge` and `fraction`;
  `distance_m` is now the distance to the road. Routes and reachability start
  and end part-way along edges.
- Fallible queries return `Result<_, OsmGraphError>` rather than `Option`;
  `prism` returns `Result<PrismGraph, OsmGraphError>` and reports an
  impossible trip as `OsmGraphError::Infeasible`.
- Isochrones are `geo::MultiPolygon`s with holes rather than single polygons;
  `isochrone::isochrones_from(&sg, &snap, &limits)` replaces
  `calculate_isochrones_concurrently`, `build_isochrone_polygons` takes
  `&SpatialGraph`, and `utils::multipolygon_to_geojson(_string)` serializes
  them. Polygon
  coordinates are `x` = longitude, `y` = latitude throughout.
- `compute_reachability(_with)` and `compute_feasibility(_with)` take
  `&SpatialGraph` and a snapped origin (and destination) instead of a node.
- `ReachabilityResult::distances: HashMap` is now `times: NodeMap<f64>`, with
  the snapped `origin`; `FeasibilityResult::feasible` is a
  `NodeMap<FeasibleNode>` and its `origin`/`destination` are `SnapResult`s.
  `NodeMap::get` takes a `NodeIndex` by value.
- Driving graphs obey turn restrictions and price turns and traffic signals
  (see [Speed profiles](#speed-profiles)), so driving times are longer than
  0.4's, typically by 10-15% in a city. Build with
  `Profile { turn_costs: TurnCosts::none(), traffic_signal_s: 0.0, .. }` to
  get the old model back.
