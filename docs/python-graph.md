# Python API - `SpatialGraph`

The `SpatialGraph` object wraps a loaded road-network graph and its spatial index.  
Construct one with `gw.SpatialGraph.from_place(...)`, `from_pbf(...)`, or `from_osm(...)`.

Reusing a `SpatialGraph` across multiple queries avoids rebuilding the network
and lets you run isochrones from many different origin points over the same
loaded graph.

```python
graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="drive", max_dist=10_000)
print(graph)  # SpatialGraph(nodes=6251, edges=15356, network_type=Drive)
```

Every method that does real work releases the GIL, so a `SpatialGraph` can
serve queries from several Python threads at once.

---

## Construction

```python
gw.SpatialGraph.from_place(place, network, max_dist=None, retain_all=False, **profile)
gw.SpatialGraph.from_pbf(path, network, retain_all=False, **profile)
gw.SpatialGraph.from_osm(xml, network, retain_all=False, **profile)
gw.SpatialGraph.load(path)
```

`network` is one of `"drive"`, `"drive_service"`, `"walk"`, `"bike"`, `"all"`
or `"all_private"`. `from_place` geocodes the place and downloads the roads
within `max_dist` metres (default 5 km) from Overpass; `from_pbf` reads a
local extract and needs no network access. Driving graphs obey the turn
restrictions mapped in OpenStreetMap (no left turn, only straight on, ...).

### Speed profiles

Every builder accepts keyword arguments that tune the travel-time model:

| Keyword | Default | Description |
|---------|---------|-------------|
| `walk_speed_kph` | `5.0` | Walking speed on every walkable way |
| `bike_speed_kph` | `15.0` | Cycling speed on every ridable way |
| `drive_speeds_kph` | built in | `{highway_class: kph}` overrides, e.g. `{"residential": 20}` |
| `default_drive_speed_kph` | `50.0` | Driving speed for classes without a configured speed |
| `use_maxspeed` | `True` | Let a way's `maxspeed` tag override its class speed |
| `merge_distance_m` | `5.0` | Merge intersections closer than this when simplifying |
| `traffic_signal_s` | `2.0` | Seconds a driver loses at each traffic signal |
| `turn_penalty_s` | `7.5` | Scale of the time lost turning at junctions when driving |
| `turn_bias` | `1.075` | Above 1 makes turns across oncoming traffic dearer |
| `u_turn_penalty_s` | `20.0` | Extra seconds for a U-turn |
| `left_hand_traffic` | `False` | Traffic drives on the left, so left turns are the cheap ones |

Driving graphs price turns the way OSRM's car profile does: nearly free
straight on, about 2 s for a right turn and 5 s for a left turn (with the
defaults, in right-hand traffic), and about 27 s for a U-turn. Pass
`turn_penalty_s=0, u_turn_penalty_s=0` to switch turn costs off. Turn costs
make the routing index larger: preparing the Munich driving graph takes
about 1.5 s instead of 0.2 s, and routes about 0.2 ms instead of 0.1 ms.

```python
slow_walk = gw.SpatialGraph.from_pbf("munich.osm.pbf", "walk", walk_speed_kph=3.5)
zone_30 = gw.SpatialGraph.from_pbf(
    "munich.osm.pbf", "drive",
    drive_speeds_kph={"residential": 20, "tertiary": 30},
)
```

### `save` / `load`

```python
graph.save(path, prepare_routing=True) -> None
gw.SpatialGraph.load(path) -> SpatialGraph
```

Write the graph to a file and load it back later, much faster than
rebuilding it (Munich walking graph: about 8 s to build and prepare, under
1 s to load). By default `save` builds the routing index first (see
[`route`](#route)) so the loaded graph routes at full speed immediately; pass
`prepare_routing=False` for a smaller file. Loading a file that is not a
graphways graph, or one written by an incompatible version, raises
`ValueError`.

```python
graph = gw.SpatialGraph.from_pbf("munich.osm.pbf", "walk")
graph.save("munich-walk.graph")

# Later, or in another process:
graph = gw.SpatialGraph.load("munich-walk.graph")
```

---

## Inspection

### `node_count`

```python
graph.node_count() -> int
```

Number of nodes in the graph after simplification (unless `retain_all=True` was passed to the constructor).

---

### `edge_count`

```python
graph.edge_count() -> int
```

Number of directed edges in the graph.

---

### `nearest_node`

```python
graph.nearest_node(lat: float, lon: float) -> tuple[int, float, float] | None
```

Return `(osm_id, lat, lon)` for the graph node nearest to `(lat, lon)`.

Uses the internal R-tree spatial index -- O(log n) lookup regardless of graph size.

Returns `None` if the graph is empty.

**Example**

```python
osm_id, node_lat, node_lon = graph.nearest_node(48.137144, 11.575399)
print(f"Snapped to OSM node {osm_id} at ({node_lat:.6f}, {node_lon:.6f})")
```

---

### `snap_point`

```python
graph.snap_point(lat: float, lon: float) -> SnapResult | None
```

Where a coordinate joins the road network: the closest point on any road,
not just the closest intersection. Every query (routes, isochrones,
reachability, prisms, matrices) snaps its points this way, so a trip from the
middle of a long block starts in the middle of that block.

| Property | Description |
|----------|-------------|
| `input_lat`, `input_lon` | The coordinate you passed |
| `snapped_lat`, `snapped_lon` | The point on the road it snapped to |
| `distance_m` | Distance from the input to the road |
| `node_id`, `node_lat`, `node_lon` | The nearer end of the road segment |

---

## Isochrones

### `isochrone`

```python
graph.isochrone(
    origin: tuple[float, float],
    minutes: list[float],
    max_snap_m: float | None = 100.0,
) -> list[IsochroneResult]
```

Compute isochrones from an origin using the travel times of this graph's network type.

One search runs from the origin's snapped road point, and each limit's area
is traced from a triangulated travel-time surface. Isochrones are
`MultiPolygon`s: several parts where the reachable area is split (by a river
or a motorway, say) and holes for unreachable pockets inside it.

**Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `origin` | `tuple[float, float]` | - | `(lat, lon)` origin |
| `minutes` | `list[float]` | - | Travel-time thresholds in minutes |
| `max_snap_m` | `float` or `None` | `100.0` | Reject the query if the origin snaps farther than this many meters from the graph; pass `None` to allow unlimited snapping |

**Returns** `list[IsochroneResult]` - one structured result per time
limit, in the same order as `minutes`. Each implements `__geo_interface__`,
so GeoPandas and Shapely accept it directly; `.to_shapely()` converts it
(requires `shapely`) and `.to_geojson()` serializes it.

**Example**

```python
import geopandas as gpd

isos = graph.isochrone((48.137144, 11.575399), minutes=[5, 10, 15, 20])
frame = gpd.GeoDataFrame(
    {"minutes": [iso.minutes for iso in isos]},
    geometry=[iso.to_shapely() for iso in isos],
    crs="EPSG:4326",
)
```

---

## Routing

### `route`

```python
graph.route(
    origin: tuple[float, float],
    destination: tuple[float, float],
    max_snap_m: float | None = 100.0,
) -> RouteResult
```

Find the fastest route between two coordinates. Routes are exactly optimal.

The first `route()` call starts building a routing index (a contraction
hierarchy) in a background thread and answers with A\* meanwhile; once the
index is ready, routes take well under a millisecond. Call
`graph.prepare_routing()` to build it up front and wait for it, and
`graph.is_routing_prepared()` to check.

The network type (drive/walk/bike) is inherited from the `SpatialGraph`.
Coordinates snap to the nearest point on a road, so routes start and end
part-way along streets. Pass `max_snap_m` to reject routes whose origin or
destination is too far from the road network.

**Parameters**

| Parameter | Type | Description |
|-----------|------|-------------|
| `origin` | `tuple[float, float]` | `(lat, lon)` origin |
| `destination` | `tuple[float, float]` | `(lat, lon)` destination |
| `max_snap_m` | `float` or `None` | Maximum snap distance for both endpoints; defaults to `100.0`, pass `None` to allow unlimited snapping |

**Returns** `RouteResult` with properties:

| Property | Type | Description |
|----------|------|-------------|
| `distance_m` | `float` | Total route distance in meters |
| `duration_s` | `float` | Total travel time in seconds |
| `cumulative_times_s` | `list[float]` | Elapsed travel time at each waypoint |
| `origin_snap` | `SnapResult` | Snap diagnostics for the origin |
| `destination_snap` | `SnapResult` | Snap diagnostics for the destination |

**Example**

```python
route = graph.route((48.137144, 11.575399), (48.154560, 11.530840))

print(f"Distance: {route.distance_m:.0f} m")
print(f"Duration: {route.duration_s / 60:.1f} min")
print(f"Waypoints: {len(route.coordinates)}")
route_geojson = route.to_geojson()
```

`RouteResult` also implements `__geo_interface__` (a `LineString`) and
`.to_shapely()`.

---

### `travel_time_matrix`

```python
graph.travel_time_matrix(
    origins: list[tuple[float, float]],
    destinations: list[tuple[float, float]],
    max_snap_m: float | None = 100.0,
) -> TravelTimeMatrix
```

Fastest travel times from every origin to every destination, exactly as
`route` would find them. Large matrices build the routing index first, after
which a 1000 x 1000 matrix of Munich takes about 80 ms driving and 150 ms
walking. Points farther than `max_snap_m` from any road get `None` times
rather than failing the whole matrix.

**Returns** `TravelTimeMatrix` with properties:

| Property | Type | Description |
|----------|------|-------------|
| `durations_s` | tuple of tuples of `float` or `None` | `durations_s[i][j]`: seconds from origin `i` to destination `j`; `None` when unreachable or unsnapped |
| `distances_m` | tuple of tuples of `float` or `None` | Length in metres of each of those fastest routes |
| `origin_snaps` | list of `SnapResult` or `None` | Where each origin joined the network |
| `destination_snaps` | list of `SnapResult` or `None` | Where each destination joined the network |
| `shape` | `tuple[int, int]` | `(len(origins), len(destinations))` |

**Example**

```python
import numpy as np

homes = [(48.137, 11.575), (48.150, 11.560), (48.120, 11.600)]
clinics = [(48.140, 11.560), (48.130, 11.590)]
matrix = graph.travel_time_matrix(homes, clinics)

times = np.array(matrix.durations_s, dtype=float)  # None becomes nan
nearest_clinic_minutes = np.nanmin(times, axis=1) / 60
```

---

## Public transport

### `with_transit`

```python
graph.with_transit(
    gtfs: str | PathLike,
    date: str,
    start: str = "07:00",
    end: str = "09:00",
    wait_factor: float = 0.5,
    max_link_m: float = 300.0,
) -> SpatialGraph
```

A copy of a walking graph that can also ride public transport, from a GTFS
feed (a `.zip` or an unpacked directory; [Mobility Database](https://mobilitydatabase.org)
and [Transitland](https://www.transit.land) list feeds worldwide). Every
query on it (routes, matrices, isochrones, accessibility) may then take
transit.

The timetable between `start` and `end` on `date` is modelled by its
frequencies rather than individual departures:

- each sequence of stops a line serves becomes a chain of "on board" nodes,
  joined by the average running time in the window;
- boarding costs the expected wait, `wait_factor` times the headway at that
  stop (half the headway: someone arriving at a random time waits half the
  gap between departures);
- changing lines means getting off, walking (through the streets, or along
  the feed's `transfers.txt` links) and waiting again;
- each stop is linked to the nearest street within `max_link_m`.

Points still snap only to streets, and isochrones are shaped by the street
nodes reached, so islands of reachability appear around distant stops.

This is an approximation. It is closest to the *median* travel time over the
window that schedule-based tools such as r5 report, and it is most accurate
where service is frequent; with sparse service the real wait depends on when
one sets off. See the accuracy notes in the README of `benchmarks/transit`.

```python
walk = gw.SpatialGraph.from_pbf("munich.osm.pbf", "walk")
transit = walk.with_transit("mvg_gtfs.zip", date="2026-10-06", start="07:00", end="09:00")
print(transit.transit_summary)  # {'stops': 2696, 'patterns': 341, 'unlinked_stops': 125}

isochrones = transit.isochrone((48.137, 11.575), minutes=[15, 30])
jobs = transit.accessibility(homes, workplaces, minutes=[30, 45], weights=job_counts)
```

---

## Accessibility

### `accessibility`

```python
graph.accessibility(
    origins: list[tuple[float, float]],
    opportunities: list[tuple[float, float]],
    minutes: list[float],
    weights: list[float] | None = None,
    decay: str = "step",
    max_snap_m: float | None = 100.0,
) -> list[list[float] | None]
```

How much each origin can reach: the sum over opportunities (jobs, schools,
clinics, ...) of `weight x decay(travel time)`, one score per value in
`minutes`. Weights default to 1, so the default `"step"` decay counts the
opportunities reachable within each number of minutes.

| `decay` | Weight of an opportunity `t` minutes away |
|---------|--------------------------------------------|
| `"step"` | 1 within `minutes`, 0 beyond (cumulative opportunities) |
| `"linear"` | falls from 1 to 0 at `minutes` |
| `"exponential"` | halves every `minutes` |
| `"gaussian"` | `exp(-t^2 / (2 * minutes^2))` |

Origins too far from any road score `None`; opportunities that can't be
snapped or reached add nothing. Scores are computed origin by origin without
ever holding the full origins x opportunities table, so large problems fit
in memory: 10,000 homes against 10,000 weighted jobs at three cutoffs takes
about 3 s driving and 7 s walking on Munich (2 cores).

```python
jobs_within = graph.accessibility(homes, workplaces, minutes=[15, 30, 45], weights=job_counts)
gravity = graph.accessibility(homes, workplaces, minutes=[10], weights=job_counts, decay="exponential")
```

### `nearest_destinations`

```python
graph.nearest_destinations(
    origins: list[tuple[float, float]],
    destinations: list[tuple[float, float]],
    k: int = 1,
    max_snap_m: float | None = 100.0,
) -> list[list[tuple[int, float, float]]]
```

The `k` destinations each origin reaches fastest, nearest first, as
`(index into destinations, duration_s, distance_m)` tuples: fewer when fewer
are reachable, none for an origin too far from any road.

```python
nearest = graph.nearest_destinations(homes, clinics, k=1)
minutes_to_clinic = [row[0][1] / 60 if row else None for row in nearest]
```

---

## Reachability

### `reachable`

```python
graph.reachable(
    origin: tuple[float, float],
    minutes: float,
    max_snap_m: float | None = 100.0,
) -> ReachableGraph
```

Return a travel-time-labeled view of the graph reachable from an origin.
Inspection and GeoJSON export use the parent graph plus reachable-node labels,
so they do not copy the road network. Constrained routing and isochrones
materialize a bounded subgraph internally only when those methods are called.
Pass `max_snap_m` to reject origins that are too far from the graph.

**Example**

```python
import json

reachable = graph.reachable((48.137144, 11.575399), minutes=15)

nodes = reachable.nodes()
node_layer = json.loads(reachable.nodes_geojson())
edge_layer = json.loads(reachable.edges_geojson())
network_layer = json.loads(reachable.to_geojson())
route = reachable.route((48.137144, 11.575399), (48.142, 11.58))
isos = reachable.isochrone((48.137144, 11.575399), minutes=[5, 10, 15])
```

`ReachableGraph` methods:

| Method | Returns | Description |
|--------|---------|-------------|
| `node_count()` | `int` | Number of reachable nodes |
| `edge_count()` | `int` | Number of reachable directed edges |
| `nearest_node(lat, lon)` | `tuple` or `None` | Nearest node inside the reachable subgraph |
| `contains_node(node_id)` | `bool` | Whether an OSM node id is reachable |
| `travel_time_to_node_id(node_id)` | `float` or `None` | Travel time to an OSM node id |
| `nodes()` | `list[dict]` | Reachable nodes with `node_id`, `lat`, `lon`, `travel_time_s` |
| `nodes_geojson()` | `str` | Reachable nodes as GeoJSON points |
| `edges_geojson()` | `str` | Edges whose source and target are both reachable |
| `to_geojson()` | `str` | Reachable nodes and edges in one FeatureCollection |
| `route(origin, destination, max_snap_m=100.0)` | `RouteResult` | Route constrained to the reachable subgraph |
| `isochrone(origin, minutes, max_snap_m=100.0)` | `list[IsochroneResult]` | Isochrones constrained to the reachable subgraph |

---

## Network-Time Prisms

### `prism`

```python
graph.prism(
    origin: tuple[float, float],
    destination: tuple[float, float],
    max_minutes: float,
    stop_minutes: float = 0.0,
    buffer_minutes: float = 0.0,
    max_snap_m: float | None = 100.0,
) -> PrismGraph
```

Return a graph view of possible stops between an origin and a destination within
a fixed time window. A node is inside the prism when:

```text
origin -> node -> destination + stop_minutes + buffer_minutes <= max_minutes
```

This is useful for "what can I do on the way?" analysis without turning the
library into a trip-planning or stop-order optimizer.
Pass `max_snap_m` to reject origins or destinations that are too far from the
graph.

Like `ReachableGraph`, `PrismGraph` is a lightweight view. It stores the parent
graph plus inbound, outbound, and slack labels; constrained routes or isochrones
materialize a bounded subgraph only when needed.

**Example**

```python
prism = graph.prism(
    origin=(48.137144, 11.575399),
    destination=(48.154560, 11.530840),
    max_minutes=45,
    stop_minutes=10,
    buffer_minutes=5,
)

nodes = prism.nodes()
network_layer = prism.to_geojson()
slack = prism.slack_polygon(min_slack_s=5 * 60)
route = prism.route((48.137144, 11.575399), (48.142, 11.56))
```

`PrismGraph` methods:

| Method | Returns | Description |
|--------|---------|-------------|
| `node_count()` | `int` | Number of nodes inside the prism |
| `edge_count()` | `int` | Number of directed edges inside the prism |
| `nearest_node(lat, lon)` | `tuple` or `None` | Nearest node inside the prism graph |
| `contains_node(node_id)` | `bool` | Whether an OSM node id is inside the prism |
| `slack_at_node_id(node_id)` | `float` or `None` | Remaining slack for an OSM node id |
| `nodes()` | `list[dict]` | Nodes with `node_id`, `lat`, `lon`, `inbound_time_s`, `outbound_time_s`, `slack_s` |
| `nodes_geojson()` | `str` | Prism nodes as GeoJSON points |
| `edges_geojson()` | `str` | Edges whose source and target are inside the prism |
| `to_geojson()` | `str` | Prism nodes and edges in one FeatureCollection |
| `slack_polygon(min_slack_s)` | `str` or `None` | Polygon enclosing nodes with at least the requested slack |
| `route(origin, destination, max_snap_m=100.0)` | `RouteResult` | Route constrained to the prism graph |
| `isochrone(origin, minutes, max_snap_m=100.0)` | `list[IsochroneResult]` | Isochrones constrained to the prism graph |

---

## Points of interest

### `fetch_pois`

```python
graph.fetch_pois(isochrone: IsochroneResult | str) -> PoiCollection
```

Fetch OSM points of interest that fall within a given isochrone polygon.

Queries Overpass for the polygon's bounding box, using the XML cache when
available, then filters returned nodes to those geometrically inside the
polygon.

**Parameters**

| Parameter | Type | Description |
|-----------|------|-------------|
| `isochrone` | `IsochroneResult` or `str` | An isochrone result or a GeoJSON geometry string |

**Returns** `PoiCollection` with structured `Poi` objects. Call `.to_geojson()`
for a GeoJSON `FeatureCollection`.

**Example**

```python
isos = graph.isochrone((48.137144, 11.575399), minutes=[10])
pois = graph.fetch_pois(isos[0])

restaurants = [
    poi.tags.get("name", "?")
    for poi in pois.pois
    if poi.tags.get("amenity") == "restaurant"
]
print(f"Found {len(restaurants)} restaurants within 10 minutes")
```

---

## Visualisation

### `nodes_geojson`

```python
graph.nodes_geojson() -> str
```

Return all graph nodes as a GeoJSON `FeatureCollection` of `Point` features.

Each feature has properties: `id` (OSM node id), `lat`, `lon`.

Useful for visualising the network in mapping tools.

**Example**

```python
import folium, json

nodes = json.loads(graph.nodes_geojson())
m = folium.Map(location=[48.137, 11.575], zoom_start=13)
for feature in nodes["features"]:
    lat = feature["properties"]["lat"]
    lon = feature["properties"]["lon"]
    folium.CircleMarker([lat, lon], radius=2).add_to(m)
```

---

### `edges_geojson`

```python
graph.edges_geojson() -> str
```

Return all graph edges as a GeoJSON `FeatureCollection` of `LineString` features.

Each feature has properties:

| Property | Type | Description |
|----------|------|-------------|
| `highway` | `str` | OSM highway tag value (e.g. `"residential"`) |
| `length_m` | `float` | Edge length in meters |
| `speed_kph` | `float` | Assigned speed in km/h |
| `drive_time_s` | `float` | Drive travel time in seconds |
| `walk_time_s` | `float` | Walk travel time in seconds |
| `bike_time_s` | `float` | Bike travel time in seconds |

**Example**

```python
import folium, json

edges = json.loads(graph.edges_geojson())
m = folium.Map(location=[48.137, 11.575], zoom_start=13)
folium.GeoJson(edges, style_function=lambda _: {"color": "#3388ff", "weight": 1}).add_to(m)
```

