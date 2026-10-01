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

One Dijkstra pass is run from the nearest graph node; one triangulated contour
polygon is computed per time limit.

**Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `origin` | `tuple[float, float]` | - | `(lat, lon)` origin |
| `minutes` | `list[float]` | - | Travel-time thresholds in minutes |
| `max_snap_m` | `float` or `None` | `100.0` | Reject the query if the origin snaps farther than this many meters from the graph; pass `None` to allow unlimited snapping |

**Returns** `list[IsochroneResult]` - one structured polygon result per time
limit, in the same order as `minutes`. Use `.to_geojson()` for mapping tools.

**Example**

```python
isos = graph.isochrone((48.137144, 11.575399), minutes=[5, 10, 15, 20])
first_geojson = isos[0].to_geojson()
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
Coordinates snap to the nearest graph node. Pass `max_snap_m` to reject routes
whose origin or destination is too far from the road network.

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

