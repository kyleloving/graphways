# Quickstart

## The core pattern

Graphways is centered on a reusable `SpatialGraph`. Build the graph once for an area, then run reachability, isochrone, route, and POI queries against that graph.

```python
import graphways as gw

origin = (48.137144, 11.575399)
destination = (48.154560, 11.530840)

graph = gw.SpatialGraph.from_place(
    "Marienplatz, Munich, Germany",
    network="drive",
    max_dist=10_000,
)
print(graph)

isos = graph.isochrone(origin, minutes=[5, 10, 15, 20])
route = graph.route(origin, destination)
pois = graph.fetch_pois(isos[-1])

print(route.distance_m, route.duration_s)
print(route.origin_snap.distance_m)
print(pois.count)
```

Use `SpatialGraph.from_pbf(path, network="walk")` for local offline OSM PBF workflows, or `SpatialGraph.from_osm(xml, network="walk")` when you already have OSM XML.

Points snap to the closest point on any road, so routes and isochrones start
part-way along a street rather than at the nearest intersection.

For repeated work over the same area, save the graph once and load it next
time, and use a matrix when you need many trips at once:

```python
graph.save("munich-drive.graph")             # includes the routing index
graph = gw.SpatialGraph.load("munich-drive.graph")

stops = [(48.140, 11.560), (48.130, 11.590), (48.150, 11.580)]
matrix = graph.travel_time_matrix([origin], stops)
print(matrix.durations_s[0])                 # seconds to each stop
```

---
## Working with structured results

Routes, snap diagnostics, and isochrones are structured Python objects. Routes
and isochrones implement `__geo_interface__`, so `shapely.geometry.shape(iso)`
and GeoPandas accept them directly. Export GeoJSON explicitly when you need to
pass geometry to other mapping tools:

```python
import json

# Isochrone geometry
iso = json.loads(isos[0].to_geojson())
print(iso["type"])        # "MultiPolygon": parts, each with holes
print(iso["coordinates"]) # [[[[lon, lat], ...], ...], ...]

# Route metrics and feature export
print(f"Distance: {route.distance_m:.0f} m")
print(f"Duration: {route.duration_s:.0f} s")
print(f"Waypoints: {len(route.cumulative_times_s)}")
print(f"Origin snap: {route.origin_snap.distance_m:.1f} m")

route_data = json.loads(route.to_geojson())
props = route_data["properties"]

# POI FeatureCollection
pois_data = json.loads(pois.to_geojson())
for feature in pois_data["features"]:
    tags = feature["properties"]
    name = tags.get("name", "unnamed")
    kind = tags.get("amenity") or tags.get("shop") or "?"
    print(f"{name} ({kind})")
```

---

## Choosing a network type

| Value | Includes |
|-------|----------|
| `"drive"` | Public roads accessible to private cars; excludes service roads, driveways |
| `"drive_service"` | Like `Drive` but includes service roads |
| `"walk"` | Footways, pedestrian paths, and shared roads where walking is permitted |
| `"bike"` | Cycleways and roads open to bicycles |
| `"all"` | All highway types except private access |
| `"all_private"` | All highway types including private access |

---

## Caching

graphways caches Overpass XML responses so repeated `from_place(...)` calls can
skip the network:

```
Overpass API (network) -> disk XML -> in-memory XML
```

Disk cache entries persist across process restarts. The in-memory XML cache only
lasts for the current process. Built graphs are not hidden in a global cache;
hold onto the returned `SpatialGraph` and reuse it directly.

```python
print(gw.cache_dir())  # shows the disk cache location
gw.clear_cache()       # wipe XML caches
```

Set `GRAPHWAYS_CACHE_DIR` to override the default cache location.

## Network services

`SpatialGraph.from_place(...)`, `gw.geocode(...)`, and POI fetching use public
OpenStreetMap services by default. Graphways sends a descriptive User-Agent,
rate-limits Nominatim geocoding requests, and retries transient `429` / `5xx`
responses.

Set these environment variables when you need local mirrors, custom endpoints,
or a project-specific contact string:

```bash
GRAPHWAYS_OVERPASS_URL=https://overpass-api.de/api/interpreter
GRAPHWAYS_NOMINATIM_URL=https://nominatim.openstreetmap.org/search
GRAPHWAYS_USER_AGENT="your-app/1.0 contact@example.com"
```

Use `SpatialGraph.from_pbf(...)` for offline workflows that should not touch
network services.

