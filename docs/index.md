# graphways

**Fast isochrones, routing, and POI lookups from OpenStreetMap -- written in Rust, callable from Python.**

[![PyPI](https://img.shields.io/pypi/v/graphways)](https://pypi.org/project/graphways/)
[![Crates.io](https://img.shields.io/crates/v/graphways)](https://crates.io/crates/graphways)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## What it does

graphways queries OpenStreetMap, builds a road-network graph, and gives you:

- **Isochrones** -- multipolygons covering everything reachable within a time limit, with holes for unreachable pockets
- **Point-to-point routing** -- exact routes with per-waypoint cumulative travel times, sub-millisecond once prepared
- **Travel-time matrices** -- many origins to many destinations in one call
- **POI fetching** -- amenities, shops, and other features within any isochrone
- **Graph introspection** -- inspect nodes, edges, and the network structure directly

Routes, isochrones, snap diagnostics, and POIs return structured Python objects.
Call `.to_geojson()` when you need serialized GeoJSON for maps or files.

---

## 30-second start

```python
import graphways as gw

# Build the graph once; reuse this object for repeated local queries.
graph = gw.SpatialGraph.from_place(
    "Marienplatz, Munich, Germany",
    network="drive",
    max_dist=10_000,
)

isos = graph.isochrone((48.137144, 11.575399), minutes=[5, 10, 15, 20])

route = graph.route((48.137144, 11.575399), (48.154560, 11.530840))
print(route.distance_m, route.duration_s)

pois = graph.fetch_pois(isos[0])
print(pois.count)
```

---

## Features at a glance

| Feature | Detail |
|---------|--------|
| Graph construction | Parses OSM XML or local OSM PBF into a reusable `SpatialGraph` |
| Simplification | Collapses linear chains, deduplicates parallel edges, and preserves edge geometry |
| Snapping | Points join the network at the closest point on any road (R-tree over road segments) |
| Speed profiles | Configurable walking, cycling and per-road-class driving speeds |
| Turn restrictions | OSM `type=restriction` relations are obeyed when driving |
| Isochrones | Bounded graph search plus triangulated travel-time contours, as multipolygons with holes |
| Routing | Contraction hierarchies once prepared, A* before; both exact |
| Matrices | Many-to-many travel times via hierarchy buckets |
| Persistence | Save a prepared graph and load it back in a fraction of the build time |
| Network types | Drive, DriveService, Walk, Bike, All, AllPrivate |
| Caching | Overpass XML cache: disk XML -> in-memory XML |
| Python bindings | Structured result objects with `__geo_interface__` and explicit GeoJSON export; the GIL is released during queries |

---

## Performance

Graphways is designed for repeated local queries over a reusable `SpatialGraph`.
The benchmark suite reports graph construction separately from steady-state
route, reachability, and isochrone queries:

```bash
python benchmarks/comparison.py
python benchmarks/engines/engines.py --pbf C:\path\to\extract.osm.pbf
cargo bench --bench pipeline              # Rust pipeline on the bundled Munich extract
```

Treat benchmark numbers as workload-specific. They depend on graph size,
network profile, machine, cache state, and whether comparisons include
server-based routing engines such as OSRM or Valhalla.
