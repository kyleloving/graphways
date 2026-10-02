# Graphways

[![PyPI Version][pypi-version-badge]][pypi-link]
[![Crates.io Version][crates-version-badge]][crates-link]
[![Python Versions][python-versions-badge]][pypi-link]
[![CI][ci-badge]][ci-link]
[![Documentation][docs-badge]][docs-link]
[![License: MIT][license-badge]][license-link]

**Graphways** is a Rust-powered Python library for fast local reachability,
routing, and isochrone analysis on OpenStreetMap road networks. It builds a
reusable `SpatialGraph` once, then runs repeated network-aware queries directly
in process without deploying a routing server.

Graphways is designed for workflows that need the street network as a practical
analysis primitive: accessibility studies, urban analytics, site selection,
agent-based simulations, notebook exploration, and application backends that
need low-latency local graph operations.

![Graphways demo](docs/assets/graphways-demo.gif)

## Installation

Install the Python package from PyPI:

```bash
pip install graphways
```

Or add the Rust crate to `Cargo.toml`:

```toml
[dependencies]
graphways = "0.5.0"
```

To build the Python extension from source, install Rust and maturin, then run:

```bash
maturin develop
```

## Usage

```python
import graphways as gw

origin = (38.9097, -77.0432)
destination = (38.8977, -77.0365)

graph = gw.SpatialGraph.from_place(
    "Washington, DC",
    network="walk",
    max_dist=10_000,
)

isochrones = graph.isochrone(origin, minutes=[10, 20, 30])
route = graph.route(origin, destination, max_snap_m=100)
reachable = graph.reachable(origin, minutes=15, max_snap_m=100)

print(route.duration_s, route.distance_m)
route_geojson = route.to_geojson()
iso_geojson = isochrones[0].to_geojson()
```

`SpatialGraph` is the central object. Reachability and network-time prism
queries return lightweight graph views over the parent graph, so inspection and
GeoJSON export do not copy the full road network.

```python
reachable.nodes()
reachable.edges_geojson()
reachable.route(origin, destination)

prism = graph.prism(
    origin=origin,
    destination=destination,
    max_minutes=45,
    stop_minutes=10,
    buffer_minutes=5,
)

possible_stops = prism.nodes()
slack_polygon = prism.slack_polygon(min_slack_s=5 * 60)
```

Routes, snap diagnostics, and isochrones return structured Python objects.
They implement `__geo_interface__`, so GeoPandas and Shapely accept them
directly; call `.to_geojson()` when you need serialized GeoJSON.

Travel-time matrices, saved graphs, and speed profiles cover the heavier
workflows:

```python
# Seconds from every origin to every destination (None where unreachable).
matrix = graph.travel_time_matrix(homes, clinics)

# Save once (routing index included), then load in a fraction of the time.
graph.save("dc-walk.graph")
graph = gw.SpatialGraph.load("dc-walk.graph")

# Slower walking for an older-adult accessibility study.
slow = gw.SpatialGraph.from_pbf("dc.osm.pbf", "walk", walk_speed_kph=3.5)
```

## Features

- Build reusable walking, biking, driving, and custom-access OSM road graphs.
- Load from Overpass XML, existing OSM XML strings, or local OSM PBF files;
  save prepared graphs and load them back in well under a second.
- Tune walking, cycling, and per-road-class driving speeds.
- Snap coordinates to the nearest point on any road, not just the nearest
  intersection.
- Respect turn restrictions (no left turn, only straight on, ...) when driving,
  and price turns and traffic signals the way OSRM's car profile does.
- Compute reachability over the road network from a single origin.
- Generate isochrones as multipolygons, with holes for unreachable areas.
- Route point-to-point with distance, duration, geometry, and cumulative
  times: exact, and well under a millisecond once routing is prepared
  (contraction hierarchies).
- Compute many-to-many travel-time and distance matrices (a million cells
  in about a tenth of a second on a city graph).
- Score accessibility (opportunities within reach, gravity-style decay) and
  find each origin's nearest destinations, without holding the full table
  in memory.
- Build network-time prisms for "what can I visit between A and B?" analysis.
- Export nodes, edges, routes, POIs, and isochrones as GeoJSON.

## Documentation

Start with the [quickstart][docs-quickstart], then see the
[Python graph API][docs-python-graph] and [Rust API notes][docs-rust-api].

The examples directory and documentation show common workflows:

- building graphs from places, XML, and PBF files
- computing walking and driving isochrones
- routing between coordinates
- querying reachable nodes and graph views
- working with POIs and GeoJSON output

## Network Services

`SpatialGraph.from_place(...)`, `gw.geocode(...)`, and POI fetching use public
OpenStreetMap services by default. Graphways sends a descriptive User-Agent,
rate-limits Nominatim geocoding requests, and retries transient `429` / `5xx`
responses.

For production workloads, local mirrors, or stricter service policies, configure
the service layer with environment variables:

```bash
GRAPHWAYS_OVERPASS_URL=https://overpass-api.de/api/interpreter
GRAPHWAYS_NOMINATIM_URL=https://nominatim.openstreetmap.org/search
GRAPHWAYS_USER_AGENT="your-app/1.0 contact@example.com"
GRAPHWAYS_CACHE_DIR=/path/to/graphways-cache
```

Use `SpatialGraph.from_pbf(...)` when you need fully offline graph construction.
Rust users who only load local files can drop the HTTP stack entirely with
`graphways = { version = "0.5", default-features = false }`.

## Performance

Graphways is optimized for repeated local queries over the same area. Graph
construction is paid once; subsequent reachability, routing, and isochrone
queries reuse the in-memory graph and spatial index.

The benchmark suite compares steady-state graphways queries against NetworkX /
OSMnx baselines and includes staged timings for graph construction versus
per-query work:

```bash
python benchmarks/comparison.py
cargo bench --bench pipeline              # Rust pipeline on the bundled Munich extract
```

Current benchmarks are intentionally kept in `benchmarks/` rather than treated
as a universal claim. Performance depends on graph size, network profile,
machine, cache state, and output geometry settings.

## License

Graphways is open source and licensed under the MIT license. OpenStreetMap data
is licensed separately; when using OSM-derived outputs, follow the
[OpenStreetMap attribution guidelines][osm-copyright].

<!-- badges -->

[ci-badge]: https://github.com/kyleloving/graphways/actions/workflows/ci.yml/badge.svg
[ci-link]: https://github.com/kyleloving/graphways/actions/workflows/ci.yml
[crates-link]: https://crates.io/crates/graphways
[crates-version-badge]: https://img.shields.io/crates/v/graphways
[docs-badge]: https://github.com/kyleloving/graphways/actions/workflows/docs.yml/badge.svg
[docs-link]: https://kyleloving.github.io/graphways/
[license-badge]: https://img.shields.io/badge/license-MIT-green
[license-link]: https://github.com/kyleloving/graphways/blob/main/LICENSE
[pypi-link]: https://pypi.org/project/graphways/
[pypi-version-badge]: https://img.shields.io/pypi/v/graphways
[python-versions-badge]: https://img.shields.io/pypi/pyversions/graphways

<!-- links -->

[docs-python-graph]: https://kyleloving.github.io/graphways/python-graph/
[docs-quickstart]: https://kyleloving.github.io/graphways/quickstart/
[docs-rust-api]: https://kyleloving.github.io/graphways/rust-api/
[osm-copyright]: https://www.openstreetmap.org/copyright
