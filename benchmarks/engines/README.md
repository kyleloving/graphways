# Engine Benchmarks

`head_to_head.py` compares graphways with OSRM (both of its algorithms) and
Valhalla on the same extract and the same query points. All three run
in-process through their official Python packages (`osrm-bindings`,
`pyvalhalla`), so no engine pays for HTTP or JSON transport.

## Setup

The engine wheels need Python 3.12 or newer.

```bash
pip install graphways osrm-bindings pyvalhalla numpy
PROFILES=$(python -c "import osrm, os; print(os.path.dirname(osrm.__file__))")/share/profiles

# OSRM: CH and MLD need separate extracts (partitioning renumbers the graph).
for algo in ch mld; do for p in car foot; do
  mkdir -p ${algo}_$p && cp munich.osm.pbf ${algo}_$p/
  python -m osrm extract -p $PROFILES/$p.lua ${algo}_$p/munich.osm.pbf
done; done
python -m osrm contract ch_car/munich.osrm && python -m osrm contract ch_foot/munich.osrm
for p in car foot; do python -m osrm partition mld_$p/munich.osrm && python -m osrm customize mld_$p/munich.osrm; done

# Valhalla: write a config whose mjolnir.tile_dir / tile_extract point at a
# working directory (valhalla.get_config()), then build tiles and the extract.
python -m valhalla valhalla_build_tiles -c valhalla.json munich.osm.pbf
python <site-packages>/valhalla/valhalla_build_extract.py -c valhalla.json
```

## Run

```bash
python benchmarks/engines/head_to_head.py --pbf munich.osm.pbf \
  --osrm-ch-car ch_car/munich.osrm --osrm-ch-foot ch_foot/munich.osrm \
  --osrm-mld-car mld_car/munich.osrm --osrm-mld-foot mld_foot/munich.osrm \
  --valhalla valhalla.json
```

Every argument but `--pbf` is optional; leave out an engine to skip it.

## What the numbers mean

Query points are road nodes that are mutually reachable on the graphways
graph, so no engine spends time exhausting the network on an unreachable
pair. Setup is reported separately from query latency.

These are latencies, **not** like-for-like answers. Each engine has its own
travel-time model, and the script prints how far each engine's matrix
durations are from graphways':

- OSRM's car profile uses its own road speeds (it drives 80% of the posted
  limit, for example). Both engines price turns and traffic signals the same
  way, yet OSRM's driving times still differ from graphways' by a median of
  25% on Munich; walking times agree to about 2%.
- Valhalla prices every query on the fly (dynamic costing, no
  precomputation), which is what makes it flexible and why its single
  queries are slower. Its durations agree with graphways' to within about
  3% (driving) and 1% (walking).
- OSRM and Valhalla also do things graphways does not: turn-by-turn
  instructions (disabled for Valhalla here), map matching, trip planning.

## Sample results

Munich extract (bundled), 2-core cloud VM, October 2026. Medians for single
queries, wall time for matrices.

| | graphways | OSRM CH | OSRM MLD | Valhalla |
|---|---|---|---|---|
| Drive route | 0.18 ms | 0.64 ms | 0.85 ms | 28 ms |
| Drive matrix 100 x 100 | 13 ms | 37 ms | 103 ms | 10 s* |
| Drive matrix 1000 x 1000 | 74 ms | 590 ms | 2.3 s | - |
| Drive isochrone (5/10/15 min) | 11 ms | - | - | 210 ms |
| Walk route | 0.33 ms | 1.3 ms | 3.3 ms | 33 ms |
| Walk matrix 1000 x 1000 | 126 ms | 1.7 s | 10 s | - |
| Walk isochrone (10/20/30 min) | 6 ms | - | - | 33 ms |
| Drive setup | 3.0 s | 53 s | 18 s | 19 s (tiles, both modes) |
| Walk setup | 9 s | 248 s | 17 s | (shared) |

\* Valhalla's CostMatrix with default `thor` settings; it was not tuned
further and is likely not representative of a production deployment.

Graphways' drive numbers include turn costs and signal delays (switching
them off makes routes and matrices about twice as fast and preparation ten
times faster). The graphways matrix uses all cores (rayon), while OSRM
answers each table request on one thread; on one thread graphways took about
twice as long. Treat the comparison as indicative.
