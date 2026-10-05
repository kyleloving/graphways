# Benchmarks

This directory keeps benchmark and external comparison artifacts.

## Local PBF Pipeline

`benchmark.rs` is a Rust harness for the local PBF pipeline, run through Cargo:

```bash
cargo bench --bench pipeline                          # bundled Munich extract, driving
NETWORK=walk VERIFY=1 cargo bench --bench pipeline
cargo bench --bench pipeline -- path/to/extract.osm.pbf
```

It times one-shot setup (PBF load, routing preparation, save and load)
separately from steady-state queries (reachability, isochrones, prisms,
routes before and after preparation, travel-time matrices). Query points are
drawn with a fixed seed, so runs are comparable.

Useful environment variables:

- `NETWORK=drive|walk|bike`
- `ITERS=20` (queries per measured stage)
- `BUDGET=600` (reachability budget in seconds)
- `VERIFY=1` (also check every route against plain Dijkstra)

## External Comparison

```powershell
python benchmarks/comparison.py
python benchmarks/comparison.py --skip-r5py
python benchmarks/comparison.py --pbf C:\path\to\extract.osm.pbf
```

This runs three sections:

- a steady-state graphways vs NetworkX comparison on pre-warmed OSM graphs
- a graphways-only split between cached XML graph construction and repeated query cost
- an optional r5py comparison when `--pbf` is supplied and r5py is installed

The chart uses the headline comparison and updates `benchmarks/performance.png`
when plotting dependencies are installed.

The r5py section also requires a compatible Java JDK. If r5py imports but the
JVM fails to start, check `java -version` and `JAVA_HOME`; on Windows, a
conda/mamba environment from `conda-forge` is usually the least fussy setup.

## Routing Engine Comparison

```powershell
python benchmarks/engines/engines.py --pbf C:\path\to\munich.osm.pbf
```

This optional harness compares steady-state route latency against OSRM and
Valhalla, plus isochrone latency against Valhalla. Engine setup is documented in
`benchmarks/engines/README.md`; both engines require preprocessing before they
can serve requests.
