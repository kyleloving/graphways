"""Head-to-head latency: graphways vs OSRM vs Valhalla, all in-process.

OSRM and Valhalla run through their official Python packages
(``osrm-bindings`` and ``pyvalhalla``), so no HTTP or JSON-over-the-wire
overhead is measured for any engine. Everything runs on the same extract and
the same query points.

Fairness rules:

* Query points are OSM road nodes that are mutually reachable on the
  graphways graph, so no engine is penalised for exhausting the network on an
  unreachable pair (Valhalla's CostMatrix is especially sensitive to that).
* Setup (graph build, contraction, tiles) is reported separately and never
  included in query latency.
* Results are latencies, not like-for-like outputs: each engine has its own
  travel-time model (OSRM's car profile adds turn and signal penalties,
  Valhalla's costing is dynamic, graphways uses edge speeds plus turn costs).
  The script prints how closely the engines' durations agree.

Setup (Python >= 3.12 for the engine wheels)::

    pip install graphways osrm-bindings pyvalhalla numpy
    python -m osrm extract -p <site-packages>/osrm/share/profiles/car.lua munich.osm.pbf
    # CH and MLD need separate extracts (partitioning renumbers the graph):
    python -m osrm contract ch/munich.osrm
    python -m osrm partition mld/munich.osrm && python -m osrm customize mld/munich.osrm
    # Valhalla: build a config with tile_dir / tile_extract set, then
    #   valhalla_build_tiles -c valhalla.json munich.osm.pbf
    #   python <site-packages>/valhalla/valhalla_build_extract.py -c valhalla.json

Run::

    python benchmarks/engines/head_to_head.py --pbf munich.osm.pbf \\
        --osrm-ch-car ch_car/munich.osrm --osrm-mld-car mld_car/munich.osrm \\
        --osrm-ch-foot ch_foot/munich.osrm --osrm-mld-foot mld_foot/munich.osrm \\
        --valhalla valhalla.json
"""

from __future__ import annotations

import argparse
import json
import random
import statistics
import time

import graphways as gw


def timed(fn):
    start = time.perf_counter()
    out = fn()
    return (time.perf_counter() - start) * 1e3, out


def median_ms(fn, items):
    samples = []
    for item in items:
        start = time.perf_counter()
        fn(item)
        samples.append(time.perf_counter() - start)
    return statistics.median(samples) * 1e3


def connected_points(graph: gw.SpatialGraph, count: int, seed: int) -> list[tuple[float, float]]:
    """Road nodes that can all reach each other on ``graph``."""
    rng = random.Random(seed)
    nodes = json.loads(graph.nodes_geojson())["features"]
    pool = [(f["properties"]["lat"], f["properties"]["lon"]) for f in rng.sample(nodes, min(len(nodes), count * 3))]
    hub = pool[0]
    out_row = graph.travel_time_matrix([hub], pool).durations_s[0]
    in_col = [row[0] for row in graph.travel_time_matrix(pool, [hub]).durations_s]
    keep = [p for p, a, b in zip(pool, out_row, in_col) if a is not None and b is not None]
    return keep[:count]


def report(rows, key, engine, ms, note=""):
    rows.append((key, engine, ms, note))
    print(f"{key:<30} {engine:<18} {ms:>11.3f} ms  {note}", flush=True)


def agreement(a, b):
    """Median |a - b| / a over cells both engines answered."""
    ratios = [abs(x - y) / x for x, y in zip(a, b) if x and y]
    return statistics.median(ratios) if ratios else float("nan")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pbf", required=True)
    for algorithm in ("ch", "mld"):
        for profile in ("car", "foot"):
            ap.add_argument(f"--osrm-{algorithm}-{profile}",
                            help=f"munich.osrm prepared for {algorithm.upper()} with the {profile} profile")
    ap.add_argument("--valhalla", help="path to valhalla.json")
    ap.add_argument("--routes", type=int, default=200)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rows: list = []
    modes = [("drive", "car", "auto", [5, 10, 15]), ("walk", "foot", "pedestrian", [10, 20, 30])]
    for net, osrm_profile, valhalla_costing, limits in modes:
        ms, graph = timed(lambda: gw.SpatialGraph.from_pbf(args.pbf, net))
        report(rows, f"setup {net}", "graphways build", ms)
        ms, _ = timed(graph.prepare_routing)
        report(rows, f"setup {net}", "graphways prepare", ms)

        points = connected_points(graph, 1000, args.seed)
        rng = random.Random(args.seed)
        pairs = [(rng.choice(points), rng.choice(points)) for _ in range(args.routes)]
        small, large = points[:100], points[:1000]
        graph.route(*pairs[0])

        report(rows, f"route {net}", "graphways", median_ms(lambda p: graph.route(*p), pairs))
        ms, gw_small = timed(lambda: graph.travel_time_matrix(small, small))
        report(rows, f"matrix 100x100 {net}", "graphways", ms)
        ms, _ = timed(lambda: graph.travel_time_matrix(large, large))
        report(rows, f"matrix 1000x1000 {net}", "graphways", ms)
        report(rows, f"isochrone {net} {limits}", "graphways",
               median_ms(lambda o: graph.isochrone(o, limits), points[:20]))
        gw_cells = [c for row in gw_small.durations_s for c in row]

        for algorithm in ("CH", "MLD"):
            osrm_path = getattr(args, f"osrm_{algorithm.lower()}_{osrm_profile}")
            if not osrm_path:
                continue
            import osrm

            engine = osrm.OSRM(storage_config=osrm_path, algorithm=algorithm, use_shared_memory=False,
                               max_locations_distance_table=1_000_000)

            def route(p, engine=engine):
                (a, b) = p
                return engine.Route(osrm.RouteParameters(
                    coordinates=[(a[1], a[0]), (b[1], b[0])], overview="full", geometries="geojson"))

            def table(pts, engine=engine):
                return engine.Table(osrm.TableParameters(
                    coordinates=[(p[1], p[0]) for p in pts], annotations=["duration"]))

            route(pairs[0])
            name = f"OSRM {algorithm}"
            report(rows, f"route {net}", name, median_ms(route, pairs))
            ms, res = timed(lambda: table(small))
            cells = [c for row in res["durations"] for c in row]
            report(rows, f"matrix 100x100 {net}", name, ms,
                   f"durations differ from graphways by {agreement(gw_cells, cells):.0%} (median)")
            report(rows, f"matrix 1000x1000 {net}", name, timed(lambda: table(large))[0])

        if args.valhalla:
            from valhalla import Actor

            actor = Actor(args.valhalla)

            def loc(p):
                return {"lat": p[0], "lon": p[1]}

            def vroute(p):
                return actor.route(json.dumps({"locations": [loc(p[0]), loc(p[1])],
                                               "costing": valhalla_costing, "directions_type": "none"}))

            def vmatrix(pts):
                locs = [loc(p) for p in pts]
                return json.loads(actor.matrix(json.dumps(
                    {"sources": locs, "targets": locs, "costing": valhalla_costing})))

            def viso(o):
                return actor.isochrone(json.dumps({"locations": [loc(o)], "costing": valhalla_costing,
                                                   "contours": [{"time": t} for t in limits],
                                                   "polygons": True}))

            vroute(pairs[0])
            report(rows, f"route {net}", "Valhalla", median_ms(vroute, pairs[:100]), "no turn-by-turn")
            ms, res = timed(lambda: vmatrix(small))
            cells = [c.get("time") for row in res["sources_to_targets"] for c in row]
            report(rows, f"matrix 100x100 {net}", "Valhalla", ms,
                   f"durations differ from graphways by {agreement(gw_cells, cells):.0%} (median)")
            report(rows, f"isochrone {net} {limits}", "Valhalla", median_ms(viso, points[:20]))

    print(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
