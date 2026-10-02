"""Check graphways' frequency-based transit model against r5's timetable
routing, on the same street network, feed, points and time window.

r5 (through r5py) routes every departure minute in the window and reports
percentiles of the travel time; graphways' expected-wait model is compared
with the median.

Usage::

    pip install graphways r5py geopandas
    python benchmarks/transit/validate.py --pbf munich.osm.pbf --gtfs mvg.zip \\
        --date 2026-10-06 --start 07:00 --end 09:00 --points 300 \\
        --bbox 48.10,11.50,48.18,11.62 [--max-memory 5G]
"""

from __future__ import annotations

import argparse
import datetime as dt
import gc
import random
import sys
import time

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pbf", required=True)
    ap.add_argument("--gtfs", required=True)
    ap.add_argument("--date", required=True, help="YYYY-MM-DD")
    ap.add_argument("--start", default="07:00")
    ap.add_argument("--end", default="09:00")
    ap.add_argument("--points", type=int, default=300)
    ap.add_argument("--bbox", default="48.10,11.50,48.18,11.62", help="south,west,north,east")
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--max-memory", default=None, help="JVM heap for r5, e.g. 5G")
    args = ap.parse_args()

    south, west, north, east = map(float, args.bbox.split(","))
    rng = random.Random(args.seed)
    points = [(south + rng.random() * (north - south), west + rng.random() * (east - west))
              for _ in range(args.points)]

    # --- graphways -----------------------------------------------------------
    import graphways as gw

    t = time.perf_counter()
    walk = gw.SpatialGraph.from_pbf(args.pbf, "walk")
    transit = walk.with_transit(args.gtfs, date=args.date, start=args.start, end=args.end)
    built = time.perf_counter() - t
    t = time.perf_counter()
    ours = np.array(transit.travel_time_matrix(points, points).durations_s, dtype=float) / 60
    queried = time.perf_counter() - t
    print(f"graphways: {transit.transit_summary}, build {built:.1f} s, "
          f"{args.points}x{args.points} matrix {queried:.1f} s (incl. routing preparation)")
    del walk, transit
    gc.collect()

    # --- r5 --------------------------------------------------------------------
    if args.max_memory:
        sys.argv += ["--max-memory", args.max_memory]
    import geopandas as gpd
    import r5py
    from shapely.geometry import Point

    sites = gpd.GeoDataFrame({"id": range(len(points))},
                             geometry=[Point(lon, lat) for lat, lon in points], crs="EPSG:4326")
    t = time.perf_counter()
    network = r5py.TransportNetwork(args.pbf, [args.gtfs])
    hour, minute = map(int, args.start.split(":"))
    start = dt.datetime.fromisoformat(args.date).replace(hour=hour, minute=minute)
    h2, m2 = map(int, args.end.split(":"))
    window = dt.timedelta(hours=h2, minutes=m2) - dt.timedelta(hours=hour, minutes=minute)
    table = r5py.TravelTimeMatrix(
        network, origins=sites, destinations=sites, departure=start,
        departure_time_window=window, percentiles=[25, 50, 75],
        transport_modes=[r5py.TransportMode.TRANSIT, r5py.TransportMode.WALK],
        speed_walking=5.0, max_time=dt.timedelta(hours=3),
    )
    print(f"r5: {time.perf_counter() - t:.1f} s (network build and matrix)")

    n = len(points)
    def grid(column):
        out = np.full((n, n), np.nan)
        out[table.from_id.to_numpy(), table.to_id.to_numpy()] = table[column].to_numpy(dtype=float)
        return out
    p25, p50, p75 = grid("travel_time_p25"), grid("travel_time_p50"), grid("travel_time_p75")

    # --- comparison ----------------------------------------------------------------
    mask = ~np.isnan(ours) & ~np.isnan(p50) & (p50 >= 10) & ~np.eye(n, dtype=bool)
    rel = (ours[mask] - p50[mask]) / p50[mask]
    absolute = np.abs(ours[mask] - p50[mask])
    within = ((ours[mask] >= p25[mask] - 1) & (ours[mask] <= p75[mask] + 1)).mean()
    print(f"pairs compared (r5 median >= 10 min): {mask.sum()}")
    print(f"error vs r5 median: median {np.median(rel):+.1%}, mean absolute {np.mean(np.abs(rel)):.1%}, "
          f"10th-90th percentile {np.quantile(rel, 0.1):+.1%} to {np.quantile(rel, 0.9):+.1%}")
    print(f"absolute error: median {np.median(absolute):.1f} min, 90th percentile {np.quantile(absolute, 0.9):.1f} min")
    print(f"within r5's 25th-75th percentile range (+-1 min): {within:.0%}")


if __name__ == "__main__":
    main()
