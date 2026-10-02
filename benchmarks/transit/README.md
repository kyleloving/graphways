# Transit validation

`validate.py` checks graphways' frequency-based transit model
(`SpatialGraph.with_transit`) against [r5](https://github.com/conveyal/r5)
through r5py, on the same street network, GTFS feed, points and time window.
r5 routes every departure minute in the window and reports percentiles of the
travel time; graphways estimates one expected travel time, which is compared
with r5's median.

```bash
pip install graphways r5py geopandas   # r5py needs Java 21
python benchmarks/transit/validate.py --pbf munich.osm.pbf --gtfs mvg.zip \
    --date 2026-10-06 --start 07:00 --end 09:00 --points 300 --max-memory 4G
```

## Munich, October 2026

MVG feed (U-Bahn, tram and bus; no S-Bahn), Tuesday 6 October 2026,
07:00-09:00, 300 random points in central Munich, walking at 5 km/h in both
tools. 88,045 origin-destination pairs with an r5 median of at least 10 min:

| | |
|---|---|
| Median error vs r5's median | 0.0% |
| Mean absolute error | 4.3% |
| 10th to 90th percentile of the error | -6.4% to +7.1% |
| Absolute error, median / 90th percentile | 1.1 / 3.0 min |
| Within r5's 25th-75th percentile range (+-1 min) | 83% |

For reference, walking alone differs from r5's walking times by a median of
-2.6% (mean absolute 2.9%): part of the transit error is the street model.

Building the transit graph took 7 s and the 300 x 300 matrix 25 s including
routing preparation (later queries reuse it); r5 took 212 s for the same
matrix over the window.

Waits on one line's variants (short turns, branches) heading for the same
next stop are pooled. Without pooling, graphways ran 3% slower than r5's
median; pooling across different lines too ran 5% faster.

## When the model is weaker

The expected wait is half the headway. That is close to the median over a
window when service is frequent, but a single departure time with sparse
service (a bus every 30 minutes) can be much better or worse than the
estimate. Timed connections, first and last trips, and service changes
within the window are not represented. For those questions use a
schedule-based tool, or several narrower windows.
