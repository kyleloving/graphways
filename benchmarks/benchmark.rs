//! Benchmark harness for the local PBF pipeline.
//!
//! Times one-shot setup (PBF load, routing preparation, save and load) and
//! steady-state queries (reachability, isochrones, routes with and without
//! the routing index, travel-time matrices, prisms) on a real extract.
//!
//! ```text
//! cargo bench --bench pipeline                      # Munich extract, driving
//! NETWORK=walk cargo bench --bench pipeline
//! cargo bench --bench pipeline -- path/to/area.osm.pbf
//! ```
//!
//! Env vars:
//!     NETWORK=drive|walk|bike   network type (default drive)
//!     ITERS=20                  queries per measured stage (default 20)
//!     BUDGET=600                reachability budget in seconds (default 600)
//!     VERIFY=1                  also check routes against plain Dijkstra
//!
//! Query points are drawn uniformly from the middle half of the graph's
//! bounding box with a fixed seed, so runs are comparable.

use std::time::{Duration, Instant};

use graphways::feasibility::compute_feasibility;
use graphways::graph::SpatialGraph;
use graphways::overpass::NetworkType;
use graphways::reachability::compute_reachability;

type Point = (f64, f64);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // `cargo bench` passes `--bench`; anything else is the PBF path.
    let path = std::env::args()
        .skip(1)
        .find(|a| !a.starts_with("--"))
        .unwrap_or_else(|| "benchmarks/data/munich.osm.pbf".into());
    let network = match std::env::var("NETWORK").as_deref() {
        Ok("walk") => NetworkType::Walk,
        Ok("bike") => NetworkType::Bike,
        _ => NetworkType::Drive,
    };
    let iters = env_or("ITERS", 20.0) as usize;
    let budget = env_or("BUDGET", 600.0);
    println!("{path} ({network:?}), {iters} queries per stage, budget {budget} s\n");

    let (sg, load) = timed(|| SpatialGraph::from_pbf(&path, network, false));
    let sg = sg?;
    report(
        &format!(
            "load PBF ({} nodes, {} edges, {} restricted turns)",
            sg.graph.node_count(),
            sg.graph.edge_count(),
            sg.forbidden_turns().len()
        ),
        load,
    );

    let mut points = PointSampler::new(&sg);
    let origins: Vec<Point> = (0..iters).map(|_| points.next()).collect();
    let pairs: Vec<(Point, Point)> = (0..iters).map(|_| (points.next(), points.next())).collect();

    per_query("reachability", &origins, |&o| {
        sg.reachability(o, budget, None).expect("reachability");
    });
    let limits = [budget / 3.0, budget * 2.0 / 3.0, budget];
    per_query("isochrones (3 limits)", &origins, |&o| {
        sg.isochrones(o, &limits, None).expect("isochrones");
    });
    // Infeasible trips and unroutable pairs are legitimate answers.
    per_query("prism", &pairs, |&(o, d)| {
        let (o, d) = (sg.snap_point(o).unwrap(), sg.snap_point(d).unwrap());
        let _ = compute_feasibility(&sg, &o, &d, budget * 2.0);
    });
    per_query("route (A*, unprepared)", &pairs, |&(o, d)| {
        let _ = sg.route(o, d, None);
    });
    let small: Vec<Point> = (0..100).map(|_| points.next()).collect();
    let (_, t) = timed(|| sg.travel_time_matrix(&small[..10], &small, None));
    report("matrix 10 x 100 (Dijkstra, unprepared)", t);

    let ((), prepare) = timed(|| sg.prepare_routing());
    report("prepare routing", prepare);
    per_query("route (prepared)", &pairs, |&(o, d)| {
        let _ = sg.route(o, d, None);
    });
    let large: Vec<Point> = (0..1000).map(|_| points.next()).collect();
    let (_, t) = timed(|| sg.travel_time_matrix(&small, &small, None));
    report("matrix 100 x 100 (prepared)", t);
    let (_, t) = timed(|| sg.travel_time_matrix(&large, &large, None));
    report("matrix 1000 x 1000 (prepared)", t);

    let file = std::env::temp_dir().join("graphways-bench.graph");
    let (saved, t) = timed(|| sg.save(&file));
    saved?;
    let size = std::fs::metadata(&file)?.len() as f64 / 1e6;
    report(&format!("save ({size:.1} MB)"), t);
    let (loaded, t) = timed(|| SpatialGraph::load(&file));
    let loaded = loaded?;
    report("load saved graph", t);
    std::fs::remove_file(&file).ok();
    assert!(loaded.is_routing_prepared());

    if std::env::var("VERIFY").is_ok() {
        let mut worst = 0.0_f64;
        for &(o, d) in &pairs {
            let (Some(so), Some(sd)) = (sg.snap_point(o), sg.snap_point(d)) else {
                continue;
            };
            let exact = compute_reachability(&sg, &so, f64::INFINITY).time_to(&sg, &sd);
            let fast = loaded.route(o, d, None).ok().map(|r| r.duration_s);
            match (exact, fast) {
                // A route may beat the network search by staying on one road.
                (Some(x), Some(y)) => worst = worst.max(y - x),
                (None, None) => {}
                other => return Err(format!("route disagrees with Dijkstra: {other:?}").into()),
            }
        }
        println!("\nverify: routes exceed Dijkstra by at most {worst:.2e} s");
    }
    Ok(())
}

fn per_query<T>(label: &str, items: &[T], mut run: impl FnMut(&T)) {
    let mut samples: Vec<Duration> = items
        .iter()
        .map(|item| {
            let start = Instant::now();
            run(item);
            start.elapsed()
        })
        .collect();
    samples.sort();
    let median = samples[samples.len() / 2];
    let p95 = samples[(samples.len() * 95 / 100).min(samples.len() - 1)];
    println!(
        "{label:<44} median {:>9}   p95 {:>9}",
        fmt(median),
        fmt(p95)
    );
}

fn timed<T>(f: impl FnOnce() -> T) -> (T, Duration) {
    let start = Instant::now();
    let value = f();
    (value, start.elapsed())
}

fn report(label: &str, elapsed: Duration) {
    println!("{label:<44} {:>16}", fmt(elapsed));
}

fn fmt(d: Duration) -> String {
    let ms = d.as_secs_f64() * 1e3;
    if ms >= 1000.0 {
        format!("{:.2} s", ms / 1000.0)
    } else {
        format!("{ms:.3} ms")
    }
}

fn env_or(name: &str, default: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Deterministic points in the middle half of the graph's bounding box.
struct PointSampler {
    state: u64,
    centre: Point,
    half_span: Point,
}

impl PointSampler {
    fn new(sg: &SpatialGraph) -> Self {
        let (mut south, mut north, mut west, mut east) = (90.0, -90.0, 180.0, -180.0);
        for n in sg.graph.node_weights() {
            south = f64::min(south, n.lat);
            north = f64::max(north, n.lat);
            west = f64::min(west, n.lon);
            east = f64::max(east, n.lon);
        }
        PointSampler {
            state: 42,
            centre: ((south + north) / 2.0, (west + east) / 2.0),
            half_span: ((north - south) / 4.0, (east - west) / 4.0),
        }
    }

    fn uniform(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    }

    fn next(&mut self) -> Point {
        let (dlat, dlon) = (self.uniform(), self.uniform());
        (
            self.centre.0 + dlat * self.half_span.0,
            self.centre.1 + dlon * self.half_span.1,
        )
    }
}
