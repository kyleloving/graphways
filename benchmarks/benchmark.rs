//! Unified benchmark harness for the local PBF pipeline.
//!
//! This measures both one-shot setup (PBF parse, graph build, spatial index)
//! and steady-state hot-path work (snap origin, Dijkstra, contour construction).
//!
//! Usage:
//! This file is a repo-local profiling harness. If you want to run it through
//! Cargo, temporarily expose it as an example target or move it into
//! `examples/benchmark.rs`.
//!
//! Env vars:
//!     ITERS=10          measured hot-path iterations (default 10)
//!     WARMUP=3          warm-up hot-path iterations (default 3)
//!     LAT, LON          origin override (default Dupont Circle, DC)
//!     LIMITS=300,600    comma-separated time budgets in seconds
//!     NETWORK=drive|walk|bike
//!     RETAIN_ALL=1      skip graph simplification
//!     PROFILE_LOOP=1    run the production hot path repeatedly for profiler sampling

use std::env;
use std::time::{Duration, Instant};

use graphways::graph::{create_graph, SpatialGraph};
use graphways::isochrone::{build_isochrone_polygons, isochrones_from_node};
use graphways::overpass::NetworkType;
use graphways::pbf::read_pbf;
use graphways::reachability::{compute_reachability_with, ReachabilityResult};

#[derive(Debug)]
struct Config {
    path: String,
    lat: f64,
    lon: f64,
    limits: Vec<f64>,
    warmup: usize,
    iters: usize,
    network_type: NetworkType,
    retain_all: bool,
    profile_loop: bool,
}

struct SetupTimings {
    parse_pbf: Duration,
    create_graph: Duration,
    spatial_index: Duration,
    input_nodes: usize,
    input_ways: usize,
    pois: usize,
    graph_nodes: usize,
    graph_edges: usize,
}

struct Stage {
    name: &'static str,
    samples: Vec<Duration>,
}

struct Percentiles {
    min: Duration,
    p50: Duration,
    p95: Duration,
    p99: Duration,
    max: Duration,
    mean: Duration,
}

struct HotPathStats {
    nearest_node: Stage,
    dijkstra: Stage,
    hulls_sequential: Stage,
    isochrones_parallel: Stage,
    hull_limits: Vec<LimitHullStats>,
    total_settled: u64,
    total_in_budget: u64,
    total_edge_evals: u64,
}

struct LimitHullStats {
    limit_s: f64,
    samples: Vec<Duration>,
    total_points: u64,
}

impl Config {
    fn from_env() -> Self {
        let args: Vec<String> = env::args().collect();
        let path = args
            .get(1)
            .cloned()
            .unwrap_or_else(|| "data/district-of-columbia-latest.osm.pbf".to_string());

        Self {
            path,
            lat: env_f64("LAT", 38.9097),
            lon: env_f64("LON", -77.0432),
            limits: env::var("LIMITS")
                .map(|s| parse_limits(&s))
                .unwrap_or_else(|_| vec![300.0, 600.0, 900.0, 1200.0, 1500.0, 1800.0]),
            warmup: env_usize("WARMUP", 3),
            iters: env_usize("ITERS", 10),
            network_type: env::var("NETWORK")
                .map(|s| parse_network(&s))
                .unwrap_or(NetworkType::Drive),
            retain_all: env::var("RETAIN_ALL").is_ok(),
            profile_loop: env::var("PROFILE_LOOP").is_ok(),
        }
    }
}

impl Stage {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            samples: Vec::new(),
        }
    }

    fn record(&mut self, duration: Duration) {
        self.samples.push(duration);
    }

    fn percentiles(&self) -> Option<Percentiles> {
        if self.samples.is_empty() {
            return None;
        }
        let mut samples = self.samples.clone();
        samples.sort();
        let pick = |p: f64| {
            let idx = ((samples.len() - 1) as f64 * p).round() as usize;
            samples[idx]
        };
        let mean = samples.iter().sum::<Duration>() / samples.len() as u32;
        Some(Percentiles {
            min: samples[0],
            p50: pick(0.50),
            p95: pick(0.95),
            p99: pick(0.99),
            max: samples[samples.len() - 1],
            mean,
        })
    }
}

impl HotPathStats {
    fn new(limits: &[f64]) -> Self {
        Self {
            nearest_node: Stage::new("nearest_node"),
            dijkstra: Stage::new("dijkstra"),
            hulls_sequential: Stage::new("hulls_seq"),
            isochrones_parallel: Stage::new("isochrones_par"),
            hull_limits: limits
                .iter()
                .map(|&limit_s| LimitHullStats {
                    limit_s,
                    samples: Vec::new(),
                    total_points: 0,
                })
                .collect(),
            total_settled: 0,
            total_in_budget: 0,
            total_edge_evals: 0,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = Config::from_env();
    print_config(&config);

    let (spatial_graph, setup) = build_graph(&config)?;
    print_setup(&setup);

    let stats = run_hot_path(&config, &spatial_graph)?;
    print_hot_path(&config, &setup, &stats);

    if config.profile_loop {
        run_profile_loop(&config, &spatial_graph)?;
    }

    Ok(())
}

fn build_graph(
    config: &Config,
) -> Result<(SpatialGraph, SetupTimings), Box<dyn std::error::Error>> {
    let t = Instant::now();
    let (data, pois) = read_pbf(&config.path, config.network_type)?;
    let parse_pbf = t.elapsed();
    let input_nodes = data.nodes.len();
    let input_ways = data.ways.len();
    let poi_count = pois.len();

    let bidirectional = matches!(config.network_type, NetworkType::Walk);
    let t = Instant::now();
    let graph = create_graph(data.nodes, data.ways, config.retain_all, bidirectional);
    let create_graph_time = t.elapsed();
    let graph_nodes = graph.node_count();
    let graph_edges = graph.edge_count();

    let t = Instant::now();
    let mut spatial_graph = SpatialGraph::new(graph);
    spatial_graph.snap_pois(&pois);
    let spatial_index = t.elapsed();

    Ok((
        spatial_graph,
        SetupTimings {
            parse_pbf,
            create_graph: create_graph_time,
            spatial_index,
            input_nodes,
            input_ways,
            pois: poi_count,
            graph_nodes,
            graph_edges,
        },
    ))
}

fn run_hot_path(
    config: &Config,
    sg: &SpatialGraph,
) -> Result<HotPathStats, Box<dyn std::error::Error>> {
    let max_limit = config.limits.iter().cloned().fold(0.0_f64, f64::max);
    let mut stats = HotPathStats::new(&config.limits);

    for i in 0..(config.warmup + config.iters) {
        let measure = i >= config.warmup;

        let t = Instant::now();
        let start = sg
            .nearest_node(config.lat, config.lon)
            .ok_or("nearest_node")?;
        let nearest_node_time = t.elapsed();

        // Unbounded search, to measure how much a budget saves.
        let mut evals = 0_u64;
        let t = Instant::now();
        let unbounded = compute_reachability_with(sg, start, f64::INFINITY, |e| {
            evals += 1;
            e.weight.travel_time(config.network_type)
        });
        let dijkstra_time = t.elapsed();

        let settled = unbounded.times.len() as u64;
        let in_budget = unbounded
            .times
            .values()
            .filter(|&&time| time <= max_limit)
            .count() as u64;

        let result = ReachabilityResult {
            max_cost: max_limit,
            ..unbounded
        };
        let t = Instant::now();
        let limit_hull_samples = build_hulls_by_limit(sg, &result, &config.limits);
        let hulls_sequential_time = t.elapsed();

        let t = Instant::now();
        let _polygons = isochrones_from_node(sg, start, &config.limits, config.network_type);
        let isochrones_parallel_time = t.elapsed();

        if measure {
            stats.nearest_node.record(nearest_node_time);
            stats.dijkstra.record(dijkstra_time);
            stats.hulls_sequential.record(hulls_sequential_time);
            stats.isochrones_parallel.record(isochrones_parallel_time);
            stats.total_settled += settled;
            stats.total_in_budget += in_budget;
            stats.total_edge_evals += evals;
            for (limit_stats, sample) in stats.hull_limits.iter_mut().zip(limit_hull_samples) {
                limit_stats.samples.push(sample.duration);
                limit_stats.total_points += sample.points as u64;
            }
        }
    }

    Ok(stats)
}

fn build_hulls_by_limit(
    sg: &SpatialGraph,
    result: &ReachabilityResult,
    limits: &[f64],
) -> Vec<LimitHullSample> {
    let mut samples = Vec::with_capacity(limits.len());
    for &budget in limits {
        let t = Instant::now();
        let _ = build_isochrone_polygons(&sg.graph, result, &[budget]);
        let point_count = result
            .times
            .values()
            .filter(|&&time| time <= budget)
            .count();
        samples.push(LimitHullSample {
            duration: t.elapsed(),
            points: point_count,
        });
    }
    samples
}

struct LimitHullSample {
    duration: Duration,
    points: usize,
}

fn run_profile_loop(config: &Config, sg: &SpatialGraph) -> Result<(), Box<dyn std::error::Error>> {
    let start_node = sg
        .nearest_node(config.lat, config.lon)
        .ok_or("nearest_node")?;

    println!();
    println!("profile loop:");
    let start = Instant::now();
    let mut iters = 0_u32;
    while start.elapsed().as_secs() < 20 {
        let _ = isochrones_from_node(sg, start_node, &config.limits, config.network_type);
        iters += 1;
    }
    println!(
        "  ran {} production hot-path iterations in {:.2}s",
        iters,
        start.elapsed().as_secs_f64()
    );

    Ok(())
}

fn print_config(config: &Config) {
    println!("=== graphways benchmark ===");
    println!("pbf:        {}", config.path);
    println!("center:     {}, {}", config.lat, config.lon);
    println!("limits:     {:?}s", config.limits);
    println!("shape:      triangulated contour");
    println!("network:    {:?}", config.network_type);
    println!("retain_all: {}", config.retain_all);
    println!("warmup:     {}", config.warmup);
    println!("iters:      {}", config.iters);
    println!();
}

fn print_setup(setup: &SetupTimings) {
    println!("setup:");
    println!(
        "  parse_pbf      {}ms  ({} graph nodes in extract, {} ways, {} POIs)",
        fmt(setup.parse_pbf),
        setup.input_nodes,
        setup.input_ways,
        setup.pois
    );
    println!(
        "  create_graph   {}ms  ({} nodes, {} edges)",
        fmt(setup.create_graph),
        setup.graph_nodes,
        setup.graph_edges
    );
    println!(
        "  spatial_index  {}ms  (includes POI snapping)",
        fmt(setup.spatial_index)
    );
    println!();
}

fn print_hot_path(config: &Config, setup: &SetupTimings, stats: &HotPathStats) {
    println!("hot path ({} measured):", config.iters);
    println!(
        "  {:<18} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "stage", "min", "p50", "p95", "p99", "max", "mean"
    );

    for stage in [
        &stats.nearest_node,
        &stats.dijkstra,
        &stats.hulls_sequential,
        &stats.isochrones_parallel,
    ] {
        if let Some(p) = stage.percentiles() {
            println!(
                "  {:<18} {} {} {} {} {} {}",
                stage.name,
                fmt(p.min),
                fmt(p.p50),
                fmt(p.p95),
                fmt(p.p99),
                fmt(p.max),
                fmt(p.mean)
            );
        }
    }
    println!();

    let avg_settled = stats.total_settled as f64 / config.iters as f64;
    let avg_in_budget = stats.total_in_budget as f64 / config.iters as f64;
    let avg_evals = stats.total_edge_evals as f64 / config.iters as f64;
    let max_limit = config.limits.iter().cloned().fold(0.0_f64, f64::max);
    let waste = if avg_settled > 0.0 {
        1.0 - avg_in_budget / avg_settled
    } else {
        0.0
    };

    println!("dijkstra quality:");
    println!(
        "  settled:       {:.0} / {} graph nodes ({:.1}%)",
        avg_settled,
        setup.graph_nodes,
        100.0 * avg_settled / setup.graph_nodes as f64
    );
    println!(
        "  in_budget:     {:.0} ({:.1}% of settled, budget = {}s)",
        avg_in_budget,
        if avg_settled > 0.0 {
            100.0 * avg_in_budget / avg_settled
        } else {
            0.0
        },
        max_limit as u64
    );
    println!(
        "  wasted_ratio:  {:.1}%  (upper bound on bounded-Dijkstra-only speedup)",
        100.0 * waste
    );
    println!("  edge_evals:    {:.0}", avg_evals);
    println!();

    if let (Some(dijkstra), Some(hulls), Some(parallel)) = (
        stats.dijkstra.percentiles(),
        stats.hulls_sequential.percentiles(),
        stats.isochrones_parallel.percentiles(),
    ) {
        let sequential = dijkstra.mean + hulls.mean;
        if sequential.as_nanos() > 0 {
            println!("bottleneck:");
            println!("  sequential_mean: {}ms", fmt(sequential));
            println!(
                "  dijkstra:        {:>5.1}%",
                100.0 * dijkstra.mean.as_secs_f64() / sequential.as_secs_f64()
            );
            println!(
                "  hulls:           {:>5.1}%",
                100.0 * hulls.mean.as_secs_f64() / sequential.as_secs_f64()
            );
            println!(
                "  parallel_mean:   {}ms ({:.2}x vs sequential)",
                fmt(parallel.mean),
                sequential.as_secs_f64() / parallel.mean.as_secs_f64()
            );
        }
    }

    if !stats.hull_limits.is_empty() {
        println!();
        println!(
            "contour breakdown (single-limit calls; saturated-polygon reuse is only reflected above):"
        );
        println!("  {:>8} {:>10} {:>10}", "limit_s", "points", "mean_ms");
        for limit in &stats.hull_limits {
            if limit.samples.is_empty() {
                continue;
            }
            let mean = limit.samples.iter().sum::<Duration>() / limit.samples.len() as u32;
            let avg_points = limit.total_points as f64 / limit.samples.len() as f64;
            println!(
                "  {:>8.0} {:>10.0} {}",
                limit.limit_s,
                avg_points,
                fmt(mean)
            );
        }
    }
}

fn env_f64(name: &str, default: f64) -> f64 {
    env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

fn env_usize(name: &str, default: usize) -> usize {
    env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

fn parse_network(value: &str) -> NetworkType {
    match value.to_ascii_lowercase().as_str() {
        "walk" => NetworkType::Walk,
        "bike" => NetworkType::Bike,
        _ => NetworkType::Drive,
    }
}

fn parse_limits(value: &str) -> Vec<f64> {
    value
        .split(',')
        .filter_map(|time| time.trim().parse().ok())
        .collect()
}

fn fmt(duration: Duration) -> String {
    format!("{:>8.2}", duration.as_secs_f64() * 1e3)
}
