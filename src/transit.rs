//! Public transport from GTFS timetables, as a frequency-based network on
//! top of the walking graph.
//!
//! Timetables are time-dependent; graphways' searches are not. So instead of
//! following individual departures, the network describes one time window
//! (say 07:00-09:00 on a Tuesday) by its *frequencies*:
//!
//! * Each sequence of stops served by a line becomes a *pattern*, with one
//!   node per stop ("on board at stop k") joined by ride edges carrying the
//!   average running time in the window.
//! * Boarding a pattern at a stop costs the expected wait, `wait_factor`
//!   times the headway there (half the headway by default: a passenger who
//!   turns up at a random time waits half the gap between departures).
//!   Variants of the same line heading for the same next stop count
//!   together.
//! * Alighting is free; changing lines means alighting, walking (through
//!   the street network, or along a GTFS `transfers.txt` link) and boarding
//!   again, paying a new wait.
//! * Each stop is linked to the nearest walkable street.
//!
//! Every query (routes, matrices, isochrones, accessibility) then works on
//! the combined graph unchanged, and stays exact for this model. The model
//! approximates a timetable best where service is frequent; with sparse
//! service the real wait depends on when one sets off, which a single
//! expected wait cannot capture. Results are comparable to the *median* over
//! the window reported by schedule-based tools such as r5.

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::sync::Arc;

use petgraph::graph::NodeIndex;

use crate::error::OsmGraphError;
use crate::graph::{Edge, LatLon, OsmNode, OsmTag, RoadGraph, SpatialGraph};
use crate::overpass::NetworkType;

/// Which service to model and how.
#[derive(Debug, Clone, PartialEq)]
pub struct TransitOptions {
    /// Service date as `YYYYMMDD` (e.g. `20261006`).
    pub date: u32,
    /// Start and end of the time window, in seconds after midnight.
    pub start_s: u32,
    pub end_s: u32,
    /// Expected wait as a share of the headway (0.5 = half the headway).
    pub wait_factor: f64,
    /// Stops farther than this from any walkable street are left out.
    pub max_link_m: f64,
}

impl TransitOptions {
    /// Options for `date` (`"2026-10-06"` or `"20261006"`) between `start`
    /// and `end` (`"07:00"`, `"07:00:00"`).
    pub fn new(date: &str, start: &str, end: &str) -> Result<Self, OsmGraphError> {
        let digits: String = date.chars().filter(|c| *c != '-').collect();
        let date = parse_date(&digits)
            .ok_or_else(|| OsmGraphError::InvalidInput(format!("invalid date '{date}'")))?;
        let start_s = parse_time(start)
            .ok_or_else(|| OsmGraphError::InvalidInput(format!("invalid time '{start}'")))?;
        let end_s = parse_time(end)
            .ok_or_else(|| OsmGraphError::InvalidInput(format!("invalid time '{end}'")))?;
        if end_s <= start_s {
            return Err(OsmGraphError::InvalidInput(
                "the time window must end after it starts".into(),
            ));
        }
        Ok(TransitOptions {
            date,
            start_s,
            end_s,
            wait_factor: 0.5,
            max_link_m: 300.0,
        })
    }
}

/// `YYYYMMDD` as a number, if it is a real date.
fn parse_date(s: &str) -> Option<u32> {
    if s.len() != 8 || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let n: u32 = s.parse().ok()?;
    let (y, m, d) = (n / 10_000, n / 100 % 100, n % 100);
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let days = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    ((1..=12).contains(&m) && d >= 1 && d <= days[m as usize - 1]).then_some(n)
}

/// `HH:MM[:SS]` as seconds after midnight; hours may exceed 23 (GTFS).
fn parse_time(s: &str) -> Option<u32> {
    let mut parts = s.trim().split(':');
    let h: u32 = parts.next()?.trim().parse().ok()?;
    let m: u32 = parts.next()?.trim().parse().ok()?;
    let sec: u32 = match parts.next() {
        Some(p) => p.trim().parse().ok()?,
        None => 0,
    };
    (parts.next().is_none() && m < 60 && sec < 60).then_some(h * 3600 + m * 60 + sec)
}

/// Day of the week for `YYYYMMDD`, 0 = Monday.
fn weekday(date: u32) -> usize {
    // Sakamoto's method gives 0 = Sunday.
    const T: [i64; 12] = [0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4];
    let (mut y, m, d) = (
        (date / 10_000) as i64,
        (date / 100 % 100) as i64,
        (date % 100) as i64,
    );
    if m < 3 {
        y -= 1;
    }
    let sunday_based = (y + y / 4 - y / 100 + y / 400 + T[m as usize - 1] + d) % 7;
    ((sunday_based + 6) % 7) as usize
}

// ---------------------------------------------------------------------------
// Reading GTFS
// ---------------------------------------------------------------------------

/// The files of a feed, from a `.zip` or an unpacked directory.
struct FeedSource<'a> {
    path: &'a Path,
}

impl FeedSource<'_> {
    /// The contents of `name`, or `None` if the feed lacks it.
    fn read(&self, name: &str) -> Result<Option<Vec<u8>>, OsmGraphError> {
        if self.path.is_dir() {
            let file = self.path.join(name);
            return match std::fs::read(&file) {
                Ok(bytes) => Ok(Some(bytes)),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
                Err(e) => Err(e.into()),
            };
        }
        let mut archive = zip::ZipArchive::new(File::open(self.path)?)
            .map_err(|e| OsmGraphError::InvalidInput(format!("not a GTFS zip: {e}")))?;
        // Some feeds nest their files in a folder inside the zip.
        let entry = (0..archive.len()).find(|&i| {
            archive
                .by_index(i)
                .is_ok_and(|f| f.name().rsplit('/').next() == Some(name))
        });
        let Some(i) = entry else { return Ok(None) };
        let mut file = archive
            .by_index(i)
            .map_err(|e| OsmGraphError::InvalidInput(format!("unreadable {name}: {e}")))?;
        let mut bytes = Vec::with_capacity(file.size() as usize);
        file.read_to_end(&mut bytes)?;
        Ok(Some(bytes))
    }

    fn required(&self, name: &str) -> Result<Vec<u8>, OsmGraphError> {
        self.read(name)?
            .ok_or_else(|| OsmGraphError::InvalidInput(format!("GTFS feed has no {name}")))
    }
}

/// A CSV table with named columns.
struct Table {
    reader: csv::Reader<std::io::Cursor<Vec<u8>>>,
    columns: HashMap<String, usize>,
    name: String,
}

impl Table {
    fn new(name: &str, bytes: Vec<u8>) -> Result<Self, OsmGraphError> {
        let mut reader = csv::ReaderBuilder::new()
            .flexible(true)
            .trim(csv::Trim::All)
            .from_reader(std::io::Cursor::new(bytes));
        let columns = reader
            .headers()
            .map_err(|e| bad(name, e))?
            .iter()
            .enumerate()
            .map(|(i, h)| (h.trim_start_matches('\u{feff}').to_owned(), i))
            .collect();
        Ok(Table {
            reader,
            columns,
            name: name.to_owned(),
        })
    }

    fn column(&self, column: &str) -> Result<usize, OsmGraphError> {
        self.columns.get(column).copied().ok_or_else(|| {
            OsmGraphError::InvalidInput(format!("{} has no column '{column}'", self.name))
        })
    }

    fn optional(&self, column: &str) -> Option<usize> {
        self.columns.get(column).copied()
    }

    /// Call `row` for each record.
    fn each(
        mut self,
        mut row: impl FnMut(&csv::StringRecord) -> Result<(), OsmGraphError>,
    ) -> Result<(), OsmGraphError> {
        let mut record = csv::StringRecord::new();
        while self
            .reader
            .read_record(&mut record)
            .map_err(|e| bad(&self.name, e))?
        {
            row(&record)?;
        }
        Ok(())
    }
}

fn bad(name: &str, e: csv::Error) -> OsmGraphError {
    OsmGraphError::InvalidInput(format!("malformed {name}: {e}"))
}

fn field(record: &csv::StringRecord, column: Option<usize>) -> &str {
    column.and_then(|c| record.get(c)).unwrap_or("")
}

struct Stop {
    id: String,
    name: String,
    lat: f64,
    lon: f64,
}

/// One stop of a trip, in order.
#[derive(Clone, Copy)]
struct TripStop {
    sequence: u32,
    stop: u32,
    arrival: u32,
    departure: u32,
    pickup: bool,
    drop_off: bool,
}

/// A line's sequence of stops, with what the window's trips did on it.
struct Pattern {
    label: String,
    route_type: String,
    stops: Vec<u32>,
    /// Per stop: departures in the window allowing boarding.
    boardings: Vec<u32>,
    /// Per stop: whether any trip lets passengers off.
    drop_off: Vec<bool>,
    /// Per hop k -> k+1: summed running time and count of trips.
    ride: Vec<(f64, u32)>,
}

/// The frequency model of a feed's service in a window.
struct Service {
    stops: Vec<Stop>,
    patterns: Vec<Pattern>,
    /// `transfers.txt` links: `(from stop, to stop, seconds)`.
    transfers: Vec<(u32, u32, f64)>,
}

fn time_field(name: &str, value: &str) -> Result<Option<u32>, OsmGraphError> {
    if value.is_empty() {
        return Ok(None);
    }
    parse_time(value)
        .map(Some)
        .ok_or_else(|| OsmGraphError::InvalidInput(format!("{name}: invalid time '{value}'")))
}

impl Service {
    fn read(path: &Path, options: &TransitOptions) -> Result<Self, OsmGraphError> {
        let feed = FeedSource { path };

        // Stops (platforms and plain stops; stations and entrances are not boarded).
        let mut stops = Vec::new();
        let mut stop_index: HashMap<String, u32> = HashMap::new();
        let table = Table::new("stops.txt", feed.required("stops.txt")?)?;
        let (id, name, lat, lon) = (
            table.column("stop_id")?,
            table.optional("stop_name"),
            table.column("stop_lat")?,
            table.column("stop_lon")?,
        );
        let location_type = table.optional("location_type");
        table.each(|r| {
            if !matches!(field(r, location_type), "" | "0") {
                return Ok(());
            }
            let (Ok(lat), Ok(lon)) = (field(r, Some(lat)).parse(), field(r, Some(lon)).parse())
            else {
                return Ok(());
            };
            let stop_id = field(r, Some(id)).to_owned();
            stop_index.insert(stop_id.clone(), stops.len() as u32);
            stops.push(Stop {
                id: stop_id,
                name: field(r, name).to_owned(),
                lat,
                lon,
            });
            Ok(())
        })?;

        // Services running on the date.
        let active = active_services(&feed, options.date)?;

        // Routes: a label per route and its mode.
        let mut routes: HashMap<String, (String, String)> = HashMap::new();
        let table = Table::new("routes.txt", feed.required("routes.txt")?)?;
        let (id, short, long, kind) = (
            table.column("route_id")?,
            table.optional("route_short_name"),
            table.optional("route_long_name"),
            table.optional("route_type"),
        );
        table.each(|r| {
            let label = match field(r, short) {
                "" => field(r, long),
                s => s,
            };
            routes.insert(
                field(r, Some(id)).to_owned(),
                (label.to_owned(), field(r, kind).to_owned()),
            );
            Ok(())
        })?;

        // Trips running on the date, by route label.
        let mut trip_route: HashMap<String, (String, String)> = HashMap::new();
        let table = Table::new("trips.txt", feed.required("trips.txt")?)?;
        let (route, service, trip) = (
            table.column("route_id")?,
            table.column("service_id")?,
            table.column("trip_id")?,
        );
        table.each(|r| {
            if active.contains(field(r, Some(service))) {
                let label = routes
                    .get(field(r, Some(route)))
                    .cloned()
                    .unwrap_or_else(|| (field(r, Some(route)).to_owned(), String::new()));
                trip_route.insert(field(r, Some(trip)).to_owned(), label);
            }
            Ok(())
        })?;

        // Stop times of those trips.
        let mut trips: HashMap<&str, Vec<TripStop>> = HashMap::new();
        let trip_ids: HashMap<String, ()> = trip_route.keys().map(|k| (k.clone(), ())).collect();
        let table = Table::new("stop_times.txt", feed.required("stop_times.txt")?)?;
        let (trip, arrival, departure, stop, sequence) = (
            table.column("trip_id")?,
            table.column("arrival_time")?,
            table.column("departure_time")?,
            table.column("stop_id")?,
            table.column("stop_sequence")?,
        );
        let (pickup, drop_off) = (
            table.optional("pickup_type"),
            table.optional("drop_off_type"),
        );
        table.each(|r| {
            let Some((trip_id, _)) = trip_ids.get_key_value(field(r, Some(trip))) else {
                return Ok(());
            };
            let Some(&stop) = stop_index.get(field(r, Some(stop))) else {
                return Ok(());
            };
            let arrival_t = time_field("stop_times.txt", field(r, Some(arrival)))?;
            let departure_t = time_field("stop_times.txt", field(r, Some(departure)))?;
            // Untimed stops (interpolated in GTFS) are skipped.
            let (Some(a), Some(d)) = (arrival_t.or(departure_t), departure_t.or(arrival_t)) else {
                return Ok(());
            };
            trips.entry(trip_id.as_str()).or_default().push(TripStop {
                sequence: field(r, Some(sequence)).parse().unwrap_or(0),
                stop,
                arrival: a,
                departure: d.max(a),
                pickup: field(r, pickup) != "1",
                drop_off: field(r, drop_off) != "1",
            });
            Ok(())
        })?;

        // Group trips into patterns and aggregate the window.
        let mut patterns: Vec<Pattern> = Vec::new();
        let mut pattern_of: HashMap<(String, Vec<u32>), usize> = HashMap::new();
        let (start, end) = (options.start_s, options.end_s);
        let mut trip_list: Vec<(&str, Vec<TripStop>)> = trips.into_iter().collect();
        trip_list.sort_unstable_by(|a, b| a.0.cmp(b.0));
        for (trip_id, mut stops_of_trip) in trip_list {
            stops_of_trip.sort_unstable_by_key(|s| s.sequence);
            if stops_of_trip.len() < 2 {
                continue;
            }
            let in_window = |t: u32| t >= start && t < end;
            if !stops_of_trip.iter().any(|s| in_window(s.departure)) {
                continue;
            }
            let (label, route_type) = &trip_route[trip_id];
            let sequence: Vec<u32> = stops_of_trip.iter().map(|s| s.stop).collect();
            let key = (label.clone(), sequence);
            let p = *pattern_of.entry(key.clone()).or_insert_with(|| {
                let n = key.1.len();
                patterns.push(Pattern {
                    label: label.clone(),
                    route_type: route_type.clone(),
                    stops: key.1.clone(),
                    boardings: vec![0; n],
                    drop_off: vec![false; n],
                    ride: vec![(0.0, 0); n - 1],
                });
                patterns.len() - 1
            });
            let pattern = &mut patterns[p];
            for (k, s) in stops_of_trip.iter().enumerate() {
                pattern.drop_off[k] |= s.drop_off;
                if !in_window(s.departure) {
                    continue;
                }
                if s.pickup && k + 1 < stops_of_trip.len() {
                    pattern.boardings[k] += 1;
                }
                if let Some(next) = stops_of_trip.get(k + 1) {
                    // Arrival to arrival, so riders staying on pay the dwell.
                    let from = if k == 0 { s.departure } else { s.arrival };
                    let hop = &mut pattern.ride[k];
                    hop.0 += next.arrival.saturating_sub(from) as f64;
                    hop.1 += 1;
                }
            }
        }

        // Explicit transfers between stops.
        let mut transfers = Vec::new();
        if let Some(bytes) = feed.read("transfers.txt")? {
            let table = Table::new("transfers.txt", bytes)?;
            let (from, to, kind, min_time) = (
                table.column("from_stop_id")?,
                table.column("to_stop_id")?,
                table.optional("transfer_type"),
                table.optional("min_transfer_time"),
            );
            table.each(|r| {
                let (Some(&a), Some(&b)) = (
                    stop_index.get(field(r, Some(from))),
                    stop_index.get(field(r, Some(to))),
                ) else {
                    return Ok(());
                };
                let seconds = match field(r, kind) {
                    "" | "0" | "1" => 0.0,
                    "2" => field(r, min_time).parse().unwrap_or(0.0),
                    _ => return Ok(()), // not possible, or in-seat transfers
                };
                if a != b {
                    transfers.push((a, b, seconds));
                }
                Ok(())
            })?;
        }
        Ok(Service {
            stops,
            patterns,
            transfers,
        })
    }
}

/// `service_id`s running on `date`, from `calendar.txt` and `calendar_dates.txt`.
fn active_services(feed: &FeedSource, date: u32) -> Result<HashSet<String>, OsmGraphError> {
    const DAYS: [&str; 7] = [
        "monday",
        "tuesday",
        "wednesday",
        "thursday",
        "friday",
        "saturday",
        "sunday",
    ];
    let mut active = HashSet::new();
    let calendar = feed.read("calendar.txt")?;
    let dates = feed.read("calendar_dates.txt")?;
    if calendar.is_none() && dates.is_none() {
        return Err(OsmGraphError::InvalidInput(
            "GTFS feed has neither calendar.txt nor calendar_dates.txt".into(),
        ));
    }
    if let Some(bytes) = calendar {
        let table = Table::new("calendar.txt", bytes)?;
        let (id, start, end, day) = (
            table.column("service_id")?,
            table.column("start_date")?,
            table.column("end_date")?,
            table.column(DAYS[weekday(date)])?,
        );
        table.each(|r| {
            let start: u32 = field(r, Some(start)).parse().unwrap_or(u32::MAX);
            let end: u32 = field(r, Some(end)).parse().unwrap_or(0);
            if field(r, Some(day)) == "1" && start <= date && date <= end {
                active.insert(field(r, Some(id)).to_owned());
            }
            Ok(())
        })?;
    }
    if let Some(bytes) = dates {
        let table = Table::new("calendar_dates.txt", bytes)?;
        let (id, on, kind) = (
            table.column("service_id")?,
            table.column("date")?,
            table.column("exception_type")?,
        );
        table.each(|r| {
            if field(r, Some(on)).parse::<u32>().ok() == Some(date) {
                let service = field(r, Some(id)).to_owned();
                match field(r, Some(kind)) {
                    "1" => {
                        active.insert(service);
                    }
                    "2" => {
                        active.remove(&service);
                    }
                    _ => {}
                }
            }
            Ok(())
        })?;
    }
    Ok(active)
}

// ---------------------------------------------------------------------------
// Building the combined graph
// ---------------------------------------------------------------------------

/// The tag that marks transit nodes and edges (`graphways:transit=stop`,
/// `ride`, ...).
/// Namespaced so no OSM tag can collide with it: street nodes keep all their
/// OSM tags, and a node mistaken for transit is dropped from snapping and
/// isochrones.
pub(crate) const TRANSIT_TAG: &str = "graphways:transit";

/// Whether a node or edge belongs to the transit layer (not the streets).
pub(crate) fn is_transit(tags: &[OsmTag]) -> bool {
    tags.iter().any(|t| t.key == TRANSIT_TAG)
}

fn tags(pairs: &[(&str, &str)]) -> Arc<[OsmTag]> {
    pairs
        .iter()
        .map(|&(k, v)| OsmTag {
            key: k.to_owned(),
            value: v.to_owned(),
        })
        .collect::<Vec<_>>()
        .into()
}

/// An edge that only walking (and transit) uses: `seconds` for walking,
/// impassable for cycling and driving.
fn transit_edge(tags: Arc<[OsmTag]>, length: f64, seconds: f64) -> Edge {
    Edge {
        way_id: 0,
        last_way_id: 0,
        tags,
        length,
        speed_kph: 0.0,
        walk_travel_time: seconds,
        bike_travel_time: f64::INFINITY,
        drive_travel_time: f64::INFINITY,
        signal_delay_s: 0.0,
        geometry: Vec::new(),
    }
}

/// What [`SpatialGraph::with_transit`] added.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TransitSummary {
    /// Stops linked to the street network.
    pub stops: usize,
    /// Distinct stop sequences with service in the window.
    pub patterns: usize,
    /// Stops left out for being too far from any walkable street.
    pub unlinked_stops: usize,
}

impl SpatialGraph {
    /// Add public transport from a GTFS feed (`.zip` or directory) to this
    /// walking graph, modelling the service in `options`' window by its
    /// frequencies (see the [module docs](crate::transit)).
    ///
    /// Every query on the returned graph may ride transit: routes, matrices,
    /// isochrones and accessibility all work unchanged. Points still snap
    /// only to streets. Routing preparation is not carried over.
    pub fn with_transit(
        &self,
        gtfs: impl AsRef<Path>,
        options: &TransitOptions,
    ) -> Result<(SpatialGraph, TransitSummary), OsmGraphError> {
        if self.network_type() != NetworkType::Walk {
            return Err(OsmGraphError::InvalidInput(
                "transit is added to walking graphs (network type Walk)".into(),
            ));
        }
        if !(options.wait_factor.is_finite() && options.wait_factor >= 0.0) {
            return Err(OsmGraphError::InvalidInput(
                "wait_factor must be a non-negative number".into(),
            ));
        }
        if options.max_link_m.is_nan() || options.max_link_m < 0.0 {
            return Err(OsmGraphError::InvalidInput(
                "max_link_m must be a non-negative number".into(),
            ));
        }
        let service = Service::read(gtfs.as_ref(), options)?;
        let mut graph: RoadGraph = (*self.graph).clone();
        let walk_mps = walk_speed_mps(&graph);
        let mut summary = TransitSummary::default();
        // New nodes count down from below every id in use, so a graph that
        // already carries transit (or negative OSM ids) keeps ids unique.
        let lowest = graph.node_weights().map(|n| n.id).min().unwrap_or(0);
        let mut next_id = lowest.min(0) - 1;
        let mut new_node = |graph: &mut RoadGraph, lat: f64, lon: f64, tags: Vec<OsmTag>| {
            let id = next_id;
            next_id -= 1;
            graph.add_node(OsmNode { id, lat, lon, tags })
        };

        // Stops, linked both ways to the street they snap to.
        let access = tags(&[(TRANSIT_TAG, "access")]);
        let mut stop_nodes: Vec<Option<NodeIndex>> = vec![None; service.stops.len()];
        let used: HashSet<u32> = service
            .patterns
            .iter()
            .flat_map(|p| p.stops.iter().copied())
            .collect();
        for (i, stop) in service.stops.iter().enumerate() {
            if !used.contains(&(i as u32)) {
                continue;
            }
            let Some(snap) = self.snap_point((stop.lat, stop.lon)) else {
                summary.unlinked_stops += 1;
                continue;
            };
            if snap.distance_m > options.max_link_m {
                summary.unlinked_stops += 1;
                continue;
            }
            let node_tags = vec![
                OsmTag {
                    key: TRANSIT_TAG.into(),
                    value: "stop".into(),
                },
                OsmTag {
                    key: "gtfs:stop_id".into(),
                    value: stop.id.clone(),
                },
                OsmTag {
                    key: "name".into(),
                    value: stop.name.clone(),
                },
            ];
            let node = new_node(&mut graph, stop.lat, stop.lon, node_tags);
            stop_nodes[i] = Some(node);
            summary.stops += 1;
            let links: Vec<(NodeIndex, f64)> = match snap.edge {
                Some(edge) => {
                    let (u, v) = self
                        .graph
                        .edge_endpoints(edge)
                        .expect("snapped edge exists");
                    let along = self.graph[edge].length;
                    vec![
                        (u, snap.distance_m + snap.fraction * along),
                        (v, snap.distance_m + (1.0 - snap.fraction) * along),
                    ]
                }
                None => vec![(snap.node_index, snap.distance_m)],
            };
            for (street, metres) in links {
                let seconds = metres / walk_mps;
                graph.add_edge(node, street, transit_edge(access.clone(), metres, seconds));
                graph.add_edge(street, node, transit_edge(access.clone(), metres, seconds));
            }
        }

        // Patterns: a node per stop served, ride edges between them, and
        // board/alight edges to the stops.
        let window = (options.end_s - options.start_s) as f64;
        let alight = tags(&[(TRANSIT_TAG, "alight")]);
        // Variants of one line (short turns, branches) that leave a stop for
        // the same next stop share their departures: a passenger takes
        // whichever comes first, so the wait follows their combined headway.
        // (Validated against r5 on Munich: halves the bias of per-variant
        // waits; pooling different lines as well over-corrects.)
        let mut departures: HashMap<(u32, Option<u32>, &str), u32> = HashMap::new();
        for p in &service.patterns {
            for (k, &s) in p.stops.iter().enumerate() {
                let next = p.stops.get(k + 1).copied();
                *departures.entry((s, next, p.label.as_str())).or_default() += p.boardings[k];
            }
        }
        for pattern in &service.patterns {
            let ride = tags(&[
                (TRANSIT_TAG, "ride"),
                ("route", &pattern.label),
                ("gtfs:route_type", &pattern.route_type),
            ]);
            let board = tags(&[(TRANSIT_TAG, "board"), ("route", &pattern.label)]);
            let mut previous: Option<(NodeIndex, usize)> = None;
            let mut added = false;
            for (k, &stop) in pattern.stops.iter().enumerate() {
                // A stop left out is ridden through: the next ride edge
                // spans the hops on both sides of it.
                let Some(stop_node) = stop_nodes[stop as usize] else {
                    continue;
                };
                let at = &service.stops[stop as usize];
                let on_board = new_node(
                    &mut graph,
                    at.lat,
                    at.lon,
                    vec![
                        OsmTag {
                            key: TRANSIT_TAG.into(),
                            value: "platform".into(),
                        },
                        OsmTag {
                            key: "route".into(),
                            value: pattern.label.clone(),
                        },
                    ],
                );
                if pattern.boardings[k] > 0 {
                    let next = pattern.stops.get(k + 1).copied();
                    let shared = departures[&(stop, next, pattern.label.as_str())];
                    let headway = window / shared as f64;
                    let wait = options.wait_factor * headway;
                    graph.add_edge(stop_node, on_board, transit_edge(board.clone(), 0.0, wait));
                }
                if k > 0 && pattern.drop_off[k] {
                    graph.add_edge(on_board, stop_node, transit_edge(alight.clone(), 0.0, 0.0));
                }
                if let Some((from, j)) = previous {
                    // Running time over the hops since the last kept stop.
                    let hops = &pattern.ride[j..k];
                    if hops.iter().all(|&(_, n)| n > 0) {
                        let seconds: f64 = hops.iter().map(|&(t, n)| t / n as f64).sum();
                        let from_stop = &service.stops[pattern.stops[j] as usize];
                        let metres = LatLon::new(from_stop.lat, from_stop.lon)
                            .distance_m(LatLon::new(at.lat, at.lon));
                        graph.add_edge(from, on_board, transit_edge(ride.clone(), metres, seconds));
                        added = true;
                    }
                }
                previous = Some((on_board, k));
            }
            summary.patterns += usize::from(added);
        }

        // GTFS transfers between stops.
        let transfer = tags(&[(TRANSIT_TAG, "transfer")]);
        for &(a, b, seconds) in &service.transfers {
            if let (Some(from), Some(to)) = (stop_nodes[a as usize], stop_nodes[b as usize]) {
                let (sa, sb) = (&service.stops[a as usize], &service.stops[b as usize]);
                let metres = LatLon::new(sa.lat, sa.lon).distance_m(LatLon::new(sb.lat, sb.lon));
                // A transfer is walked: `min_transfer_time` can add to the
                // walk, never shorten it (types 0/1 give no time at all).
                let seconds = seconds.max(metres / walk_mps);
                graph.add_edge(from, to, transit_edge(transfer.clone(), metres, seconds));
            }
        }

        let mut combined = SpatialGraph::new(graph, NetworkType::Walk);
        combined.poi_snaps = self.poi_snaps.clone();
        Ok((combined, summary))
    }
}

/// The walking speed the graph was built with, from any street edge.
fn walk_speed_mps(graph: &RoadGraph) -> f64 {
    graph
        .edge_weights()
        .filter(|e| e.length > 1.0 && e.walk_travel_time.is_finite() && !is_transit(&e.tags))
        .map(|e| e.length / e.walk_travel_time)
        .next()
        .unwrap_or(5.0 / 3.6)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 5.5 km straight street along the equator, walked in about 66 min.
    fn long_street() -> SpatialGraph {
        let nodes: String = (0..=10)
            .map(|i| {
                format!(
                    r#"<node id="{}" lat="0" lon="{}"/>"#,
                    i + 1,
                    i as f64 * 0.005
                )
            })
            .collect();
        let refs: String = (0..=10)
            .map(|i| format!(r#"<nd ref="{}"/>"#, i + 1))
            .collect();
        let xml =
            format!(r#"<osm>{nodes}<way id="1">{refs}<tag k="highway" v="footway"/></way></osm>"#);
        SpatialGraph::from_osm(&xml, NetworkType::Walk, true).unwrap()
    }

    /// Line X from stop W (west end) to stop E (east end), every 10 min
    /// from 07:00 to 09:00 on weekdays, 8 min end to end.
    fn feed_files() -> Vec<(&'static str, String)> {
        let mut stop_times =
            String::from("trip_id,arrival_time,departure_time,stop_id,stop_sequence\n");
        let mut trips = String::from("route_id,service_id,trip_id\n");
        for i in 0..12 {
            let (h, m) = (7 + i / 6, (i % 6) * 10);
            trips += &format!("x,weekdays,t{i}\n");
            stop_times += &format!("t{i},{h:02}:{m:02}:00,{h:02}:{m:02}:00,W,1\n");
            stop_times += &format!("t{i},{h:02}:{:02}:00,{h:02}:{:02}:00,E,2\n", m + 8, m + 8);
        }
        vec![
            ("stops.txt", "\u{feff}stop_id,stop_name,stop_lat,stop_lon\nW,West,0.0001,0.0\nE,East,0.0001,0.05\n".into()),
            ("routes.txt", "route_id,route_short_name,route_type\nx,X,3\n".into()),
            ("trips.txt", trips),
            ("stop_times.txt", stop_times),
            (
                "calendar.txt",
                "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date\n\
                 weekdays,1,1,1,1,1,0,0,20260101,20261231\n"
                    .into(),
            ),
            ("calendar_dates.txt", "service_id,date,exception_type\nweekdays,20261007,2\n".into()),
        ]
    }

    fn write_feed(dir: &Path) {
        std::fs::create_dir_all(dir).unwrap();
        for (name, body) in feed_files() {
            std::fs::write(dir.join(name), body).unwrap();
        }
    }

    fn temp(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("graphways-transit-{}-{name}", std::process::id()))
    }

    fn end_to_end(sg: &SpatialGraph) -> f64 {
        sg.route((0.0, 0.0), (0.0, 0.05), None).unwrap().duration_s
    }

    #[test]
    fn frequent_service_beats_walking_by_the_expected_amount() {
        let walk = long_street();
        let dir = temp("dir");
        write_feed(&dir);
        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        let (sg, summary) = walk.with_transit(&dir, &options).unwrap();
        assert_eq!(
            summary,
            TransitSummary {
                stops: 2,
                patterns: 1,
                unlinked_stops: 0
            }
        );

        // Walk ~11 m to the stop, wait half of 10 min, ride 8 min, walk ~11 m.
        let walking = end_to_end(&walk);
        let by_transit = end_to_end(&sg);
        assert!(walking > 3600.0, "{walking}");
        let expected = 300.0 + 480.0 + 2.0 * 11.1 / (5.0 / 3.6);
        assert!(
            (by_transit - expected).abs() < 2.0,
            "{by_transit} vs {expected}"
        );
        // No service westbound: the way back is on foot.
        let back = sg.route((0.0, 0.05), (0.0, 0.0), None).unwrap().duration_s;
        assert!((back - walking).abs() < 1.0);

        // Every search agrees, prepared or not.
        sg.prepare_routing();
        assert!((end_to_end(&sg) - by_transit).abs() < 1e-6);
        let matrix = sg.travel_time_matrix(&[(0.0, 0.0)], &[(0.0, 0.05)], None);
        assert!((matrix.durations_s[0][0].unwrap() - by_transit).abs() < 1e-6);

        // Points never snap onto the transit layer, and isochrones grow
        // around the far stop.
        let snap = sg.snap_point((0.0001, 0.05)).unwrap();
        assert!(snap.node_id > 0, "snapped to a street node");
        use geo::Area;
        let area = |g: &SpatialGraph| {
            g.isochrones((0.0, 0.0), &[1200.0], None).unwrap()[0].unsigned_area()
        };
        assert!(area(&sg) >= area(&walk));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn variants_of_a_line_share_their_departures() {
        // A second variant of line X runs on past E to F, five minutes after
        // each regular trip: from W both head for E, so waits halve.
        let walk = long_street();
        let dir = temp("variants");
        write_feed(&dir);
        let mut trips = std::fs::read_to_string(dir.join("trips.txt")).unwrap();
        let mut times = std::fs::read_to_string(dir.join("stop_times.txt")).unwrap();
        let clock = |s: u32| format!("{:02}:{:02}:00", s / 3600, s / 60 % 60);
        for i in 0..12 {
            let depart = 7 * 3600 + i * 600 + 300;
            trips += &format!("x,weekdays,v{i}\n");
            for (stop, after, seq) in [("W", 0, 1), ("E", 480, 2), ("F", 540, 3)] {
                let t = clock(depart + after);
                times += &format!("v{i},{t},{t},{stop},{seq}\n");
            }
        }
        std::fs::write(dir.join("trips.txt"), trips).unwrap();
        std::fs::write(dir.join("stop_times.txt"), times).unwrap();
        let mut stops = std::fs::read_to_string(dir.join("stops.txt")).unwrap();
        stops += "F,Far east,0.0001,0.0499\n";
        std::fs::write(dir.join("stops.txt"), stops).unwrap();

        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        let (sg, summary) = walk.with_transit(&dir, &options).unwrap();
        assert_eq!(summary.patterns, 2);
        let expected = 150.0 + 480.0 + 2.0 * 11.1 / (5.0 / 3.6);
        assert!(
            (end_to_end(&sg) - expected).abs() < 2.0,
            "{}",
            end_to_end(&sg)
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn windows_and_calendars_decide_what_runs() {
        let walk = long_street();
        let dir = temp("calendar");
        write_feed(&dir);
        let walking = end_to_end(&walk);
        for (date, start, end) in [
            ("2026-10-10", "07:00", "09:00"), // Saturday
            ("2026-10-07", "07:00", "09:00"), // removed by calendar_dates
            ("2026-10-06", "12:00", "13:00"), // no trips in the window
        ] {
            let options = TransitOptions::new(date, start, end).unwrap();
            let (sg, summary) = walk.with_transit(&dir, &options).unwrap();
            assert_eq!(summary.patterns, 0, "{date} {start}");
            assert!((end_to_end(&sg) - walking).abs() < 1e-6);
        }
        // A sparser window means a longer expected wait.
        let options = TransitOptions::new("2026-10-06", "08:00", "10:00").unwrap();
        let (sg, _) = walk.with_transit(&dir, &options).unwrap();
        let expected = 0.5 * 7200.0 / 6.0 + 480.0 + 2.0 * 11.1 / (5.0 / 3.6);
        assert!((end_to_end(&sg) - expected).abs() < 2.0);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn zipped_feeds_and_bad_input() {
        let walk = long_street();
        let path = temp("feed.zip");
        {
            let mut zip = zip::ZipWriter::new(File::create(&path).unwrap());
            let stored = zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Stored);
            for (name, body) in feed_files() {
                zip.start_file(format!("feed/{name}"), stored).unwrap();
                std::io::Write::write_all(&mut zip, body.as_bytes()).unwrap();
            }
            zip.finish().unwrap();
        }
        let options = TransitOptions::new("20261006", "07:00", "09:00").unwrap();
        let (sg, summary) = walk.with_transit(&path, &options).unwrap();
        assert_eq!(summary.patterns, 1);
        assert!(end_to_end(&sg) < 900.0);
        std::fs::remove_file(&path).ok();

        let not_a_feed = temp("empty");
        std::fs::create_dir_all(&not_a_feed).unwrap();
        assert!(matches!(
            walk.with_transit(&not_a_feed, &options),
            Err(OsmGraphError::InvalidInput(_))
        ));
        std::fs::remove_dir_all(&not_a_feed).ok();
        let drive = SpatialGraph::new((*walk.graph).clone(), NetworkType::Drive);
        assert!(drive.with_transit(&path, &options).is_err());
    }

    #[test]
    fn transfers_take_at_least_the_walk() {
        // A "timed" transfer (type 0) from W to E, 5.5 km apart: it must not
        // move anyone there faster than walking.
        let walk = long_street();
        let dir = temp("transfers");
        write_feed(&dir);
        std::fs::write(
            dir.join("transfers.txt"),
            "from_stop_id,to_stop_id,transfer_type\nW,E,0\nE,W,2\n",
        )
        .unwrap();
        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        let (sg, _) = walk.with_transit(&dir, &options).unwrap();
        let expected = 300.0 + 480.0 + 2.0 * 11.1 / (5.0 / 3.6);
        assert!(
            (end_to_end(&sg) - expected).abs() < 2.0,
            "{}",
            end_to_end(&sg)
        );
        let back = sg.route((0.0, 0.05), (0.0, 0.0), None).unwrap().duration_s;
        assert!(back > 3600.0, "{back}");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn osm_transit_tags_do_not_hide_street_nodes() {
        // A real OSM node tagged `transit=*` is still a street node.
        let xml = r#"<osm>
            <node id="1" lat="0" lon="0"><tag k="transit" v="yes"/></node>
            <node id="2" lat="0" lon="0.001"/>
            <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="footway"/></way>
        </osm>"#;
        let sg = SpatialGraph::from_osm(xml, NetworkType::Walk, true).unwrap();
        let node = sg.node_index(1).unwrap();
        assert!(sg.is_street_node(node));
        assert_eq!(sg.snap_point((0.0, -0.0001)).unwrap().node_id, 1);
    }

    #[test]
    fn reachable_and_prism_views_report_only_streets() {
        let walk = long_street();
        let dir = temp("views");
        write_feed(&dir);
        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        let (sg, _) = walk.with_transit(&dir, &options).unwrap();
        // Street nodes keep their OSM ids; transit ones count down from -1.
        let street = |node: NodeIndex| sg.graph[node].id > 0;

        // Fifteen minutes reaches both ends of the street (riding to the
        // far one), passing through stops and platforms on the way.
        let reach = sg.reachable_graph((0.0, 0.0), 900.0, None).unwrap();
        let reached = |node: NodeIndex| street(node) && reach.result.times.contains_key(node);
        let streets = reach.result.times.keys().filter(|&n| street(n)).count();
        assert!(streets < reach.result.times.len(), "transit was used");
        assert_eq!(reach.node_count(), streets);
        let street_edges = sg
            .graph
            .edge_indices()
            .filter(|&e| {
                let (a, b) = sg.graph.edge_endpoints(e).unwrap();
                reached(a) && reached(b)
            })
            .count();
        assert_eq!(reach.edge_count(), street_edges);

        let prism = sg.prism((0.0, 0.0), (0.0, 0.05), 900.0, None).unwrap();
        let in_prism = prism.result.feasible.keys().filter(|&n| street(n)).count();
        assert!(in_prism < prism.result.feasible.len(), "transit was used");
        assert_eq!(prism.node_count(), in_prism);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn adding_transit_twice_keeps_node_ids_unique() {
        let walk = long_street();
        let dir = temp("twice");
        write_feed(&dir);
        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        let (once, _) = walk.with_transit(&dir, &options).unwrap();
        let (twice, _) = once.with_transit(&dir, &options).unwrap();
        let mut ids: Vec<i64> = twice.graph.node_weights().map(|n| n.id).collect();
        let total = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), total);
        assert!(total > once.graph.node_count());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn link_distance_must_be_a_number() {
        let walk = long_street();
        let dir = temp("link");
        write_feed(&dir);
        for bad in [f64::NAN, -1.0] {
            let mut options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
            options.max_link_m = bad;
            assert!(matches!(
                walk.with_transit(&dir, &options),
                Err(OsmGraphError::InvalidInput(_))
            ));
        }
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn dates_times_and_weekdays() {
        assert_eq!(parse_date("20261006"), Some(20261006));
        assert_eq!(parse_date("20260230"), None);
        assert_eq!(parse_date("20240229"), Some(20240229));
        assert_eq!(parse_time("07:05"), Some(7 * 3600 + 300));
        assert_eq!(parse_time("25:00:30"), Some(25 * 3600 + 30));
        assert_eq!(parse_time("7:61"), None);
        assert_eq!(weekday(20261006), 1, "6 Oct 2026 is a Tuesday");
        assert_eq!(weekday(20240101), 0, "1 Jan 2024 was a Monday");
        assert_eq!(weekday(20000227), 6, "27 Feb 2000 was a Sunday");
        let options = TransitOptions::new("2026-10-06", "07:00", "09:00").unwrap();
        assert_eq!(
            (options.date, options.start_s, options.end_s),
            (20261006, 25200, 32400)
        );
        assert!(TransitOptions::new("2026-10-06", "09:00", "07:00").is_err());
    }
}
