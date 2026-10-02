//! Speed profiles and graph-building options.
//!
//! The defaults reproduce the library's long-standing assumptions (5 km/h
//! walking, 15 km/h cycling, a typical urban speed per road class), but every
//! number can be tuned for a region or a population, e.g. slower walking for
//! an older-adult accessibility study or lower urban speeds in a 30 km/h city.

use std::collections::HashMap;

/// Travel-speed assumptions used to cost edges when a graph is built.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    /// Walking speed on every footway, path and street, in km/h.
    pub walk_speed_kph: f64,
    /// Cycling speed on every ridable way, in km/h.
    pub bike_speed_kph: f64,
    /// Driving speed by `highway=*` class, in km/h, used when a way has no
    /// usable `maxspeed` tag. Classes missing here use `default_drive_speed_kph`.
    pub drive_speeds_kph: HashMap<String, f64>,
    /// Driving speed for road classes missing from `drive_speeds_kph`.
    pub default_drive_speed_kph: f64,
    /// Whether a way's `maxspeed` tag overrides its road-class speed.
    pub use_maxspeed: bool,
    /// Intersection nodes closer than this (metres) are merged when the
    /// graph is simplified. 0 disables merging.
    pub merge_distance_m: f64,
}

impl Profile {
    /// Driving speed for a way with the given `highway` value.
    pub fn drive_speed_kph(&self, highway: Option<&str>) -> f64 {
        highway
            .and_then(|h| self.drive_speeds_kph.get(h))
            .copied()
            .unwrap_or(self.default_drive_speed_kph)
    }

    /// Set the driving speed for one road class.
    pub fn with_drive_speed(mut self, highway: &str, kph: f64) -> Self {
        self.drive_speeds_kph.insert(highway.to_owned(), kph);
        self
    }
}

impl Default for Profile {
    fn default() -> Self {
        let drive_speeds_kph = [
            ("motorway", 110.0),
            ("motorway_link", 60.0),
            ("trunk", 90.0),
            ("trunk_link", 45.0),
            ("primary", 65.0),
            ("primary_link", 45.0),
            ("secondary", 55.0),
            ("secondary_link", 40.0),
            ("tertiary", 45.0),
            ("tertiary_link", 35.0),
            ("unclassified", 45.0),
            ("residential", 30.0),
            ("living_street", 10.0),
            ("service", 20.0),
            ("track", 20.0),
            ("road", 50.0),
        ]
        .into_iter()
        .map(|(class, kph)| (class.to_owned(), kph))
        .collect();
        Profile {
            walk_speed_kph: 5.0,
            bike_speed_kph: 15.0,
            drive_speeds_kph,
            default_drive_speed_kph: 50.0,
            use_maxspeed: true,
            merge_distance_m: 5.0,
        }
    }
}

/// How to turn OSM data into a road graph.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BuildOptions {
    /// Keep every OSM node instead of simplifying the graph (merging nearby
    /// intersections and collapsing chains of road between junctions).
    pub retain_all: bool,
    pub profile: Profile,
}

impl BuildOptions {
    pub fn retain_all(retain_all: bool) -> Self {
        BuildOptions {
            retain_all,
            ..Self::default()
        }
    }
}
