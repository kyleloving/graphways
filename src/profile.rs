//! Speed profiles and graph-building options.
//!
//! The defaults reproduce the library's long-standing assumptions (5 km/h
//! walking, 15 km/h cycling, a typical urban speed per road class), but every
//! number can be tuned for a region or a population, e.g. slower walking for
//! an older-adult accessibility study or lower urban speeds in a 30 km/h city.

use std::collections::HashMap;

use crate::error::OsmGraphError;

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
    /// Seconds added to the driving time for passing a traffic signal
    /// (`highway=traffic_signals`, honouring `traffic_signals:direction`).
    pub traffic_signal_s: f64,
    /// Time lost turning at junctions when driving.
    pub turn_costs: TurnCosts,
}

/// Time lost turning at a junction when driving, modelled on OSRM's car
/// profile so travel times are comparable.
///
/// A turn through angle `a` (degrees, 0 = straight on, positive = right)
/// costs `turn_penalty_s / (1 + exp(-(13 / bias * a / 180 - 6.5 * bias)))`
/// for right turns and the mirror image, with `1 / bias`, for left turns,
/// where `bias` favours turns away from oncoming traffic: nearly free
/// straight on, about 2 s for a right and 5 s for a left turn with the
/// defaults. U-turns add `u_turn_penalty_s`. Turns are priced only at real
/// junctions (three or more roads) and for U-turns. All zeros disables it.
#[derive(Debug, Clone, PartialEq)]
pub struct TurnCosts {
    pub turn_penalty_s: f64,
    /// Above 1 makes turns across oncoming traffic dearer.
    pub turn_bias: f64,
    pub u_turn_penalty_s: f64,
    /// Traffic drives on the left (UK, Japan, ...): left turns are the cheap ones.
    pub left_hand_traffic: bool,
}

impl TurnCosts {
    /// No turn costs at all.
    pub fn none() -> Self {
        TurnCosts {
            turn_penalty_s: 0.0,
            turn_bias: 1.0,
            u_turn_penalty_s: 0.0,
            left_hand_traffic: false,
        }
    }

    /// Whether any turn costs time.
    pub fn is_none(&self) -> bool {
        self.turn_penalty_s <= 0.0 && self.u_turn_penalty_s <= 0.0
    }

    /// Seconds for a turn through `angle_deg` (0 = straight, positive =
    /// right, within ±180); `u_turn` adds the U-turn penalty.
    pub fn cost(&self, angle_deg: f64, u_turn: bool) -> f64 {
        let bias = if self.left_hand_traffic {
            1.0 / self.turn_bias
        } else {
            self.turn_bias
        };
        let share = angle_deg.abs().min(180.0) / 180.0;
        let exponent = if angle_deg >= 0.0 {
            13.0 / bias * share - 6.5 * bias
        } else {
            13.0 * bias * share - 6.5 / bias
        };
        let turn = self.turn_penalty_s / (1.0 + (-exponent).exp());
        turn + if u_turn { self.u_turn_penalty_s } else { 0.0 }
    }
}

impl Default for TurnCosts {
    fn default() -> Self {
        TurnCosts {
            turn_penalty_s: 7.5,
            turn_bias: 1.075,
            u_turn_penalty_s: 20.0,
            left_hand_traffic: false,
        }
    }
}

impl Profile {
    /// Check that every number makes sense: speeds positive and finite,
    /// distances and delays non-negative and finite, `turn_bias` positive.
    /// Graph constructors call this, so a bad profile is an error rather
    /// than a graph on which nothing is reachable.
    pub fn validate(&self) -> Result<(), OsmGraphError> {
        let positive = |name: &str, value: f64| {
            if value.is_finite() && value > 0.0 {
                Ok(())
            } else {
                Err(OsmGraphError::InvalidInput(format!(
                    "{name} must be a positive number, got {value}"
                )))
            }
        };
        let non_negative = |name: &str, value: f64| {
            if value.is_finite() && value >= 0.0 {
                Ok(())
            } else {
                Err(OsmGraphError::InvalidInput(format!(
                    "{name} must be a non-negative number, got {value}"
                )))
            }
        };
        positive("walk_speed_kph", self.walk_speed_kph)?;
        positive("bike_speed_kph", self.bike_speed_kph)?;
        positive("default_drive_speed_kph", self.default_drive_speed_kph)?;
        for (class, &kph) in &self.drive_speeds_kph {
            positive(&format!("drive_speeds_kph['{class}']"), kph)?;
        }
        non_negative("merge_distance_m", self.merge_distance_m)?;
        non_negative("traffic_signal_s", self.traffic_signal_s)?;
        non_negative("turn_penalty_s", self.turn_costs.turn_penalty_s)?;
        positive("turn_bias", self.turn_costs.turn_bias)?;
        non_negative("u_turn_penalty_s", self.turn_costs.u_turn_penalty_s)
    }

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
            traffic_signal_s: 2.0,
            turn_costs: TurnCosts::default(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn turn_costs_match_osrm_car_profile() {
        let costs = TurnCosts::default();
        assert!(costs.cost(0.0, false) < 0.01, "straight on is nearly free");
        let (right, left) = (costs.cost(90.0, false), costs.cost(-90.0, false));
        assert!((right - 2.1).abs() < 0.05, "{right}");
        assert!((left - 5.4).abs() < 0.05, "{left}");
        assert!((costs.cost(180.0, true) - 27.5).abs() < 0.1);
        let uk = TurnCosts {
            left_hand_traffic: true,
            ..TurnCosts::default()
        };
        assert!((uk.cost(-90.0, false) - right).abs() < 1e-9, "mirror image");
        assert_eq!(TurnCosts::none().cost(-120.0, true), 0.0);
    }

    #[test]
    fn validation_rejects_nonsense_numbers() {
        assert!(Profile::default().validate().is_ok());
        let none = Profile {
            turn_costs: TurnCosts::none(),
            traffic_signal_s: 0.0,
            merge_distance_m: 0.0,
            ..Profile::default()
        };
        assert!(none.validate().is_ok(), "zero delays and merging are fine");
        let bad = [
            Profile {
                walk_speed_kph: 0.0,
                ..Profile::default()
            },
            Profile {
                bike_speed_kph: f64::NAN,
                ..Profile::default()
            },
            Profile::default().with_drive_speed("residential", -30.0),
            Profile {
                traffic_signal_s: -1.0,
                ..Profile::default()
            },
            Profile {
                merge_distance_m: f64::INFINITY,
                ..Profile::default()
            },
            Profile {
                turn_costs: TurnCosts {
                    turn_bias: 0.0,
                    ..TurnCosts::default()
                },
                ..Profile::default()
            },
        ];
        for profile in bad {
            assert!(
                matches!(profile.validate(), Err(OsmGraphError::InvalidInput(_))),
                "{profile:?}"
            );
        }
    }
}
