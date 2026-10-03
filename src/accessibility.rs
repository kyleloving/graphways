//! Accessibility measures: how many opportunities (jobs, schools, clinics,
//! shops...) each origin can reach, and which destinations are closest.
//!
//! Both are built on the travel-time matrix machinery but reduce each
//! origin's row as soon as it is computed, so memory stays proportional to
//! the number of points rather than to origins x destinations: scoring 100k
//! homes against 100k jobs never holds a 10-billion-cell table.

use petgraph::graph::EdgeIndex;

use crate::error::OsmGraphError;
use crate::graph::{LatLon, Pricing, SpatialGraph};
use crate::matrix::{Costs, Table};

/// How an opportunity's contribution falls off with travel time `t` (seconds).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Decay {
    /// Counts fully within `cutoff_s`, not at all beyond (cumulative
    /// opportunities).
    Step { cutoff_s: f64 },
    /// Falls linearly from 1 at `t = 0` to 0 at `cutoff_s`.
    Linear { cutoff_s: f64 },
    /// Halves every `half_life_s`: `0.5^(t / half_life_s)`.
    Exponential { half_life_s: f64 },
    /// `exp(-t² / (2 sigma_s²))`.
    Gaussian { sigma_s: f64 },
}

impl Decay {
    /// The weight of an opportunity `t` seconds away, between 0 and 1.
    pub fn weight(&self, t: f64) -> f64 {
        match *self {
            Decay::Step { cutoff_s } => f64::from(u8::from(t <= cutoff_s)),
            Decay::Linear { cutoff_s } => (1.0 - t / cutoff_s).max(0.0),
            Decay::Exponential { half_life_s } => 0.5_f64.powf(t / half_life_s),
            Decay::Gaussian { sigma_s } => (-(t * t) / (2.0 * sigma_s * sigma_s)).exp(),
        }
    }

    /// Its parameter in seconds.
    fn parameter(&self) -> f64 {
        match *self {
            Decay::Step { cutoff_s } | Decay::Linear { cutoff_s } => cutoff_s,
            Decay::Exponential { half_life_s } => half_life_s,
            Decay::Gaussian { sigma_s } => sigma_s,
        }
    }
}

/// One of an origin's closest destinations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NearbyDestination {
    /// Position of the destination in the list passed in.
    pub index: usize,
    pub duration_s: f64,
    /// Length of the fastest route, in metres.
    pub distance_m: f64,
}

impl SpatialGraph {
    /// Accessibility score of every origin for each decay function: the sum
    /// over opportunities of `weight × decay(travel time)`, with every
    /// weight 1 when `weights` is `None` (so a [`Decay::Step`] counts the
    /// opportunities reachable within its cutoff).
    ///
    /// `scores[i][k]` is origin `i`'s score under `decays[k]`; an origin
    /// farther than `max_snap_m` from every road scores `None`, and
    /// opportunities that cannot be snapped or reached contribute nothing.
    /// Runs one search per origin and keeps no table in memory; call
    /// [`SpatialGraph::prepare_routing`] first for more than a handful of
    /// origins.
    pub fn accessibility<O, D>(
        &self,
        origins: &[O],
        opportunities: &[D],
        weights: Option<&[f64]>,
        decays: &[Decay],
        max_snap_m: Option<f64>,
    ) -> Result<Vec<Option<Vec<f64>>>, OsmGraphError>
    where
        O: Into<LatLon> + Copy,
        D: Into<LatLon> + Copy,
    {
        if let Some(weights) = weights {
            if weights.len() != opportunities.len() {
                return Err(OsmGraphError::InvalidInput(format!(
                    "{} weights for {} opportunities",
                    weights.len(),
                    opportunities.len()
                )));
            }
            if weights.iter().any(|w| !w.is_finite()) {
                return Err(OsmGraphError::InvalidInput("weights must be finite".into()));
            }
        }
        if let Some(bad) = decays
            .iter()
            .find(|d| !(d.parameter().is_finite() && d.parameter() > 0.0))
        {
            return Err(OsmGraphError::InvalidInput(format!(
                "decay parameters must be positive and finite: {bad:?}"
            )));
        }
        let table = self.table(origins, opportunities, max_snap_m);
        Ok(table.rows(Costs::Native, true, |i, row| {
            table.origin_snaps[i].as_ref()?;
            let mut scores = vec![0.0; decays.len()];
            for (j, &(t, _)) in row.iter().enumerate() {
                if !t.is_finite() {
                    continue;
                }
                let weight = weights.map_or(1.0, |w| w[j]);
                for (score, decay) in scores.iter_mut().zip(decays) {
                    *score += weight * decay.weight(t);
                }
            }
            Some(scores)
        }))
    }

    /// The `k` destinations each origin reaches fastest, nearest first
    /// (fewer when fewer are reachable; none for an unsnapped origin).
    /// Like [`SpatialGraph::accessibility`], it never holds the full table.
    pub fn nearest_destinations<O, D>(
        &self,
        origins: &[O],
        destinations: &[D],
        k: usize,
        max_snap_m: Option<f64>,
    ) -> Vec<Vec<NearbyDestination>>
    where
        O: Into<LatLon> + Copy,
        D: Into<LatLon> + Copy,
    {
        let table = self.table(origins, destinations, max_snap_m);
        table.rows(Costs::Native, true, |_, row| {
            let mut reached: Vec<NearbyDestination> = row
                .iter()
                .enumerate()
                .filter(|(_, cell)| cell.0.is_finite())
                .map(|(index, &(duration_s, distance_m))| NearbyDestination {
                    index,
                    duration_s,
                    distance_m,
                })
                .collect();
            let by_time = |a: &NearbyDestination, b: &NearbyDestination| {
                a.duration_s
                    .total_cmp(&b.duration_s)
                    .then(a.index.cmp(&b.index))
            };
            if k < reached.len() {
                reached.select_nth_unstable_by(k, by_time);
                reached.truncate(k);
            }
            reached.sort_unstable_by(by_time);
            reached
        })
    }

    fn table<'g, O, D>(
        &'g self,
        origins: &[O],
        destinations: &[D],
        max_snap_m: Option<f64>,
    ) -> Table<'g>
    where
        O: Into<LatLon> + Copy,
        D: Into<LatLon> + Copy,
    {
        let origins: Vec<LatLon> = origins.iter().map(|&p| p.into()).collect();
        let destinations: Vec<LatLon> = destinations.iter().map(|&p| p.into()).collect();
        let nt = self.network_type();
        Table::new(
            self,
            &origins,
            &destinations,
            max_snap_m,
            move |e: EdgeIndex| self.graph[e].travel_time(nt),
            Pricing::Native,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overpass::NetworkType;

    fn sample() -> (SpatialGraph, Vec<(f64, f64)>) {
        let sg =
            SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", NetworkType::Walk, false)
                .unwrap();
        let mut points: Vec<(f64, f64)> = sg.graph.node_weights().map(|n| (n.lat, n.lon)).collect();
        points.push((0.0, 0.0)); // unsnappable with a snap limit
        (sg, points)
    }

    #[test]
    fn decays_have_the_documented_shape() {
        assert_eq!(Decay::Step { cutoff_s: 10.0 }.weight(10.0), 1.0);
        assert_eq!(Decay::Step { cutoff_s: 10.0 }.weight(10.1), 0.0);
        assert_eq!(Decay::Linear { cutoff_s: 10.0 }.weight(5.0), 0.5);
        assert_eq!(Decay::Linear { cutoff_s: 10.0 }.weight(20.0), 0.0);
        assert!((Decay::Exponential { half_life_s: 60.0 }.weight(120.0) - 0.25).abs() < 1e-12);
        assert!((Decay::Gaussian { sigma_s: 60.0 }.weight(60.0) - (-0.5f64).exp()).abs() < 1e-12);
    }

    #[test]
    fn scores_match_the_matrix() {
        let (sg, points) = sample();
        let weights: Vec<f64> = (0..points.len()).map(|i| 1.0 + i as f64).collect();
        let decays = [
            Decay::Step { cutoff_s: 60.0 },
            Decay::Linear { cutoff_s: 120.0 },
            Decay::Exponential { half_life_s: 45.0 },
            Decay::Gaussian { sigma_s: 90.0 },
        ];
        for prepared in [false, true] {
            if prepared {
                sg.prepare_routing();
            }
            let matrix = sg.travel_time_matrix(&points, &points, Some(100.0));
            let scores = sg
                .accessibility(&points, &points, Some(&weights), &decays, Some(100.0))
                .unwrap();
            for (i, score) in scores.iter().enumerate() {
                let Some(score) = score else {
                    assert!(matrix.origin_snaps[i].is_none());
                    continue;
                };
                for (k, decay) in decays.iter().enumerate() {
                    let want: f64 = matrix.durations_s[i]
                        .iter()
                        .zip(&weights)
                        .filter_map(|(t, w)| t.map(|t| w * decay.weight(t)))
                        .sum();
                    assert!(
                        (score[k] - want).abs() < 1e-9,
                        "{i} {decay:?}: {} vs {want}",
                        score[k]
                    );
                }
            }
            assert!(scores.last().unwrap().is_none(), "unsnappable origin");
        }
    }

    #[test]
    fn nearest_destinations_are_the_fastest_k() {
        let (sg, points) = sample();
        let matrix = sg.travel_time_matrix(&points, &points, Some(100.0));
        let nearest = sg.nearest_destinations(&points, &points, 3, Some(100.0));
        for (i, found) in nearest.iter().enumerate() {
            let mut all: Vec<(f64, usize)> = matrix.durations_s[i]
                .iter()
                .enumerate()
                .filter_map(|(j, t)| t.map(|t| (t, j)))
                .collect();
            all.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let want: Vec<usize> = all.iter().take(3).map(|&(_, j)| j).collect();
            let got: Vec<usize> = found.iter().map(|d| d.index).collect();
            assert_eq!(got, want, "origin {i}");
            for d in found {
                assert_eq!(Some(d.distance_m), matrix.distances_m[i][d.index]);
            }
        }
        assert!(nearest.last().unwrap().is_empty());
    }

    #[test]
    fn invalid_inputs_are_errors() {
        let (sg, points) = sample();
        let step = [Decay::Step { cutoff_s: 60.0 }];
        assert!(sg
            .accessibility(&points, &points, Some(&[1.0]), &step, None)
            .is_err());
        let zero = [Decay::Exponential { half_life_s: 0.0 }];
        assert!(sg
            .accessibility(&points, &points, None, &zero, None)
            .is_err());
    }
}
