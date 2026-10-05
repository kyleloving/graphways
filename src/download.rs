//! Building graphs from OpenStreetMap data downloaded on demand
//! (the `network` feature).
//!
//! Road networks come from the Overpass API and place names are resolved
//! with Nominatim. Responses are cached in memory and on disk, so asking for
//! the same area again does not hit the network. Set `GRAPHWAYS_OVERPASS_URL`,
//! `GRAPHWAYS_NOMINATIM_URL` and `GRAPHWAYS_USER_AGENT` to use your own
//! servers or identify your application.

use crate::cache;
use crate::error::OsmGraphError;
use crate::graph::{parse_xml, LatLon, SpatialGraph};
use crate::overpass::{self, NetworkType};
use crate::profile::BuildOptions;

/// The Overpass response for `query`, from the memory or disk cache when
/// possible.
pub(crate) async fn fetch_cached(query: &str) -> Result<String, OsmGraphError> {
    if let Some(xml) = cache::check_xml_cache(query)? {
        return Ok(xml);
    }
    if let Some(xml) = cache::check_disk_xml_cache(query) {
        cache::insert_into_xml_cache(query.to_owned(), xml.clone())?;
        return Ok(xml);
    }
    let xml = overpass::make_request(&overpass::overpass_url(), query).await?;
    cache::write_disk_xml_cache(query, &xml);
    cache::insert_into_xml_cache(query.to_owned(), xml.clone())?;
    Ok(xml)
}

impl SpatialGraph {
    /// Download the road network within `radius_m` metres of `center` (a
    /// square box around it) from Overpass and build a graph.
    ///
    /// Driving networks include turn restrictions. Errors with
    /// [`OsmGraphError::EmptyGraph`] when the area has no matching roads.
    pub async fn from_point(
        center: impl Into<LatLon>,
        radius_m: f64,
        network_type: NetworkType,
        options: &BuildOptions,
    ) -> Result<Self, OsmGraphError> {
        if !(radius_m.is_finite() && radius_m > 0.0) {
            return Err(OsmGraphError::InvalidInput(format!(
                "radius_m must be a positive number, got {radius_m}"
            )));
        }
        options.profile.validate()?;
        let center = center.into();
        let bbox = overpass::bbox_from_point(center.lat, center.lon, radius_m);
        let query = overpass::create_overpass_query(&bbox, network_type);
        let xml = fetch_cached(&query).await?;
        let data = parse_xml(&xml)?;
        if data.nodes.is_empty() {
            return Err(OsmGraphError::EmptyGraph);
        }
        SpatialGraph::from_osm_data_with(data, network_type, options)
    }

    /// Geocode `place` with Nominatim, then download the road network within
    /// `radius_m` metres of it as [`SpatialGraph::from_point`] does.
    pub async fn from_place(
        place: &str,
        radius_m: f64,
        network_type: NetworkType,
        options: &BuildOptions,
    ) -> Result<Self, OsmGraphError> {
        let center = crate::geocoding::geocode(place).await?;
        Self::from_point(center, radius_m, network_type, options).await
    }
}
