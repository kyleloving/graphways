use crate::cache;
use crate::error::OsmGraphError;
use crate::graph::{SpatialGraph, XmlData};
use crate::overpass;
use crate::reachability::ReachabilityResult;
#[cfg(any(test, feature = "extension-module"))]
use geo::{Coord, LineString, Polygon};
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Poi {
    pub id: i64,
    pub lat: f64,
    pub lon: f64,
    pub tags: HashMap<String, String>,
}

/// A POI confirmed reachable via the road network, with the actual network
/// travel time from the origin rather than a straight-line approximation.
pub struct ReachablePoi {
    pub poi: Poi,
    /// Network travel time from the origin to the nearest graph node of this
    /// POI, in seconds.
    pub travel_time_s: f64,
    /// OSM node id of the graph node this POI snapped to.
    pub snap_node_id: i64,
    /// Straight-line distance from the POI coordinate to its snapped graph node.
    pub snap_distance_m: f64,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

fn create_poi_query(bbox: &str) -> String {
    format!(
        "[out:xml];(\
         node[\"tourism\"]({bbox});\
         node[\"historic\"]({bbox});\
         node[\"natural\"~\"peak|waterfall|cave_entrance|beach|hot_spring\"]({bbox});\
         node[\"amenity\"~\"restaurant|fast_food|cafe|bar|pub|biergarten|ice_cream|food_court|\
         museum|theatre|cinema|arts_centre|library|place_of_worship|spa|swimming_pool\"]({bbox});\
         node[\"leisure\"~\"park|nature_reserve|garden|sports_centre|fitness_centre\"]({bbox});\
         node[\"shop\"~\"bakery|deli|chocolate|wine|cheese|mall|department_store\"]({bbox});\
         );out;"
    )
}

/// Extract a `south,west,north,east` Overpass bbox from an isochrone polygon.
/// Internal coordinate convention: x = lat, y = lon.
#[cfg(feature = "extension-module")]
fn bbox_from_polygon(polygon: &Polygon<f64>) -> String {
    let mut min_lat = f64::MAX;
    let mut max_lat = f64::MIN;
    let mut min_lon = f64::MAX;
    let mut max_lon = f64::MIN;
    for coord in polygon.exterior().coords() {
        min_lat = min_lat.min(coord.x);
        max_lat = max_lat.max(coord.x);
        min_lon = min_lon.min(coord.y);
        max_lon = max_lon.max(coord.y);
    }
    format!("{},{},{},{}", min_lat, min_lon, max_lat, max_lon)
}

async fn fetch_xml_cached(query: &str) -> Result<String, OsmGraphError> {
    if let Some(cached) = cache::check_xml_cache(query)? {
        return Ok(cached);
    }
    if let Some(disk) = cache::check_disk_xml_cache(query) {
        cache::insert_into_xml_cache(query.to_string(), disk.clone())?;
        return Ok(disk);
    }
    let fetched = overpass::make_request(&overpass::overpass_url(), query).await?;
    cache::write_disk_xml_cache(query, &fetched);
    cache::insert_into_xml_cache(query.to_string(), fetched.clone())?;
    Ok(fetched)
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Parse a GeoJSON geometry string (as produced by `polygon_to_geojson_string`)
/// back into a `geo::Polygon<f64>` using the library's internal x=lat, y=lon convention.
#[cfg(any(test, feature = "extension-module"))]
pub(crate) fn parse_isochrone(geojson_str: &str) -> Result<Polygon<f64>, OsmGraphError> {
    let gj: geojson::GeoJson = geojson_str
        .parse()
        .map_err(|_| OsmGraphError::InvalidInput("invalid GeoJSON".into()))?;
    let rings = match gj {
        geojson::GeoJson::Geometry(geom) => match geom.value {
            geojson::Value::Polygon(rings) => rings,
            _ => {
                return Err(OsmGraphError::InvalidInput(
                    "expected Polygon geometry".into(),
                ))
            }
        },
        _ => {
            return Err(OsmGraphError::InvalidInput(
                "expected a GeoJSON Geometry, not a Feature or FeatureCollection".into(),
            ))
        }
    };
    // GeoJSON coords are [lon, lat]; internal convention is x=lat, y=lon.
    let exterior: Vec<Coord<f64>> = rings[0]
        .iter()
        .map(|c| Coord { x: c[1], y: c[0] })
        .collect();
    Ok(Polygon::new(LineString::from(exterior), vec![]))
}

/// Fetch POIs within a polygon and filter by geometric containment.
///
/// This is the original approach: POIs whose lat/lon falls inside the polygon
/// are kept. It is fast and simple but uses polygon geometry as a proxy for
/// network reachability. Prefer [`fetch_pois_within_reachability`] when a
/// [`ReachabilityResult`] is already available.
#[cfg(feature = "extension-module")]
pub(crate) async fn fetch_pois_within(polygon: &Polygon<f64>) -> Result<Vec<Poi>, OsmGraphError> {
    use geo::{Contains, Point};

    let bbox = bbox_from_polygon(polygon);
    let query = create_poi_query(&bbox);
    let xml = fetch_xml_cached(&query).await?;
    let data: XmlData = quick_xml::de::from_str(&xml)?;

    let pois = data
        .nodes
        .into_iter()
        .filter(|n| polygon.contains(&Point::new(n.lat, n.lon)))
        .map(|n| Poi {
            id: n.id,
            lat: n.lat,
            lon: n.lon,
            tags: n.tags.into_iter().map(|t| (t.key, t.value)).collect(),
        })
        .collect();

    Ok(pois)
}

/// Fetch POIs and filter them by actual network travel time.
///
/// Uses the [`ReachabilityResult`] from a prior graph search as the truth
/// source instead of polygon containment:
///
/// 1. Derives a bounding box from the origin node and `max_cost`.
/// 2. Fetches POI nodes from Overpass within that box (cached).
/// 3. Snaps each POI to its nearest graph node via the spatial index.
/// 4. Keeps only POIs whose snapped node appears in `reachability.distances`.
///
/// The `travel_time_s` on each [`ReachablePoi`] is the Dijkstra distance to
/// the snapped node — the same value that drove the isochrone — not a
/// straight-line estimate. POIs that snap to unreachable nodes (across a
/// river, behind a highway, in a disconnected subgraph) are correctly excluded.
pub(crate) async fn fetch_pois_within_reachability(
    sg: &SpatialGraph,
    reachability: &ReachabilityResult,
) -> Result<Vec<ReachablePoi>, OsmGraphError> {
    // Size the bbox using the same generous speed assumption as isochrone
    // bbox sizing so the box always contains the full reachable area.
    let origin = &sg.graph[reachability.start];
    let max_speed_m_per_s = 120.0_f64 / 3.6;
    let radius_m = reachability.max_cost * max_speed_m_per_s * 1.2;
    let bbox = overpass::bbox_from_point(origin.lat, origin.lon, radius_m);
    let query = create_poi_query(&bbox);
    let xml = fetch_xml_cached(&query).await?;
    let data: XmlData = quick_xml::de::from_str(&xml)?;

    let pois = data
        .nodes
        .into_iter()
        .filter_map(|n| {
            // Pre-snapped POIs (PBF graphs) are an O(1) lookup; otherwise
            // snap on the fly through the spatial index.
            let snap = match &sg.poi_snaps {
                Some(snaps) => snaps.get(&n.id)?.snap,
                None => sg.snap_point(n.lat, n.lon)?,
            };
            let travel_time_s = *reachability.distances.get(&snap.node_index)?;
            Some(ReachablePoi {
                poi: Poi {
                    id: n.id,
                    lat: n.lat,
                    lon: n.lon,
                    tags: n.tags.into_iter().map(|t| (t.key, t.value)).collect(),
                },
                travel_time_s,
                snap_node_id: snap.node_id,
                snap_distance_m: snap.distance_m,
            })
        })
        .collect();

    Ok(pois)
}

/// Serialize a slice of [`Poi`] to a GeoJSON FeatureCollection.
#[cfg(any(test, feature = "extension-module"))]
pub(crate) fn pois_to_geojson(pois: &[Poi]) -> String {
    let features: Vec<geojson::Feature> = pois
        .iter()
        .map(|poi| {
            let geometry = geojson::Geometry::new(geojson::Value::Point(vec![poi.lon, poi.lat]));
            let props: geojson::JsonObject = poi
                .tags
                .iter()
                .map(|(k, v)| (k.clone(), geojson::JsonValue::String(v.clone())))
                .collect();
            geojson::Feature {
                geometry: Some(geometry),
                properties: Some(props),
                ..Default::default()
            }
        })
        .collect();

    geojson::GeoJson::FeatureCollection(geojson::FeatureCollection {
        features,
        bbox: None,
        foreign_members: None,
    })
    .to_string()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_isochrone_valid() {
        let geojson = r#"{"type":"Polygon","coordinates":[[[11.0,48.0],[11.1,48.0],[11.05,48.1],[11.0,48.0]]]}"#;
        let polygon = parse_isochrone(geojson).unwrap();
        let first = polygon.exterior().coords().next().unwrap();
        assert!(
            (first.x - 48.0).abs() < 1e-9,
            "x should be lat (48.0), got {}",
            first.x
        );
        assert!(
            (first.y - 11.0).abs() < 1e-9,
            "y should be lon (11.0), got {}",
            first.y
        );
    }

    #[test]
    fn test_parse_isochrone_invalid_json() {
        let result = parse_isochrone("not valid json");
        assert!(matches!(
            result,
            Err(crate::error::OsmGraphError::InvalidInput(_))
        ));
    }

    #[test]
    fn test_parse_isochrone_wrong_geometry_type() {
        let geojson = r#"{"type":"Point","coordinates":[11.0,48.0]}"#;
        let result = parse_isochrone(geojson);
        assert!(matches!(
            result,
            Err(crate::error::OsmGraphError::InvalidInput(_))
        ));
    }

    #[test]
    fn test_pois_to_geojson_empty() {
        let json = pois_to_geojson(&[]);
        let gj: geojson::GeoJson = json.parse().unwrap();
        if let geojson::GeoJson::FeatureCollection(fc) = gj {
            assert_eq!(fc.features.len(), 0);
        } else {
            panic!("expected FeatureCollection");
        }
    }

    #[test]
    fn test_pois_to_geojson_coordinate_order() {
        let poi = Poi {
            id: 1,
            lat: 48.0,
            lon: 11.0,
            tags: HashMap::new(),
        };
        let json = pois_to_geojson(&[poi]);
        let gj: geojson::GeoJson = json.parse().unwrap();
        if let geojson::GeoJson::FeatureCollection(fc) = gj {
            let geom = fc.features[0].geometry.as_ref().unwrap();
            if let geojson::Value::Point(coords) = &geom.value {
                assert!((coords[0] - 11.0).abs() < 1e-9, "first coord should be lon");
                assert!(
                    (coords[1] - 48.0).abs() < 1e-9,
                    "second coord should be lat"
                );
            } else {
                panic!("expected Point geometry");
            }
        } else {
            panic!("expected FeatureCollection");
        }
    }

    #[test]
    fn test_pois_to_geojson_tags_as_properties() {
        let mut tags = HashMap::new();
        tags.insert("tourism".to_string(), "museum".to_string());
        let poi = Poi {
            id: 1,
            lat: 48.0,
            lon: 11.0,
            tags,
        };
        let json = pois_to_geojson(&[poi]);
        let gj: geojson::GeoJson = json.parse().unwrap();
        if let geojson::GeoJson::FeatureCollection(fc) = gj {
            let props = fc.features[0].properties.as_ref().unwrap();
            assert_eq!(
                props["tourism"],
                geojson::JsonValue::String("museum".to_string())
            );
        } else {
            panic!("expected FeatureCollection");
        }
    }
}
