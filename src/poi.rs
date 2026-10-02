#[cfg(feature = "network")]
use crate::cache;
#[cfg(any(test, feature = "network"))]
use crate::error::OsmGraphError;
#[cfg(feature = "network")]
use crate::graph::{OsmData, SpatialGraph};
#[cfg(feature = "network")]
use crate::overpass;
#[cfg(feature = "network")]
use crate::reachability::ReachabilityResult;
#[cfg(any(test, feature = "extension-module"))]
use geo::{Coord, LineString, MultiPolygon, Polygon};
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

#[cfg(feature = "network")]
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

/// A `south,west,north,east` Overpass bbox around an area (x = lon, y = lat).
#[cfg(feature = "extension-module")]
fn bbox_from_area(area: &MultiPolygon<f64>) -> Option<String> {
    use geo::BoundingRect;
    let rect = area.bounding_rect()?;
    Some(format!(
        "{},{},{},{}",
        rect.min().y,
        rect.min().x,
        rect.max().y,
        rect.max().x
    ))
}

#[cfg(feature = "network")]
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

/// Parse a GeoJSON Polygon or MultiPolygon (a bare geometry or a Feature)
/// into a `geo::MultiPolygon` with x = longitude, y = latitude.
#[cfg(any(test, feature = "extension-module"))]
pub(crate) fn parse_area(geojson_str: &str) -> Result<MultiPolygon<f64>, OsmGraphError> {
    let gj: geojson::GeoJson = geojson_str
        .parse()
        .map_err(|_| OsmGraphError::InvalidInput("invalid GeoJSON".into()))?;
    let value = match gj {
        geojson::GeoJson::Geometry(geometry) => geometry.value,
        geojson::GeoJson::Feature(geojson::Feature {
            geometry: Some(geometry),
            ..
        }) => geometry.value,
        _ => {
            return Err(OsmGraphError::InvalidInput(
                "expected a Polygon or MultiPolygon geometry or Feature".into(),
            ))
        }
    };
    let ring = |points: &Vec<geojson::Position>| -> LineString<f64> {
        points
            .iter()
            .map(|c| Coord { x: c[0], y: c[1] })
            .collect::<Vec<_>>()
            .into()
    };
    let polygon = |rings: &Vec<Vec<geojson::Position>>| -> Option<Polygon<f64>> {
        let (exterior, interiors) = rings.split_first()?;
        Some(Polygon::new(
            ring(exterior),
            interiors.iter().map(ring).collect(),
        ))
    };
    let polygons = match &value {
        geojson::GeometryValue::Polygon { coordinates: rings } => {
            polygon(rings).into_iter().collect()
        }
        geojson::GeometryValue::MultiPolygon { coordinates: parts } => {
            parts.iter().filter_map(polygon).collect()
        }
        _ => {
            return Err(OsmGraphError::InvalidInput(
                "expected Polygon or MultiPolygon geometry".into(),
            ))
        }
    };
    Ok(MultiPolygon(polygons))
}

/// Fetch POIs within a polygon and filter by geometric containment.
///
/// This is the original approach: POIs whose lat/lon falls inside the polygon
/// are kept. It is fast and simple but uses polygon geometry as a proxy for
/// network reachability. Prefer [`fetch_pois_within_reachability`] when a
/// [`ReachabilityResult`] is already available.
#[cfg(feature = "extension-module")]
pub(crate) async fn fetch_pois_within(area: &MultiPolygon<f64>) -> Result<Vec<Poi>, OsmGraphError> {
    use geo::{Contains, Point};

    let Some(bbox) = bbox_from_area(area) else {
        return Ok(Vec::new());
    };
    let query = create_poi_query(&bbox);
    let xml = fetch_xml_cached(&query).await?;
    let data: OsmData = quick_xml::de::from_str(&xml)?;

    let pois = data
        .nodes
        .into_iter()
        .filter(|n| area.contains(&Point::new(n.lon, n.lat)))
        .map(|n| Poi {
            id: n.id,
            lat: n.lat,
            lon: n.lon,
            tags: n.tags.into_iter().map(|t| (t.key, t.value)).collect(),
        })
        .collect();

    Ok(pois)
}

#[cfg(feature = "network")]
/// Fetch POIs and filter them by actual network travel time.
///
/// Uses the [`ReachabilityResult`] from a prior graph search as the truth
/// source instead of polygon containment:
///
/// 1. Derives a bounding box from the origin and `max_cost`.
/// 2. Fetches POI nodes from Overpass within that box (cached).
/// 3. Snaps each POI to the nearest road.
/// 4. Keeps POIs whose snapped road point is reachable within `max_cost`.
///
/// The `travel_time_s` on each [`ReachablePoi`] is the network travel time to
/// that road point — the same search that drove the isochrone — not a
/// straight-line estimate. POIs whose road is unreachable (across a river,
/// behind a highway, in a disconnected subgraph) are correctly excluded.
pub(crate) async fn fetch_pois_within_reachability(
    sg: &SpatialGraph,
    reachability: &ReachabilityResult,
) -> Result<Vec<ReachablePoi>, OsmGraphError> {
    // Size the bbox using the same generous speed assumption as isochrone
    // bbox sizing so the box always contains the full reachable area.
    let origin = reachability.origin;
    let max_speed_m_per_s = 120.0_f64 / 3.6;
    let radius_m = reachability.max_cost * max_speed_m_per_s * 1.2;
    let bbox = overpass::bbox_from_point(origin.lat, origin.lon, radius_m);
    let query = create_poi_query(&bbox);
    let xml = fetch_xml_cached(&query).await?;
    let data: OsmData = quick_xml::de::from_str(&xml)?;

    let pois = data
        .nodes
        .into_iter()
        .filter_map(|n| {
            // Pre-snapped POIs (PBF graphs) are an O(1) lookup; otherwise
            // snap on the fly through the spatial index.
            let snap = match &sg.poi_snaps {
                Some(snaps) => snaps.get(&n.id)?.snap,
                None => sg.snap_point((n.lat, n.lon))?,
            };
            let travel_time_s = reachability.time_to(sg, &snap)?;
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
            let geometry = geojson::Geometry::new_point((poi.lon, poi.lat));
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
    fn test_parse_area_polygon_uses_lon_as_x() {
        let geojson = r#"{"type":"Polygon","coordinates":[[[11.0,48.0],[11.1,48.0],[11.05,48.1],[11.0,48.0]]]}"#;
        let area = parse_area(geojson).unwrap();
        let first = area.0[0].exterior().coords().next().unwrap();
        assert!(
            (first.x - 11.0).abs() < 1e-9,
            "x should be lon, got {}",
            first.x
        );
        assert!(
            (first.y - 48.0).abs() < 1e-9,
            "y should be lat, got {}",
            first.y
        );
    }

    #[test]
    fn test_parse_area_multipolygon_keeps_parts_and_holes() {
        let geojson = r#"{"type":"MultiPolygon","coordinates":[
            [[[0,0],[4,0],[4,4],[0,4],[0,0]],[[1,1],[2,1],[2,2],[1,2],[1,1]]],
            [[[10,10],[11,10],[11,11],[10,10]]]]}"#;
        let area = parse_area(geojson).unwrap();
        assert_eq!(area.0.len(), 2);
        assert_eq!(area.0[0].interiors().len(), 1);
    }

    #[test]
    fn test_parse_area_invalid_json() {
        let result = parse_area("not valid json");
        assert!(matches!(
            result,
            Err(crate::error::OsmGraphError::InvalidInput(_))
        ));
    }

    #[test]
    fn test_parse_area_wrong_geometry_type() {
        let geojson = r#"{"type":"Point","coordinates":[11.0,48.0]}"#;
        let result = parse_area(geojson);
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
            if let geojson::GeometryValue::Point {
                coordinates: coords,
            } = &geom.value
            {
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
