use geo::{MultiPolygon, Polygon};
use geojson::{GeoJson, Geometry, GeometryValue, Position};

pub fn calculate_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;

    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();

    let lat1 = lat1.to_radians();
    let lat2 = lat2.to_radians();

    let a = (dlat / 2.0).sin().powi(2) + (dlon / 2.0).sin().powi(2) * lat1.cos() * lat2.cos();
    let c = 2.0 * a.sqrt().min(1.0).asin();

    EARTH_RADIUS_M * c
}

pub fn calculate_travel_time(length: f64, speed_kph: f64) -> f64 {
    if !length.is_finite() || !speed_kph.is_finite() || length < 0.0 || speed_kph <= 0.0 {
        return f64::INFINITY;
    }
    let speed_m_per_s = speed_kph / 3.6;
    length / speed_m_per_s
}

fn ring_coords(ring: &geo::LineString<f64>) -> Vec<Position> {
    // geo coordinates are x = lon, y = lat, which is GeoJSON's order too.
    ring.0
        .iter()
        .map(|coord| (coord.x, coord.y).into())
        .collect()
}

fn polygon_rings(polygon: &Polygon<f64>) -> Vec<Vec<Position>> {
    std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .map(ring_coords)
        .collect()
}

/// A polygon (x = lon, y = lat) as a GeoJSON Polygon geometry.
pub fn polygon_to_geojson(polygon: &Polygon<f64>) -> GeoJson {
    GeoJson::Geometry(Geometry::new(GeometryValue::Polygon {
        coordinates: polygon_rings(polygon),
    }))
}

pub fn polygon_to_geojson_string(polygon: &Polygon<f64>) -> String {
    polygon_to_geojson(polygon).to_string()
}

/// A multipolygon (x = lon, y = lat) as a GeoJSON MultiPolygon geometry.
pub fn multipolygon_to_geojson(area: &MultiPolygon<f64>) -> GeoJson {
    GeoJson::Geometry(Geometry::new(GeometryValue::MultiPolygon {
        coordinates: area.0.iter().map(polygon_rings).collect(),
    }))
}

pub fn multipolygon_to_geojson_string(area: &MultiPolygon<f64>) -> String {
    multipolygon_to_geojson(area).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance_same_point() {
        let d = calculate_distance(48.0, 11.0, 48.0, 11.0);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn test_distance_known_value() {
        // Munich to roughly 1km north — should be close to 1000m
        let d = calculate_distance(48.0, 11.0, 48.009, 11.0);
        assert!((d - 1000.0).abs() < 10.0, "Expected ~1000m, got {}", d);
    }

    #[test]
    fn test_distance_is_symmetric() {
        let d1 = calculate_distance(48.0, 11.0, 52.0, 13.0);
        let d2 = calculate_distance(52.0, 13.0, 48.0, 11.0);
        assert!((d1 - d2).abs() < 1e-6);
    }

    #[test]
    fn test_travel_time_basic() {
        // 1000m at 36 kph = 100 seconds
        let t = calculate_travel_time(1000.0, 36.0);
        assert!((t - 100.0).abs() < 1e-6, "Expected 100s, got {}", t);
    }

    #[test]
    fn test_travel_time_walking() {
        // 500m at 5 kph = 360 seconds
        let t = calculate_travel_time(500.0, 5.0);
        assert!((t - 360.0).abs() < 1e-6);
    }

    #[test]
    fn test_travel_time_rejects_invalid_speed() {
        assert!(calculate_travel_time(500.0, 0.0).is_infinite());
        assert!(calculate_travel_time(500.0, -10.0).is_infinite());
    }

    #[test]
    fn test_polygon_to_geojson_preserves_interiors() {
        let exterior = geo::LineString::from(vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        ]);
        let hole = geo::LineString::from(vec![
            (2.0, 2.0),
            (3.0, 2.0),
            (3.0, 3.0),
            (2.0, 3.0),
            (2.0, 2.0),
        ]);
        let polygon = Polygon::new(exterior, vec![hole]);
        let geojson = polygon_to_geojson(&polygon);

        if let GeoJson::Geometry(Geometry {
            value: GeometryValue::Polygon { coordinates: rings },
            ..
        }) = geojson
        {
            assert_eq!(rings.len(), 2);
        } else {
            panic!("expected GeoJSON polygon");
        }
    }
}
