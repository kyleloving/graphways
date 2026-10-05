// Define an enum for network types
#[cfg(feature = "network")]
use reqwest::header::{HeaderMap, RETRY_AFTER};
#[cfg(feature = "network")]
use std::sync::{Mutex, OnceLock};
#[cfg(feature = "network")]
use std::time::{Duration, Instant};
#[cfg(feature = "network")]
use tokio::time::sleep;

const DEFAULT_OVERPASS_URL: &str = "https://overpass-api.de/api/interpreter";
const DEFAULT_NOMINATIM_URL: &str = "https://nominatim.openstreetmap.org/search";
#[cfg(feature = "network")]
const MAX_RETRIES: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum NetworkType {
    Drive,
    DriveService,
    Walk,
    Bike,
    All,
    AllPrivate,
}

// Custom error type for better error messages
#[derive(Debug)]
pub enum OverpassError {
    #[cfg(feature = "network")]
    RequestError(reqwest::Error),
    InvalidNetworkType,
}

impl std::fmt::Display for OverpassError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            #[cfg(feature = "network")]
            OverpassError::RequestError(err) => write!(f, "Request Error: {}", err),
            OverpassError::InvalidNetworkType => write!(f, "Invalid Network Type"),
        }
    }
}

// Function to get OSM filter
pub fn get_osm_filter(network_type: NetworkType) -> Result<&'static str, OverpassError> {
    match network_type {
        NetworkType::Drive => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|bridleway|bus_guideway|construction|corridor|cycleway|elevator|escalator|footway|no|path|pedestrian|planned|platform|proposed|raceway|razed|service|steps|track\"][\"motor_vehicle\"!~\"no\"][\"motorcar\"!~\"no\"][\"service\"!~\"alley|driveway|emergency_access|parking|parking_aisle|private\"]"
        ),
        NetworkType::DriveService => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|bridleway|bus_guideway|construction|corridor|cycleway|elevator|escalator|footway|no|path|pedestrian|planned|platform|proposed|raceway|razed|steps|track\"][\"motor_vehicle\"!~\"no\"][\"motorcar\"!~\"no\"][\"service\"!~\"emergency_access|parking|parking_aisle|private\"]"
        ),
        NetworkType::Walk => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|bus_guideway|construction|corridor|elevator|escalator|motor|no|planned|platform|proposed|raceway|razed\"][\"foot\"!~\"no\"][\"service\"!~\"private\"]"
        ),
        NetworkType::Bike => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|bus_guideway|construction|corridor|elevator|escalator|footway|motor|no|planned|platform|proposed|raceway|razed|steps\"][\"bicycle\"!~\"no\"][\"service\"!~\"private\"]"
        ),
        NetworkType::All => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|construction|no|planned|platform|proposed|raceway|razed\"][\"service\"!~\"private\"]"
        ),
        NetworkType::AllPrivate => Ok(
            "[\"highway\"][\"area\"!~\"yes\"][\"highway\"!~\"abandoned|construction|no|planned|platform|proposed|raceway|razed\"]"
        ),
    }
}

// Function to create the Overpass query string
/// The Overpass query for a road network in a bounding box. Driving queries
/// also fetch the turn-restriction relations on those roads.
pub fn create_overpass_query(polygon_coord_str: &str, network_type: NetworkType) -> String {
    let filter = get_osm_filter(network_type).unwrap_or("");
    if crate::restrictions::applies_to(network_type) {
        format!(
            "[out:xml][timeout:50];(way{filter}({polygon_coord_str});>;)->.roads;\
             (.roads;rel(bw.roads)[\"type\"=\"restriction\"];);out;"
        )
    } else {
        format!("[out:xml][timeout:50];(way{filter}({polygon_coord_str});>;);out;")
    }
}

#[cfg(feature = "network")]
static CLIENT: OnceLock<reqwest::Client> = OnceLock::new();
#[cfg(feature = "network")]
static OVERPASS_LAST_REQUEST: OnceLock<Mutex<Option<Instant>>> = OnceLock::new();
#[cfg(feature = "network")]
static NOMINATIM_LAST_REQUEST: OnceLock<Mutex<Option<Instant>>> = OnceLock::new();

// Reuse a single reqwest::Client across all HTTP calls in the library.
#[cfg(feature = "network")]
pub(crate) fn client() -> &'static reqwest::Client {
    CLIENT.get_or_init(|| {
        reqwest::Client::builder()
            .user_agent(user_agent())
            .build()
            .expect("failed to build HTTP client")
    })
}

#[cfg(feature = "network")]
fn overpass_last_request() -> &'static Mutex<Option<Instant>> {
    OVERPASS_LAST_REQUEST.get_or_init(|| Mutex::new(None))
}

#[cfg(feature = "network")]
fn nominatim_last_request() -> &'static Mutex<Option<Instant>> {
    NOMINATIM_LAST_REQUEST.get_or_init(|| Mutex::new(None))
}

#[cfg_attr(not(feature = "network"), allow(dead_code))]
pub(crate) fn user_agent() -> String {
    std::env::var("GRAPHWAYS_USER_AGENT").unwrap_or_else(|_| {
        format!(
            "graphways/{} (https://github.com/kyleloving/graphways)",
            env!("CARGO_PKG_VERSION")
        )
    })
}

pub fn overpass_url() -> String {
    std::env::var("GRAPHWAYS_OVERPASS_URL").unwrap_or_else(|_| DEFAULT_OVERPASS_URL.to_string())
}

pub fn nominatim_url() -> String {
    std::env::var("GRAPHWAYS_NOMINATIM_URL").unwrap_or_else(|_| DEFAULT_NOMINATIM_URL.to_string())
}

#[cfg(feature = "network")]
async fn wait_for_slot(last_request: &Mutex<Option<Instant>>, min_interval: Duration) {
    let delay = {
        let mut last = match last_request.lock() {
            Ok(last) => last,
            Err(_) => return,
        };
        let now = Instant::now();
        let delay = last
            .and_then(|previous| previous.checked_add(min_interval))
            .and_then(|next_allowed| next_allowed.checked_duration_since(now))
            .unwrap_or_default();
        *last = Some(now + delay);
        delay
    };

    if !delay.is_zero() {
        sleep(delay).await;
    }
}

#[cfg(feature = "network")]
fn retry_after(headers: &HeaderMap) -> Option<Duration> {
    headers
        .get(RETRY_AFTER)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.parse::<u64>().ok())
        .map(Duration::from_secs)
}

#[cfg(feature = "network")]
pub(crate) fn is_retryable_status(status: reqwest::StatusCode) -> bool {
    status == reqwest::StatusCode::TOO_MANY_REQUESTS
        || status == reqwest::StatusCode::BAD_GATEWAY
        || status == reqwest::StatusCode::SERVICE_UNAVAILABLE
        || status == reqwest::StatusCode::GATEWAY_TIMEOUT
}

#[cfg(feature = "network")]
pub(crate) async fn retry_delay(headers: &HeaderMap, attempt: usize) {
    let fallback = Duration::from_millis(500 * (attempt as u64 + 1));
    sleep(retry_after(headers).unwrap_or(fallback)).await;
}

#[cfg(feature = "network")]
pub(crate) async fn wait_for_nominatim_slot() {
    wait_for_slot(nominatim_last_request(), Duration::from_secs(1)).await;
}

#[cfg(feature = "network")]
async fn wait_for_overpass_slot() {
    wait_for_slot(overpass_last_request(), Duration::from_millis(250)).await;
}

// Function to make request to Overpass API
#[cfg(feature = "network")]
pub async fn make_request(url: &str, query: &str) -> Result<String, reqwest::Error> {
    for attempt in 0..=MAX_RETRIES {
        wait_for_overpass_slot().await;
        let response = client()
            .post(url)
            .header("User-Agent", user_agent())
            .form(&[("data", query)])
            .send()
            .await?;

        if response.status().is_success() {
            return response.text().await;
        }

        let status = response.status();
        let headers = response.headers().clone();
        let error = response.error_for_status().unwrap_err();
        if attempt < MAX_RETRIES && is_retryable_status(status) {
            retry_delay(&headers, attempt).await;
            continue;
        }

        return Err(error);
    }

    unreachable!("retry loop always returns before completion")
}

/// Construct a `south,west,north,east` bounding box string from a point and radius.
pub fn bbox_from_point(lat: f64, lon: f64, dist: f64) -> String {
    const EARTH_RADIUS_M: f64 = 6_371_009.0;

    // Calculate deltas
    let delta_lat = (dist / EARTH_RADIUS_M) * (180.0 / std::f64::consts::PI);
    let delta_lon = (dist / EARTH_RADIUS_M) * (180.0 / std::f64::consts::PI)
        / (lat * std::f64::consts::PI / 180.0).cos();

    // Calculate bounding box
    let north = lat + delta_lat;
    let south = lat - delta_lat;
    let east = lon + delta_lon;
    let west = lon - delta_lon;

    // Construct polygon_coord_str for Overpass API query
    format!("{},{},{},{}", south, west, north, east)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    static ENV_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn test_bbox_is_symmetric() {
        let bbox = bbox_from_point(48.0, 11.0, 1000.0);
        let parts: Vec<f64> = bbox.split(',').map(|s| s.parse().unwrap()).collect();
        let (south, west, north, east) = (parts[0], parts[1], parts[2], parts[3]);
        assert!((48.0 - south - (north - 48.0)).abs() < 1e-6);
        assert!((11.0 - west - (east - 11.0)).abs() < 1e-6);
    }

    #[test]
    fn test_bbox_larger_dist_gives_larger_box() {
        let small = bbox_from_point(48.0, 11.0, 1_000.0);
        let large = bbox_from_point(48.0, 11.0, 10_000.0);
        let small_parts: Vec<f64> = small.split(',').map(|s| s.parse().unwrap()).collect();
        let large_parts: Vec<f64> = large.split(',').map(|s| s.parse().unwrap()).collect();
        assert!(large_parts[2] > small_parts[2]);
    }

    #[test]
    fn service_urls_can_be_overridden_by_environment() {
        let _guard = ENV_LOCK.lock().unwrap();
        std::env::set_var("GRAPHWAYS_OVERPASS_URL", "https://example.com/overpass");
        std::env::set_var("GRAPHWAYS_NOMINATIM_URL", "https://example.com/search");

        assert_eq!(overpass_url(), "https://example.com/overpass");
        assert_eq!(nominatim_url(), "https://example.com/search");

        std::env::remove_var("GRAPHWAYS_OVERPASS_URL");
        std::env::remove_var("GRAPHWAYS_NOMINATIM_URL");
    }

    #[test]
    fn user_agent_defaults_to_current_package_version() {
        let _guard = ENV_LOCK.lock().unwrap();
        std::env::remove_var("GRAPHWAYS_USER_AGENT");

        let ua = user_agent();
        assert!(ua.contains(env!("CARGO_PKG_VERSION")));
        assert!(ua.contains("graphways"));
    }

    #[test]
    fn user_agent_can_be_overridden_by_environment() {
        let _guard = ENV_LOCK.lock().unwrap();
        std::env::set_var("GRAPHWAYS_USER_AGENT", "my-app/1.0 contact@example.com");

        assert_eq!(user_agent(), "my-app/1.0 contact@example.com");

        std::env::remove_var("GRAPHWAYS_USER_AGENT");
    }
}
