use crate::error::OsmGraphError;
use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

const XML_CACHE_CAPACITY: usize = 20;

#[derive(Default)]
struct XmlCache {
    entries: HashMap<String, String>,
    order: VecDeque<String>,
}

impl XmlCache {
    fn get(&self, query: &str) -> Option<String> {
        self.entries.get(query).cloned()
    }

    fn put(&mut self, query: String, xml: String) {
        if !self.entries.contains_key(&query) {
            self.order.push_back(query.clone());
        }

        self.entries.insert(query, xml);

        while self.entries.len() > XML_CACHE_CAPACITY {
            if let Some(oldest) = self.order.pop_front() {
                self.entries.remove(&oldest);
            }
        }
    }

    #[cfg(feature = "extension-module")]
    fn clear(&mut self) {
        self.entries.clear();
        self.order.clear();
    }
}

static XML_CACHE: OnceLock<Mutex<XmlCache>> = OnceLock::new();

fn xml_cache() -> &'static Mutex<XmlCache> {
    XML_CACHE.get_or_init(|| Mutex::new(XmlCache::default()))
}

pub fn check_xml_cache(query: &str) -> Result<Option<String>, OsmGraphError> {
    Ok(xml_cache()
        .lock()
        .map_err(|_| OsmGraphError::LockPoisoned)?
        .get(query))
}

pub fn insert_into_xml_cache(query: String, xml: String) -> Result<(), OsmGraphError> {
    xml_cache()
        .lock()
        .map_err(|_| OsmGraphError::LockPoisoned)?
        .put(query, xml);
    Ok(())
}

#[cfg(feature = "extension-module")]
pub fn clear_cache() -> Result<(), OsmGraphError> {
    xml_cache()
        .lock()
        .map_err(|_| OsmGraphError::LockPoisoned)?
        .clear();
    Ok(())
}

// --- Disk-backed XML cache ---

/// FNV-1a 64-bit hash — stable across Rust versions, no dependencies.
fn fnv1a(s: &str) -> u64 {
    let mut hash: u64 = 14695981039346656037;
    for byte in s.bytes() {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(1099511628211);
    }
    hash
}

/// Returns the disk cache directory, overridable via `GRAPHWAYS_CACHE_DIR`.
///
/// The Python package defaults to a `cache/` folder in the current working
/// directory — the same convention used by OSMnx, so researchers get
/// persistent, visible caching next to their notebooks and scripts. As a Rust
/// library it defaults to the user's cache directory instead, so programs
/// using it do not find folders appearing wherever they run.
pub fn disk_cache_dir() -> PathBuf {
    std::env::var_os("GRAPHWAYS_CACHE_DIR")
        .or_else(|| std::env::var_os("OSM_GRAPH_CACHE_DIR"))
        .map(PathBuf::from)
        .or_else(|| {
            if cfg!(feature = "extension-module") {
                None
            } else {
                user_cache_dir()
            }
        })
        .unwrap_or_else(|| PathBuf::from("cache"))
}

/// The platform's per-user cache location for graphways: `%LOCALAPPDATA%`
/// on Windows, `~/Library/Caches` on macOS, `$XDG_CACHE_HOME` or `~/.cache`
/// elsewhere. `None` when the environment does not say where that is.
fn user_cache_dir() -> Option<PathBuf> {
    let var = |name: &str| {
        std::env::var_os(name)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    let base = if cfg!(windows) {
        return var("LOCALAPPDATA").map(|d| d.join("graphways").join("cache"));
    } else if cfg!(target_os = "macos") {
        var("HOME")?.join("Library").join("Caches")
    } else {
        var("XDG_CACHE_HOME").or_else(|| Some(var("HOME")?.join(".cache")))?
    };
    Some(base.join("graphways"))
}

#[cfg(any(test, feature = "extension-module"))]
fn is_safe_cache_dir(dir: &std::path::Path) -> bool {
    let Some(name) = dir.file_name().and_then(|n| n.to_str()) else {
        return false;
    };
    !dir.as_os_str().is_empty() && name.contains("cache")
}

fn disk_xml_path(query: &str) -> PathBuf {
    disk_cache_dir().join(format!("{:016x}.xml", fnv1a(query)))
}

/// Check the disk cache for a previously fetched Overpass XML response.
/// Returns None on any error — a cache miss is always safe.
pub fn check_disk_xml_cache(query: &str) -> Option<String> {
    std::fs::read_to_string(disk_xml_path(query)).ok()
}

/// Persist an Overpass XML response to disk. Best-effort — silently ignores errors.
///
/// The response goes to a temporary file first and is then renamed into
/// place, so an interrupted or concurrent write never leaves a truncated
/// file for later reads to pick up.
pub fn write_disk_xml_cache(query: &str, xml: &str) {
    use std::sync::atomic::{AtomicU64, Ordering};
    static WRITES: AtomicU64 = AtomicU64::new(0);

    let dir = disk_cache_dir();
    if std::fs::create_dir_all(&dir).is_err() {
        return;
    }
    let path = disk_xml_path(query);
    let temp = path.with_extension(format!(
        "{}-{}.tmp",
        std::process::id(),
        WRITES.fetch_add(1, Ordering::Relaxed)
    ));
    if std::fs::write(&temp, xml).is_err() || std::fs::rename(&temp, &path).is_err() {
        let _ = std::fs::remove_file(&temp);
    }
}

/// Delete all files in the disk cache directory.
#[cfg(any(test, feature = "extension-module"))]
pub fn clear_disk_cache() -> Result<(), OsmGraphError> {
    let dir = disk_cache_dir();
    if dir.exists() {
        if !is_safe_cache_dir(&dir) {
            return Err(OsmGraphError::InvalidInput(format!(
                "refusing to clear cache directory '{}': final path component must contain 'cache'",
                dir.display()
            )));
        }
        std::fs::remove_dir_all(&dir)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    static ENV_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn test_disk_cache_round_trip() {
        let _guard = ENV_LOCK.lock().unwrap();
        // Use a nanosecond-suffixed temp dir to avoid collisions with parallel test runs
        let dir = std::env::temp_dir().join(format!(
            "osm_graph_test_cache_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .subsec_nanos()
        ));
        std::env::set_var("GRAPHWAYS_CACHE_DIR", &dir);

        write_disk_xml_cache("test_query", "<xml>hello</xml>");
        let result = check_disk_xml_cache("test_query");
        assert_eq!(result, Some("<xml>hello</xml>".to_string()));

        assert!(check_disk_xml_cache("other_query").is_none());

        // Rewriting replaces the entry and leaves no temporary files behind.
        write_disk_xml_cache("test_query", "<xml>again</xml>");
        assert_eq!(
            check_disk_xml_cache("test_query"),
            Some("<xml>again</xml>".to_string())
        );
        let files: Vec<_> = std::fs::read_dir(&dir).unwrap().collect();
        assert_eq!(files.len(), 1, "{files:?}");

        let _ = std::fs::remove_dir_all(&dir);
        std::env::remove_var("GRAPHWAYS_CACHE_DIR");
    }

    #[test]
    fn default_cache_dir_depends_on_the_build() {
        let _guard = ENV_LOCK.lock().unwrap();
        let saved: Vec<_> = ["GRAPHWAYS_CACHE_DIR", "OSM_GRAPH_CACHE_DIR"]
            .into_iter()
            .map(|k| (k, std::env::var_os(k)))
            .collect();
        for (k, _) in &saved {
            std::env::remove_var(k);
        }

        let dir = disk_cache_dir();
        if cfg!(feature = "extension-module") {
            // Python: next to the notebook, OSMnx style.
            assert_eq!(dir, PathBuf::from("cache"));
        } else {
            // Rust library: the user's cache directory, not the working one.
            assert!(dir.is_absolute(), "{}", dir.display());
            assert!(dir.components().any(|c| c.as_os_str() == "graphways"));
        }

        for (k, v) in saved {
            if let Some(v) = v {
                std::env::set_var(k, v);
            }
        }
    }

    #[test]
    fn test_clear_disk_cache_refuses_unsafe_path() {
        let _guard = ENV_LOCK.lock().unwrap();
        let dir = std::env::temp_dir().join(format!(
            "osm_graph_unsafe_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .subsec_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        std::env::set_var("GRAPHWAYS_CACHE_DIR", &dir);

        let result = clear_disk_cache();
        assert!(matches!(result, Err(OsmGraphError::InvalidInput(_))));
        assert!(dir.exists());

        let _ = std::fs::remove_dir_all(&dir);
        std::env::remove_var("GRAPHWAYS_CACHE_DIR");
    }
}
