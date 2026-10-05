// Public modules — available to any Rust crate that depends on this library.
// None of these import pyo3, so they compile cleanly without the extension-module feature.
pub mod accessibility;
pub mod error;
pub mod feasibility;
pub mod filters;
#[cfg(feature = "network")]
pub mod geocoding;
pub mod graph;
pub mod isochrone;
pub mod matrix;
pub mod overpass;
pub mod pbf;
pub mod poi;
pub mod profile;
pub mod reachability;
pub mod restrictions;
pub mod routing;
pub mod transit;
pub mod utils;

// Internal implementation details; not part of the public Rust API.
#[cfg(feature = "network")]
mod cache;
mod ch;
#[cfg(feature = "network")]
mod download;
mod persist;
mod search;
mod simplify;
mod turns;

// Python bindings: compiled only when maturin builds the extension module,
// so plain Rust dependents get no pyo3 / Python linkage at all.
#[cfg(feature = "extension-module")]
mod python;
