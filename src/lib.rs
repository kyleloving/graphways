// Public modules — available to any Rust crate that depends on this library.
// None of these import pyo3, so they compile cleanly without the extension-module feature.
pub mod error;
pub mod feasibility;
pub mod filters;
pub mod geocoding;
pub mod graph;
pub mod isochrone;
pub mod overpass;
pub mod pbf;
pub mod poi;
pub mod profile;
pub mod reachability;
pub mod restrictions;
pub mod routing;
pub mod utils;

// Internal implementation details; not part of the public Rust API.
mod cache;
mod ch;
mod search;
mod simplify;

// Python bindings: compiled only when maturin builds the extension module,
// so plain Rust dependents get no pyo3 / Python linkage at all.
#[cfg(feature = "extension-module")]
mod python;
