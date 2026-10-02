#[derive(Debug)]
pub enum OsmGraphError {
    Network(reqwest::Error),
    XmlParse(quick_xml::DeError),
    EmptyGraph,
    /// Backward-compatible catch-all for older call sites.
    NodeNotFound,
    OriginNodeNotFound,
    DestinationNodeNotFound,
    SnapDistanceExceeded {
        role: &'static str,
        distance_m: f64,
        max_distance_m: f64,
    },
    PathNotFound,
    LockPoisoned,
    GeocodingFailed(String),
    InvalidInput(String),
    Io(std::io::Error),
    PbfError(String),
    /// An origin-destination query whose trip cannot fit the time budget.
    Infeasible(crate::feasibility::InfeasibleReason),
    /// A saved graph file that is corrupt, truncated, or from an
    /// incompatible version.
    InvalidGraphFile(String),
}

impl std::fmt::Display for OsmGraphError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OsmGraphError::Network(e) => write!(f, "Network error: {}", e),
            OsmGraphError::XmlParse(e) => write!(f, "XML parse error: {}", e),
            OsmGraphError::EmptyGraph => write!(f, "Graph is empty"),
            OsmGraphError::NodeNotFound => write!(f, "No node found near the given coordinates"),
            OsmGraphError::OriginNodeNotFound => {
                write!(f, "No graph node found near the origin coordinates")
            }
            OsmGraphError::DestinationNodeNotFound => {
                write!(f, "No graph node found near the destination coordinates")
            }
            OsmGraphError::SnapDistanceExceeded {
                role,
                distance_m,
                max_distance_m,
            } => write!(
                f,
                "{} snapped {:.1} m from the graph, exceeding max_snap_m {:.1}",
                role, distance_m, max_distance_m
            ),
            OsmGraphError::PathNotFound => write!(
                f,
                "No path found between the snapped origin and destination nodes"
            ),
            OsmGraphError::LockPoisoned => write!(f, "Internal cache lock was poisoned"),
            OsmGraphError::GeocodingFailed(p) => write!(f, "Could not geocode '{}'", p),
            OsmGraphError::InvalidInput(msg) => write!(f, "Invalid input: {}", msg),
            OsmGraphError::Io(e) => write!(f, "IO error: {}", e),
            OsmGraphError::PbfError(msg) => write!(f, "PBF error: {}", msg),
            OsmGraphError::Infeasible(reason) => write!(f, "{reason}"),
            OsmGraphError::InvalidGraphFile(msg) => write!(f, "Invalid graph file: {msg}"),
        }
    }
}

impl std::error::Error for OsmGraphError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            OsmGraphError::Network(e) => Some(e),
            OsmGraphError::XmlParse(e) => Some(e),
            OsmGraphError::Io(e) => Some(e),
            OsmGraphError::Infeasible(e) => Some(e),
            _ => None,
        }
    }
}

impl From<reqwest::Error> for OsmGraphError {
    fn from(e: reqwest::Error) -> Self {
        OsmGraphError::Network(e)
    }
}

impl From<quick_xml::DeError> for OsmGraphError {
    fn from(e: quick_xml::DeError) -> Self {
        OsmGraphError::XmlParse(e)
    }
}

impl From<crate::feasibility::InfeasibleReason> for OsmGraphError {
    fn from(e: crate::feasibility::InfeasibleReason) -> Self {
        OsmGraphError::Infeasible(e)
    }
}

impl From<std::io::Error> for OsmGraphError {
    fn from(e: std::io::Error) -> Self {
        OsmGraphError::Io(e)
    }
}

// Only compile the pyo3 conversion when building the Python extension.
#[cfg(feature = "extension-module")]
impl From<OsmGraphError> for pyo3::PyErr {
    fn from(e: OsmGraphError) -> Self {
        match e {
            OsmGraphError::Network(_) => {
                pyo3::exceptions::PyConnectionError::new_err(e.to_string())
            }
            OsmGraphError::Io(_) => pyo3::exceptions::PyOSError::new_err(e.to_string()),
            OsmGraphError::XmlParse(_)
            | OsmGraphError::InvalidInput(_)
            | OsmGraphError::PbfError(_)
            | OsmGraphError::InvalidGraphFile(_)
            | OsmGraphError::EmptyGraph => pyo3::exceptions::PyValueError::new_err(e.to_string()),
            OsmGraphError::Infeasible(crate::feasibility::InfeasibleReason::BudgetTooTight {
                ..
            }) => pyo3::exceptions::PyValueError::new_err(e.to_string()),
            OsmGraphError::Infeasible(crate::feasibility::InfeasibleReason::NoPathExists) => {
                pyo3::exceptions::PyLookupError::new_err(e.to_string())
            }
            OsmGraphError::NodeNotFound
            | OsmGraphError::OriginNodeNotFound
            | OsmGraphError::DestinationNodeNotFound
            | OsmGraphError::SnapDistanceExceeded { .. }
            | OsmGraphError::PathNotFound
            | OsmGraphError::GeocodingFailed(_) => {
                pyo3::exceptions::PyLookupError::new_err(e.to_string())
            }
            OsmGraphError::LockPoisoned => pyo3::exceptions::PyRuntimeError::new_err(e.to_string()),
        }
    }
}
