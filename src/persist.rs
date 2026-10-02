//! Saving and loading prepared graphs.
//!
//! Building a graph from a PBF file and preparing it for routing takes
//! seconds; loading a saved graph takes a fraction of that. The file holds
//! the road graph (with each way's tags stored once), the forbidden turns,
//! the POI snaps and, if one was built, the routing hierarchy. The spatial
//! and search indexes are rebuilt on load, which is fast.
//!
//! The format is a short header (`GRAPHWAY` and a format version) followed by
//! a [postcard](https://docs.rs/postcard) payload. Files are tied to the
//! format version, not to the library version: a version change makes
//! [`SpatialGraph::load`] fail with [`OsmGraphError::InvalidGraphFile`]
//! rather than misread the data.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::sync::Arc;

use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};

use crate::ch::ContractionHierarchy;
use crate::error::OsmGraphError;
use crate::graph::{Edge, OsmNode, OsmTag, RoadGraph, SnapResult, SnappedPoi, SpatialGraph};
use crate::overpass::NetworkType;

const MAGIC: &[u8; 8] = b"GRAPHWAY";
const FORMAT_VERSION: u32 = 1;

#[derive(Serialize, Deserialize)]
struct SavedGraph {
    network_type: NetworkType,
    nodes: Vec<OsmNode>,
    tag_sets: Vec<Vec<OsmTag>>,
    edges: Vec<SavedEdge>,
    forbidden_turns: Vec<(u32, u32)>,
    poi_snaps: Option<Vec<SavedSnap>>,
    hierarchy: Option<ContractionHierarchy>,
}

#[derive(Serialize, Deserialize)]
struct SavedEdge {
    source: u32,
    target: u32,
    way_id: i64,
    last_way_id: i64,
    tag_set: u32,
    length: f64,
    speed_kph: f64,
    times: [f64; 3],
    geometry: Vec<(f64, f64)>,
}

#[derive(Serialize, Deserialize)]
struct SavedSnap {
    poi_id: i64,
    input: (f64, f64),
    snapped: (f64, f64),
    distance_m: f64,
    edge: Option<u32>,
    fraction: f64,
    node: u32,
}

fn invalid(message: impl Into<String>) -> OsmGraphError {
    OsmGraphError::InvalidGraphFile(message.into())
}

impl SpatialGraph {
    /// Write this graph to `path`, including the routing hierarchy if
    /// [`SpatialGraph::prepare_routing`] has run, so a later
    /// [`SpatialGraph::load`] routes at full speed straight away.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), OsmGraphError> {
        let snapshot = self.snapshot();
        let mut writer = BufWriter::new(File::create(path)?);
        writer.write_all(MAGIC)?;
        writer.write_all(&FORMAT_VERSION.to_le_bytes())?;
        let writer = postcard::to_io(&snapshot, writer)
            .map_err(|e| invalid(format!("could not encode graph: {e}")))?;
        writer
            .into_inner()
            .map_err(|e| OsmGraphError::Io(e.into_error()))?
            .sync_all()?;
        Ok(())
    }

    /// Read a graph written by [`SpatialGraph::save`].
    pub fn load(path: impl AsRef<Path>) -> Result<Self, OsmGraphError> {
        let mut bytes = Vec::new();
        File::open(path)?.read_to_end(&mut bytes)?;
        let (header, payload) = bytes
            .split_at_checked(MAGIC.len() + 4)
            .ok_or_else(|| invalid("file is too short"))?;
        if &header[..MAGIC.len()] != MAGIC {
            return Err(invalid("not a graphways graph file"));
        }
        let version = u32::from_le_bytes(header[MAGIC.len()..].try_into().expect("4 bytes"));
        if version != FORMAT_VERSION {
            return Err(invalid(format!(
                "file has format version {version}, this library reads version {FORMAT_VERSION}; \
                 rebuild the graph and save it again"
            )));
        }
        let saved: SavedGraph =
            postcard::from_bytes(payload).map_err(|e| invalid(format!("corrupt data: {e}")))?;
        Self::restore(saved)
    }

    fn snapshot(&self) -> SavedGraph {
        let mut tag_sets: Vec<Vec<OsmTag>> = Vec::new();
        let mut tag_set_of: HashMap<*const [OsmTag], u32> = HashMap::new();
        let edges = self
            .graph
            .edge_references()
            .map(|edge| {
                let way = edge.weight();
                let tag_set = *tag_set_of.entry(Arc::as_ptr(&way.tags)).or_insert_with(|| {
                    tag_sets.push(way.tags.to_vec());
                    (tag_sets.len() - 1) as u32
                });
                SavedEdge {
                    source: edge.source().index() as u32,
                    target: edge.target().index() as u32,
                    way_id: way.way_id,
                    last_way_id: way.last_way_id,
                    tag_set,
                    length: way.length,
                    speed_kph: way.speed_kph,
                    times: [
                        way.walk_travel_time,
                        way.bike_travel_time,
                        way.drive_travel_time,
                    ],
                    geometry: way.geometry.clone(),
                }
            })
            .collect();
        let poi_snaps = self.poi_snaps.as_ref().map(|snaps| {
            let mut saved: Vec<SavedSnap> = snaps
                .values()
                .map(|p| SavedSnap {
                    poi_id: p.poi_id,
                    input: (p.snap.input_lat, p.snap.input_lon),
                    snapped: (p.snap.snapped_lat, p.snap.snapped_lon),
                    distance_m: p.snap.distance_m,
                    edge: p.snap.edge.map(|e| e.index() as u32),
                    fraction: p.snap.fraction,
                    node: p.snap.node_index.index() as u32,
                })
                .collect();
            saved.sort_unstable_by_key(|s| s.poi_id);
            saved
        });
        SavedGraph {
            network_type: self.network_type(),
            nodes: self.graph.node_weights().cloned().collect(),
            tag_sets,
            edges,
            forbidden_turns: self
                .forbidden_turns()
                .iter()
                .map(|&(a, b)| (a.index() as u32, b.index() as u32))
                .collect(),
            poi_snaps,
            hierarchy: self.hierarchy_slot().get().cloned(),
        }
    }

    fn restore(saved: SavedGraph) -> Result<Self, OsmGraphError> {
        let node_count = saved.nodes.len();
        let edge_count = saved.edges.len();
        let tag_sets: Vec<Arc<[OsmTag]>> = saved.tag_sets.into_iter().map(Arc::from).collect();

        let mut graph: RoadGraph = DiGraph::with_capacity(node_count, edge_count);
        for node in saved.nodes {
            graph.add_node(node);
        }
        for e in saved.edges {
            if e.source as usize >= node_count || e.target as usize >= node_count {
                return Err(invalid("edge refers to a missing node"));
            }
            let tags = tag_sets
                .get(e.tag_set as usize)
                .ok_or_else(|| invalid("edge refers to a missing tag set"))?;
            graph.add_edge(
                NodeIndex::new(e.source as usize),
                NodeIndex::new(e.target as usize),
                Edge {
                    way_id: e.way_id,
                    last_way_id: e.last_way_id,
                    tags: Arc::clone(tags),
                    length: e.length,
                    speed_kph: e.speed_kph,
                    walk_travel_time: e.times[0],
                    bike_travel_time: e.times[1],
                    drive_travel_time: e.times[2],
                    geometry: e.geometry,
                },
            );
        }
        let in_range = |edge: u32| (edge as usize) < edge_count;
        if !saved
            .forbidden_turns
            .iter()
            .all(|&(a, b)| in_range(a) && in_range(b))
        {
            return Err(invalid("forbidden turn refers to a missing edge"));
        }
        let turns = saved
            .forbidden_turns
            .iter()
            .map(|&(a, b)| (EdgeIndex::new(a as usize), EdgeIndex::new(b as usize)))
            .collect();

        let mut sg = SpatialGraph::with_forbidden_turns(graph, saved.network_type, turns);
        if let Some(snaps) = saved.poi_snaps {
            let mut by_id = HashMap::with_capacity(snaps.len());
            for s in snaps {
                if s.node as usize >= node_count || s.edge.is_some_and(|e| !in_range(e)) {
                    return Err(invalid("POI snap refers to a missing node or edge"));
                }
                let node_index = NodeIndex::new(s.node as usize);
                let node = &sg.graph[node_index];
                let snap = SnapResult {
                    input_lat: s.input.0,
                    input_lon: s.input.1,
                    snapped_lat: s.snapped.0,
                    snapped_lon: s.snapped.1,
                    distance_m: s.distance_m,
                    edge: s.edge.map(|e| EdgeIndex::new(e as usize)),
                    fraction: s.fraction,
                    node_index,
                    node_id: node.id,
                    node_lat: node.lat,
                    node_lon: node.lon,
                };
                by_id.insert(
                    s.poi_id,
                    SnappedPoi {
                        poi_id: s.poi_id,
                        snap,
                    },
                );
            }
            sg.poi_snaps = Some(Arc::new(by_id));
        }
        if let Some(hierarchy) = saved.hierarchy {
            if hierarchy.node_count() != sg.search_index().node_count() {
                return Err(invalid("routing hierarchy does not match the graph"));
            }
            let _ = sg.hierarchy_slot().set(hierarchy);
        }
        Ok(sg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("graphways-{}-{name}", std::process::id()))
    }

    #[test]
    fn saved_graph_round_trips_with_identical_routes() {
        for network_type in [NetworkType::Drive, NetworkType::Walk] {
            let sg = SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", network_type, false)
                .unwrap();
            sg.prepare_routing();
            let path = temp_path(&format!("{network_type:?}.graph"));
            sg.save(&path).unwrap();
            let loaded = SpatialGraph::load(&path).unwrap();
            std::fs::remove_file(&path).ok();

            assert_eq!(loaded.network_type(), network_type);
            assert!(loaded.is_routing_prepared());
            assert_eq!(loaded.graph.node_count(), sg.graph.node_count());
            assert_eq!(loaded.graph.edge_count(), sg.graph.edge_count());
            assert_eq!(
                loaded.poi_snaps.as_ref().map(|p| p.len()),
                sg.poi_snaps.as_ref().map(|p| p.len())
            );
            for origin in sg.graph.node_weights() {
                for destination in sg.graph.node_weights() {
                    let (a, b) = ((origin.lat, origin.lon), (destination.lat, destination.lon));
                    let before = sg
                        .route(a, b, None)
                        .ok()
                        .map(|r| (r.duration_s, r.coordinates));
                    let after = loaded
                        .route(a, b, None)
                        .ok()
                        .map(|r| (r.duration_s, r.coordinates));
                    assert_eq!(before, after);
                }
            }
            let distinct_tag_sets = |g: &SpatialGraph| {
                g.graph
                    .edge_weights()
                    .map(|e| Arc::as_ptr(&e.tags) as *const u8 as usize)
                    .collect::<std::collections::HashSet<_>>()
                    .len()
            };
            assert!(
                distinct_tag_sets(&loaded) <= distinct_tag_sets(&sg),
                "tag sets stay shared between edges"
            );
        }
    }

    #[test]
    fn restricted_graph_round_trips() {
        let xml = r#"<osm>
          <node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="0.001"/>
          <node id="3" lat="0.001" lon="0.001"/><node id="4" lat="0" lon="0.002"/>
          <way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
          <way id="20"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
          <way id="30"><nd ref="2"/><nd ref="4"/><nd ref="3"/><tag k="highway" v="residential"/></way>
          <relation id="9"><member type="way" ref="10" role="from"/>
            <member type="node" ref="2" role="via"/><member type="way" ref="20" role="to"/>
            <tag k="type" v="restriction"/><tag k="restriction" v="no_left_turn"/></relation>
        </osm>"#;
        let sg = SpatialGraph::from_osm(xml, NetworkType::Drive, false).unwrap();
        sg.prepare_routing();
        let path = temp_path("restricted.graph");
        sg.save(&path).unwrap();
        let loaded = SpatialGraph::load(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(loaded.forbidden_turns(), sg.forbidden_turns());
        assert!(!loaded.forbidden_turns().is_empty());
        let route = |g: &SpatialGraph| {
            g.route((0.0, 0.0), (0.001, 0.001), None)
                .unwrap()
                .duration_s
        };
        assert_eq!(route(&loaded), route(&sg));
    }

    #[test]
    fn rejects_foreign_and_truncated_files() {
        let path = temp_path("bogus.graph");
        std::fs::write(&path, b"not a graph").unwrap();
        assert!(matches!(
            SpatialGraph::load(&path),
            Err(OsmGraphError::InvalidGraphFile(_))
        ));

        let sg =
            SpatialGraph::from_pbf("tests/fixtures/tiny_map.osm.pbf", NetworkType::Walk, false)
                .unwrap();
        sg.save(&path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        std::fs::write(&path, &bytes[..bytes.len() / 2]).unwrap();
        assert!(matches!(
            SpatialGraph::load(&path),
            Err(OsmGraphError::InvalidGraphFile(_))
        ));
        let mut wrong_version = bytes.clone();
        wrong_version[8] = 99;
        std::fs::write(&path, &wrong_version).unwrap();
        let err = SpatialGraph::load(&path).err().unwrap();
        assert!(err.to_string().contains("format version 99"), "{err}");
        std::fs::remove_file(&path).ok();
    }
}
