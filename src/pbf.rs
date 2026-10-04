//! Read OpenStreetMap PBF files into the same intermediate shape produced by
//! the Overpass XML parser. This lets the rest of the pipeline (graph building,
//! POI extraction) work unchanged whether the data came from live Overpass or
//! a local PBF file.
//!
//! The file is read in a single pass with blobs decoded in parallel. Tags are
//! filtered while still borrowed from the decoded block, so only kept roads,
//! POIs and tagged nodes allocate strings; every node otherwise costs one
//! `(id, lat, lon)` triple. Output is sorted by OSM id, which makes graph
//! construction (and therefore simplification) deterministic.

use std::collections::HashMap;
use std::path::Path;

use osmpbf::{BlobDecode, BlobReader, Element, PrimitiveBlock, RelMemberType};
use rayon::prelude::*;

use crate::error::OsmGraphError;
use crate::filters::{is_poi_node, way_passes_road_filter};
use crate::graph::{
    OsmData, OsmMember, OsmNode, OsmNodeRef, OsmRelation, OsmTag, OsmWay, SpatialGraph,
};
use crate::overpass::NetworkType;
use crate::poi::Poi;
use crate::profile::BuildOptions;

impl SpatialGraph {
    /// Build a routable [`SpatialGraph`] directly from a local OSM PBF file.
    ///
    /// POIs are parsed separately from road-network nodes and pre-snapped onto
    /// the graph. Use [`read_pbf`] when you need access to the intermediate
    /// [`OsmData`] or raw [`Poi`] list.
    pub fn from_pbf(
        path: impl AsRef<Path>,
        network_type: NetworkType,
        retain_all: bool,
    ) -> Result<Self, OsmGraphError> {
        Self::from_pbf_with(path, network_type, &BuildOptions::retain_all(retain_all))
    }

    /// Like [`SpatialGraph::from_pbf`] with a custom speed profile.
    pub fn from_pbf_with(
        path: impl AsRef<Path>,
        network_type: NetworkType,
        options: &BuildOptions,
    ) -> Result<Self, OsmGraphError> {
        options.profile.validate()?;
        let (data, pois) = read_pbf(path, network_type)?;
        let mut spatial_graph = SpatialGraph::from_osm_data_with(data, network_type, options)?;
        spatial_graph.snap_pois(&pois);
        Ok(spatial_graph)
    }
}

/// Read a PBF file once and produce one `OsmData` per requested network type,
/// plus the POIs found in the extract (POIs are network-type-independent).
///
/// This avoids re-reading the PBF for each network type -- useful at server
/// startup when you want walk/bike/drive graphs for the same region.
pub fn read_pbf_multi(
    path: impl AsRef<Path>,
    network_types: &[NetworkType],
) -> Result<(HashMap<NetworkType, OsmData>, Vec<Poi>), OsmGraphError> {
    let scan = scan_pbf(path.as_ref(), network_types)?;
    let data = network_types
        .iter()
        .zip(&scan.roads)
        .map(|(&network_type, roads)| (network_type, scan.xml_data(roads)))
        .collect();
    Ok((data, scan.pois))
}

/// Read a PBF file and produce an `OsmData` (the canonical intermediate shape
/// our graph builder consumes) plus the POIs found in the extract.
///
/// `OsmData` holds only road-network nodes (nodes referenced by a way that
/// passes the `network_type` road filter), sorted by id. POIs are returned
/// separately as [`Poi`] values, also sorted by id, and can be snapped onto a
/// [`crate::graph::SpatialGraph`] afterward.
pub fn read_pbf(
    path: impl AsRef<Path>,
    network_type: NetworkType,
) -> Result<(OsmData, Vec<Poi>), OsmGraphError> {
    let scan = scan_pbf(path.as_ref(), &[network_type])?;
    Ok((scan.xml_data(&scan.roads[0]), scan.pois))
}

type OwnedTags = Vec<(String, String)>;

/// Everything kept from one pass over a PBF file.
#[derive(Default)]
struct PbfScan {
    /// `(id, lat, lon)` of every node, sorted by id.
    coords: Vec<(i64, f64, f64)>,
    /// Tags of every tagged node, sorted by id.
    node_tags: Vec<(i64, OwnedTags)>,
    /// POI nodes, sorted by id.
    pois: Vec<Poi>,
    /// Ways passing the road filter, one list per requested network type.
    roads: Vec<Vec<RawWay>>,
    /// Turn-restriction relations.
    restrictions: Vec<OsmRelation>,
}

struct RawWay {
    id: i64,
    refs: Vec<i64>,
    tags: OwnedTags,
}

fn pbf_error(error: impl ToString) -> OsmGraphError {
    OsmGraphError::PbfError(error.to_string())
}

fn scan_pbf(path: &Path, network_types: &[NetworkType]) -> Result<PbfScan, OsmGraphError> {
    let mut chunks: Vec<(usize, PbfScan)> = BlobReader::from_path(path)
        .map_err(pbf_error)?
        .enumerate()
        .par_bridge()
        .map(|(position, blob)| {
            let blob = blob.map_err(pbf_error)?;
            let chunk = match blob.decode().map_err(pbf_error)? {
                BlobDecode::OsmData(block) => scan_block(&block, network_types),
                BlobDecode::OsmHeader(_) | BlobDecode::Unknown(_) => PbfScan::default(),
            };
            Ok((position, chunk))
        })
        .collect::<Result<_, OsmGraphError>>()?;
    // Reassemble in file order so the output does not depend on scheduling.
    chunks.sort_unstable_by_key(|(position, _)| *position);

    let mut scan = PbfScan {
        roads: network_types.iter().map(|_| Vec::new()).collect(),
        ..PbfScan::default()
    };
    for (_, chunk) in chunks {
        scan.coords.extend(chunk.coords);
        scan.node_tags.extend(chunk.node_tags);
        scan.pois.extend(chunk.pois);
        scan.restrictions.extend(chunk.restrictions);
        for (all, part) in scan.roads.iter_mut().zip(chunk.roads) {
            all.extend(part);
        }
    }

    // Sorted extracts (the norm) are already in id order; this is a cheap check.
    if !scan.coords.is_sorted_by_key(|&(id, _, _)| id) {
        scan.coords.sort_by_key(|&(id, _, _)| id);
        scan.node_tags.sort_by_key(|(id, _)| *id);
        scan.pois.sort_by_key(|poi| poi.id);
    }
    Ok(scan)
}

fn scan_block(block: &PrimitiveBlock, network_types: &[NetworkType]) -> PbfScan {
    let mut chunk = PbfScan {
        roads: network_types.iter().map(|_| Vec::new()).collect(),
        ..PbfScan::default()
    };
    let mut tags: Vec<(&str, &str)> = Vec::new();
    let to_owned = |tags: &[(&str, &str)]| -> OwnedTags {
        tags.iter()
            .map(|&(k, v)| (k.to_owned(), v.to_owned()))
            .collect()
    };

    for element in block.elements() {
        let (id, lat, lon) = match &element {
            Element::Node(node) => {
                tags.clear();
                tags.extend(node.tags());
                (node.id(), node.lat(), node.lon())
            }
            Element::DenseNode(node) => {
                tags.clear();
                tags.extend(node.tags());
                (node.id(), node.lat(), node.lon())
            }
            Element::Way(way) => {
                tags.clear();
                tags.extend(way.tags());
                // Quick reject: ways without a highway tag aren't roads for any mode.
                if !tags.iter().any(|&(k, _)| k == "highway") {
                    continue;
                }
                let mut refs: Option<Vec<i64>> = None;
                for (roads, &network_type) in chunk.roads.iter_mut().zip(network_types) {
                    if way_passes_road_filter(&tags, network_type) {
                        roads.push(RawWay {
                            id: way.id(),
                            refs: refs.get_or_insert_with(|| way.refs().collect()).clone(),
                            tags: to_owned(&tags),
                        });
                    }
                }
                continue;
            }
            Element::Relation(relation) => {
                let is_restriction = relation
                    .tags()
                    .any(|(k, v)| k == "type" && v == "restriction");
                if is_restriction {
                    chunk.restrictions.push(OsmRelation {
                        id: relation.id(),
                        members: relation
                            .members()
                            .map(|m| OsmMember {
                                kind: match m.member_type {
                                    RelMemberType::Node => "node",
                                    RelMemberType::Way => "way",
                                    RelMemberType::Relation => "relation",
                                }
                                .to_owned(),
                                reference: m.member_id,
                                role: m.role().unwrap_or_default().to_owned(),
                            })
                            .collect(),
                        tags: relation
                            .tags()
                            .map(|(k, v)| OsmTag {
                                key: k.to_owned(),
                                value: v.to_owned(),
                            })
                            .collect(),
                    });
                }
                continue;
            }
        };

        chunk.coords.push((id, lat, lon));
        if tags.is_empty() {
            continue;
        }
        if is_poi_node(&tags) {
            chunk.pois.push(Poi {
                id,
                lat,
                lon,
                tags: tags
                    .iter()
                    .map(|&(k, v)| (k.to_owned(), v.to_owned()))
                    .collect(),
            });
        }
        chunk.node_tags.push((id, to_owned(&tags)));
    }
    chunk
}

impl PbfScan {
    /// Assemble the road nodes and ways for one network type's road list.
    fn xml_data(&self, roads: &[RawWay]) -> OsmData {
        let mut needed: Vec<i64> = roads.iter().flat_map(|w| w.refs.iter().copied()).collect();
        needed.sort_unstable();
        needed.dedup();

        // Merge-join the sorted id lists; refs to nodes absent from the
        // extract are dropped here and skipped by the graph builder.
        let mut tags = self.node_tags.iter().peekable();
        let nodes = needed
            .iter()
            .filter_map(|&id| {
                let index = self.coords.binary_search_by_key(&id, |&(id, ..)| id).ok()?;
                let (_, lat, lon) = self.coords[index];
                while tags.next_if(|(tag_id, _)| *tag_id < id).is_some() {}
                let tags = tags
                    .next_if(|(tag_id, _)| *tag_id == id)
                    .map(|(_, tags)| to_xml_tags(tags))
                    .unwrap_or_default();
                Some(OsmNode { id, lat, lon, tags })
            })
            .collect();

        let ways = roads
            .iter()
            .map(|way| OsmWay {
                id: way.id,
                nodes: way
                    .refs
                    .iter()
                    .map(|&node_id| OsmNodeRef { node_id })
                    .collect(),
                tags: to_xml_tags(&way.tags),
            })
            .collect();

        OsmData {
            nodes,
            ways,
            relations: self.restrictions.clone(),
        }
    }
}

fn to_xml_tags(tags: &[(String, String)]) -> Vec<OsmTag> {
    tags.iter()
        .map(|(key, value)| OsmTag {
            key: key.clone(),
            value: value.clone(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::parse_xml;

    const TINY_PBF: &str = "tests/fixtures/tiny_map.osm.pbf";
    const TINY_DRIVE_XML: &str = include_str!("../tests/fixtures/tiny_drive_overpass.osm");

    fn sorted_ids<T, F>(items: &[T], id: F) -> Vec<i64>
    where
        F: FnMut(&T) -> i64,
    {
        let mut ids: Vec<i64> = items.iter().map(id).collect();
        ids.sort_unstable();
        ids
    }

    fn way_tag_value(way: &OsmWay, key: &str) -> Option<String> {
        way.tags
            .iter()
            .find(|tag| tag.key == key)
            .map(|tag| tag.value.clone())
    }

    #[test]
    fn read_pbf_output_is_sorted_and_repeatable() {
        let (first, first_pois) = read_pbf(TINY_PBF, NetworkType::Walk).unwrap();
        let (second, second_pois) = read_pbf(TINY_PBF, NetworkType::Walk).unwrap();

        let ids: Vec<i64> = first.nodes.iter().map(|node| node.id).collect();
        assert!(ids.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(ids, second.nodes.iter().map(|n| n.id).collect::<Vec<_>>());
        assert_eq!(
            first_pois.iter().map(|p| p.id).collect::<Vec<_>>(),
            second_pois.iter().map(|p| p.id).collect::<Vec<_>>()
        );
    }

    #[test]
    fn tiny_pbf_drive_profile_filters_roads_and_separates_pois() {
        let (data, pois) = read_pbf(TINY_PBF, NetworkType::Drive).unwrap();

        assert_eq!(sorted_ids(&data.nodes, |node| node.id), vec![1, 2, 3, 4]);
        assert_eq!(sorted_ids(&data.ways, |way| way.id), vec![10, 20]);
        assert_eq!(sorted_ids(&pois, |poi| poi.id), vec![100]);
        assert_eq!(pois[0].tags["amenity"], "cafe");
    }

    #[test]
    fn tiny_pbf_profile_differences_are_visible() {
        let (drive, _) = read_pbf(TINY_PBF, NetworkType::Drive).unwrap();
        let (drive_service, _) = read_pbf(TINY_PBF, NetworkType::DriveService).unwrap();
        let (walk, _) = read_pbf(TINY_PBF, NetworkType::Walk).unwrap();

        assert_eq!(sorted_ids(&drive.ways, |way| way.id), vec![10, 20]);
        assert_eq!(
            sorted_ids(&drive_service.ways, |way| way.id),
            vec![10, 20, 30]
        );
        assert_eq!(sorted_ids(&walk.ways, |way| way.id), vec![10, 20, 30, 40]);
    }

    #[test]
    fn tiny_pbf_drive_matches_expected_overpass_xml_shape() {
        let (pbf_data, _) = read_pbf(TINY_PBF, NetworkType::Drive).unwrap();
        let xml_data = parse_xml(TINY_DRIVE_XML).unwrap();

        assert_eq!(
            sorted_ids(&pbf_data.nodes, |node| node.id),
            sorted_ids(&xml_data.nodes, |node| node.id)
        );
        assert_eq!(
            sorted_ids(&pbf_data.ways, |way| way.id),
            sorted_ids(&xml_data.ways, |way| way.id)
        );

        for expected in &xml_data.ways {
            let actual = pbf_data
                .ways
                .iter()
                .find(|way| way.id == expected.id)
                .unwrap();
            let actual_refs: Vec<i64> = actual.nodes.iter().map(|node| node.node_id).collect();
            let expected_refs: Vec<i64> = expected.nodes.iter().map(|node| node.node_id).collect();
            assert_eq!(actual_refs, expected_refs);
            assert_eq!(
                way_tag_value(actual, "highway"),
                way_tag_value(expected, "highway")
            );
            assert_eq!(
                way_tag_value(actual, "oneway"),
                way_tag_value(expected, "oneway")
            );
            assert_eq!(
                way_tag_value(actual, "maxspeed"),
                way_tag_value(expected, "maxspeed")
            );
        }
    }

    #[test]
    fn tiny_pbf_multi_matches_single_profile_reads() {
        let (multi, pois) =
            read_pbf_multi(TINY_PBF, &[NetworkType::Drive, NetworkType::Walk]).unwrap();
        let (drive, drive_pois) = read_pbf(TINY_PBF, NetworkType::Drive).unwrap();
        let (walk, _) = read_pbf(TINY_PBF, NetworkType::Walk).unwrap();

        assert_eq!(
            sorted_ids(&pois, |poi| poi.id),
            sorted_ids(&drive_pois, |poi| poi.id)
        );
        assert_eq!(
            sorted_ids(&multi[&NetworkType::Drive].ways, |way| way.id),
            sorted_ids(&drive.ways, |way| way.id)
        );
        assert_eq!(
            sorted_ids(&multi[&NetworkType::Walk].ways, |way| way.id),
            sorted_ids(&walk.ways, |way| way.id)
        );
    }
}
