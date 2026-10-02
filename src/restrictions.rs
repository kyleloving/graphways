//! Turn restrictions (`type=restriction` relations) for driving.
//!
//! A restriction names a `from` way, a `via` node and a `to` way, and either
//! forbids that turn (`no_left_turn`, `no_u_turn`, ...) or makes it the only
//! one allowed (`only_straight_on`, ...). Restrictions are resolved against
//! the built graph into forbidden `(incoming edge, outgoing edge)` pairs at
//! the via node; the search index then splits the node into states so every
//! search respects them. Restrictions whose via member is a way, and ones
//! that exempt cars (`except=motorcar`), are skipped.

use std::collections::{HashMap, HashSet};

use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction;

use crate::graph::{OsmRelation, RoadGraph};
use crate::overpass::NetworkType;

/// Whether turn restrictions are applied to graphs of `network_type`.
pub fn applies_to(network_type: NetworkType) -> bool {
    matches!(network_type, NetworkType::Drive | NetworkType::DriveService)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestrictionKind {
    /// The turn is forbidden.
    No,
    /// Only this turn is allowed from the `from` way at the via node.
    Only,
}

/// A via-node turn restriction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TurnRestriction {
    pub from_way: i64,
    pub via_node: i64,
    pub to_way: i64,
    pub kind: RestrictionKind,
}

impl TurnRestriction {
    /// Read a restriction relation, if it is one that applies to cars and
    /// has exactly one `from` way, one `via` node and one `to` way.
    pub fn from_relation(relation: &OsmRelation) -> Option<Self> {
        let tag = |key: &str| {
            relation
                .tags
                .iter()
                .find(|t| t.key == key)
                .map(|t| t.value.as_str())
        };
        if tag("type") != Some("restriction") {
            return None;
        }
        if tag("except").is_some_and(|e| e.split(';').any(|v| v.trim() == "motorcar")) {
            return None;
        }
        let value = tag("restriction:motorcar").or_else(|| tag("restriction"))?;
        let kind = if value.starts_with("no_") {
            RestrictionKind::No
        } else if value.starts_with("only_") {
            RestrictionKind::Only
        } else {
            return None;
        };

        let single = |kind: &str, role: &str| {
            let mut matching = relation
                .members
                .iter()
                .filter(|m| m.role == role)
                .map(|m| (m.kind.as_str(), m.reference));
            match (matching.next(), matching.next()) {
                (Some((k, id)), None) if k == kind => Some(id),
                _ => None,
            }
        };
        Some(TurnRestriction {
            from_way: single("way", "from")?,
            via_node: single("node", "via")?,
            to_way: single("way", "to")?,
            kind,
        })
    }
}

/// The restrictions among `relations`.
pub fn parse_restrictions(relations: &[OsmRelation]) -> Vec<TurnRestriction> {
    relations
        .iter()
        .filter_map(TurnRestriction::from_relation)
        .collect()
}

/// OSM ids of every via node, which simplification must keep intact.
pub(crate) fn via_nodes(restrictions: &[TurnRestriction]) -> HashSet<i64> {
    restrictions.iter().map(|r| r.via_node).collect()
}

/// Resolve restrictions against `graph` into forbidden `(into via, out of
/// via)` edge pairs, sorted. An incoming edge is matched by the way of its
/// last segment and an outgoing edge by the way of its first, so collapsed
/// chains of road still match. Restrictions that reference roads or nodes
/// missing from the graph are ignored.
pub fn forbidden_turns(
    graph: &RoadGraph,
    restrictions: &[TurnRestriction],
) -> Vec<(EdgeIndex, EdgeIndex)> {
    if restrictions.is_empty() {
        return Vec::new();
    }
    let wanted = via_nodes(restrictions);
    let via_index: HashMap<i64, NodeIndex> = graph
        .node_indices()
        .filter(|&n| wanted.contains(&graph[n].id))
        .map(|n| (graph[n].id, n))
        .collect();

    let mut forbidden = Vec::new();
    for restriction in restrictions {
        let Some(&via) = via_index.get(&restriction.via_node) else {
            continue;
        };
        let incoming = graph
            .edges_directed(via, Direction::Incoming)
            .filter(|e| e.weight().last_way_id == restriction.from_way);
        for into in incoming {
            for out in graph.edges(via) {
                let onto_target_way = out.weight().way_id == restriction.to_way;
                let banned = match restriction.kind {
                    // A u-turn ban (from == to) forbids only going back the
                    // way you came, not carrying on along the same way.
                    RestrictionKind::No if restriction.from_way == restriction.to_way => {
                        onto_target_way && out.target() == into.source()
                    }
                    RestrictionKind::No => onto_target_way,
                    RestrictionKind::Only => !onto_target_way,
                };
                if banned {
                    forbidden.push((into.id(), out.id()));
                }
            }
        }
    }
    forbidden.sort_unstable();
    forbidden.dedup();
    forbidden
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{OsmMember, OsmTag};

    fn relation(tags: &[(&str, &str)], members: &[(&str, i64, &str)]) -> OsmRelation {
        OsmRelation {
            id: 1,
            members: members
                .iter()
                .map(|&(kind, reference, role)| OsmMember {
                    kind: kind.into(),
                    reference,
                    role: role.into(),
                })
                .collect(),
            tags: tags
                .iter()
                .map(|&(key, value)| OsmTag {
                    key: key.into(),
                    value: value.into(),
                })
                .collect(),
        }
    }

    const MEMBERS: &[(&str, i64, &str)] =
        &[("way", 10, "from"), ("node", 5, "via"), ("way", 20, "to")];

    #[test]
    fn parses_no_and_only_restrictions() {
        let no = relation(
            &[("type", "restriction"), ("restriction", "no_left_turn")],
            MEMBERS,
        );
        let only = relation(
            &[("type", "restriction"), ("restriction", "only_straight_on")],
            MEMBERS,
        );

        assert_eq!(
            TurnRestriction::from_relation(&no),
            Some(TurnRestriction {
                from_way: 10,
                via_node: 5,
                to_way: 20,
                kind: RestrictionKind::No
            })
        );
        assert_eq!(
            TurnRestriction::from_relation(&only).map(|r| r.kind),
            Some(RestrictionKind::Only)
        );
    }

    #[test]
    fn skips_car_exemptions_via_ways_and_other_relations() {
        let exempt = relation(
            &[
                ("type", "restriction"),
                ("restriction", "no_left_turn"),
                ("except", "bus;motorcar"),
            ],
            MEMBERS,
        );
        let via_way = relation(
            &[("type", "restriction"), ("restriction", "no_u_turn")],
            &[("way", 10, "from"), ("way", 15, "via"), ("way", 20, "to")],
        );
        let route = relation(&[("type", "route")], MEMBERS);
        assert!(parse_restrictions(&[exempt, via_way, route]).is_empty());
    }

    /// W(1) ── V(2) ── E(4)
    ///          │      ╱
    ///         N(3) ──╯      (way 40 links E back to N)
    fn junction_xml(restriction: &str) -> String {
        format!(
            r#"<osm>
              <node id="1" lat="0" lon="0"/>
              <node id="2" lat="0" lon="0.001"/>
              <node id="3" lat="0.001" lon="0.001"/>
              <node id="4" lat="0" lon="0.002"/>
              <way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
              <way id="20"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
              <way id="30"><nd ref="2"/><nd ref="4"/><tag k="highway" v="residential"/></way>
              <way id="40"><nd ref="4"/><nd ref="3"/><tag k="highway" v="residential"/></way>
              <relation id="99">
                <member type="way" ref="10" role="from"/>
                <member type="node" ref="2" role="via"/>
                <member type="way" ref="{to}" role="to"/>
                <tag k="type" v="restriction"/>
                <tag k="restriction" v="{restriction}"/>
              </relation>
            </osm>"#,
            to = if restriction.starts_with("only_") {
                30
            } else {
                20
            },
        )
    }

    /// Whether the route turns from way `from` straight onto way `to`.
    fn turns(
        sg: &crate::graph::SpatialGraph,
        route: &crate::routing::Route,
        from: i64,
        to: i64,
    ) -> bool {
        route
            .pieces
            .windows(2)
            .any(|w| sg.graph[w[0].edge].last_way_id == from && sg.graph[w[1].edge].way_id == to)
    }

    #[test]
    fn restricted_turns_are_avoided_by_every_search() {
        for restriction in ["no_left_turn", "only_straight_on"] {
            for network in [NetworkType::Drive, NetworkType::Walk] {
                let sg = crate::graph::SpatialGraph::from_osm(
                    &junction_xml(restriction),
                    network,
                    false,
                )
                .unwrap();
                let applies = applies_to(network);
                assert_eq!(
                    sg.forbidden_turns().is_empty(),
                    !applies,
                    "{restriction} {network:?}"
                );

                let unprepared = sg.route((0.0, 0.0), (0.001, 0.001), None).unwrap();
                sg.prepare_routing();
                let prepared = sg.route((0.0, 0.0), (0.001, 0.001), None).unwrap();
                for route in [&unprepared, &prepared] {
                    assert_eq!(
                        turns(&sg, route, 10, 20),
                        !applies,
                        "{restriction} {network:?}: turn W→V→N"
                    );
                }
                assert!((unprepared.duration_s - prepared.duration_s).abs() < 1e-9);

                // Coming from the other direction the turn is allowed.
                let other = sg.route((0.0, 0.002), (0.001, 0.001), None).unwrap();
                assert!(other.duration_s <= unprepared.duration_s + 1e-9);
            }
        }
    }

    #[test]
    fn reachability_respects_restrictions() {
        let sg = crate::graph::SpatialGraph::from_osm(
            &junction_xml("no_left_turn"),
            NetworkType::Drive,
            false,
        )
        .unwrap();
        let free = crate::graph::SpatialGraph::from_osm(
            &junction_xml("no_left_turn"),
            NetworkType::Drive,
            false,
        )
        .map(|g| crate::graph::SpatialGraph::new((*g.graph).clone(), NetworkType::Drive))
        .unwrap();
        let time_to_n = |g: &crate::graph::SpatialGraph| {
            let result = g.reachability((0.0, 0.0), f64::INFINITY, None).unwrap();
            result
                .time_to(g, &g.snap_point((0.001, 0.001)).unwrap())
                .unwrap()
        };
        assert!(time_to_n(&sg) > time_to_n(&free) + 1.0);
    }

    #[test]
    fn u_turn_ban_only_blocks_going_back() {
        // A through road 1 → 2 → 3 (way 10) with no_u_turn at 2.
        let xml = r#"<osm>
          <node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="0.001"/><node id="3" lat="0" lon="0.002"/>
          <node id="4" lat="0.001" lon="0.001"/>
          <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
          <way id="20"><nd ref="2"/><nd ref="4"/><tag k="highway" v="residential"/></way>
          <relation id="99">
            <member type="way" ref="10" role="from"/><member type="node" ref="2" role="via"/>
            <member type="way" ref="10" role="to"/>
            <tag k="type" v="restriction"/><tag k="restriction" v="no_u_turn"/>
          </relation></osm>"#;
        let sg = crate::graph::SpatialGraph::from_osm(xml, NetworkType::Drive, false).unwrap();
        let turns: Vec<(i64, i64, i64, i64)> = sg
            .forbidden_turns()
            .iter()
            .map(|&(a, b)| {
                let (a0, a1) = sg.graph.edge_endpoints(a).unwrap();
                let (b0, b1) = sg.graph.edge_endpoints(b).unwrap();
                (
                    sg.graph[a0].id,
                    sg.graph[a1].id,
                    sg.graph[b0].id,
                    sg.graph[b1].id,
                )
            })
            .collect();
        let mut turns = turns;
        turns.sort_unstable();
        assert_eq!(turns, vec![(1, 2, 2, 1), (3, 2, 2, 3)]);
    }
}
