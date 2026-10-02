import json
import tempfile
import unittest
from pathlib import Path

import graphways as gw


FIXTURES = Path(__file__).parent / "fixtures"


class PythonApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xml = (FIXTURES / "tiny_map.osm").read_text(encoding="utf-8")
        cls.graph = gw.SpatialGraph.from_osm(xml, "walk")

    def test_route_returns_structured_result(self):
        route = self.graph.route((48.0, 11.0), (48.001, 11.0))

        self.assertEqual(type(route).__name__, "RouteResult")
        self.assertGreater(route.distance_m, 0)
        self.assertGreater(route.duration_s, 0)
        self.assertGreaterEqual(len(route.coordinates), 2)
        self.assertEqual(route.cumulative_times_s[0], 0)
        self.assertEqual(type(route.origin_snap).__name__, "SnapResult")
        self.assertEqual(type(route.destination_snap).__name__, "SnapResult")

        geojson = json.loads(route.to_geojson())
        self.assertEqual(geojson["type"], "Feature")
        self.assertEqual(geojson["geometry"]["type"], "LineString")
        self.assertIn("origin_snap", geojson["properties"])

    def test_isochrone_returns_structured_results(self):
        isochrones = self.graph.isochrone((48.0, 11.0), [1, 3])

        self.assertEqual([iso.minutes for iso in isochrones], [1.0, 3.0])
        self.assertTrue(all(type(iso).__name__ == "IsochroneResult" for iso in isochrones))
        geojson = json.loads(isochrones[0].to_geojson())
        self.assertEqual(geojson["type"], "MultiPolygon")
        self.assertEqual(isochrones[0].__geo_interface__, geojson)

    def test_snap_point_returns_structured_result(self):
        snap = self.graph.snap_point(48.0, 11.0)

        self.assertEqual(type(snap).__name__, "SnapResult")
        self.assertEqual(snap.node_id, 1)
        self.assertAlmostEqual(snap.distance_m, 0.0)
        self.assertEqual(snap.as_dict()["node_id"], 1)

    def test_graph_views_return_structured_route_and_isochrones(self):
        reachable = self.graph.reachable((48.0, 11.0), minutes=5)
        route = reachable.route((48.0, 11.0), (48.001, 11.0))
        isochrone = reachable.isochrone((48.0, 11.0), [3])[0]

        self.assertEqual(type(route).__name__, "RouteResult")
        self.assertEqual(type(isochrone).__name__, "IsochroneResult")

        prism = self.graph.prism((48.0, 11.0), (48.001, 11.0), max_minutes=8)
        prism_route = prism.route((48.0, 11.0), (48.001, 11.0))
        prism_iso = prism.isochrone((48.0, 11.0), [3])[0]

        self.assertEqual(type(prism_route).__name__, "RouteResult")
        self.assertEqual(type(prism_iso).__name__, "IsochroneResult")

    def test_default_max_snap_rejects_far_coordinates(self):
        with self.assertRaises(LookupError):
            self.graph.route((47.999, 11.0), (48.001, 11.0))

        route = self.graph.route((47.999, 11.0), (48.001, 11.0), max_snap_m=None)
        self.assertEqual(type(route).__name__, "RouteResult")
        self.assertGreater(route.origin_snap.distance_m, 100.0)

    def test_snap_lands_on_the_road_between_nodes(self):
        snap = self.graph.snap_point(48.0005, 11.0001)

        self.assertLess(snap.distance_m, 10.0)
        self.assertAlmostEqual(snap.snapped_lat, 48.0005, places=5)
        self.assertIn("snapped_lon", snap.as_dict())

    def test_results_expose_geo_interface(self):
        route = self.graph.route((48.0, 11.0), (48.001, 11.0))
        self.assertEqual(route.__geo_interface__["geometry"]["type"], "LineString")

    def test_prepare_routing_gives_identical_routes(self):
        xml = (FIXTURES / "tiny_map.osm").read_text(encoding="utf-8")
        graph = gw.SpatialGraph.from_osm(xml, "walk")
        before = graph.route((48.0, 11.0), (48.001, 11.0))
        graph.prepare_routing()
        self.assertTrue(graph.is_routing_prepared())
        after = graph.route((48.0, 11.0), (48.001, 11.0))
        self.assertAlmostEqual(before.duration_s, after.duration_s)
        self.assertEqual(before.coordinates, after.coordinates)

    def test_saved_graph_loads_with_identical_routes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tiny.graphways"
            self.graph.save(str(path))
            loaded = gw.SpatialGraph.load(path)

        self.assertTrue(loaded.is_routing_prepared())
        self.assertEqual(loaded.node_count(), self.graph.node_count())
        self.assertEqual(loaded.edge_count(), self.graph.edge_count())
        before = self.graph.route((48.0, 11.0), (48.001, 11.0))
        after = loaded.route((48.0, 11.0), (48.001, 11.0))
        self.assertEqual(after.coordinates, before.coordinates)
        self.assertEqual(after.duration_s, before.duration_s)

    def test_loading_a_foreign_file_raises_value_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "not-a-graph"
            path.write_bytes(b"hello world, definitely not a graph")
            with self.assertRaises(ValueError):
                gw.SpatialGraph.load(path)
            with self.assertRaises(OSError):
                gw.SpatialGraph.load(Path(tmp) / "missing")

    def test_travel_time_matrix_matches_routes(self):
        points = [(48.0, 11.0), (48.001, 11.0), (48.0005, 11.0), (10.0, 10.0)]
        matrix = self.graph.travel_time_matrix(points, points[:3])

        self.assertEqual(matrix.shape, (4, 3))
        self.assertEqual(len(matrix.durations_s), 4)
        self.assertIsNone(matrix.origin_snaps[3])
        self.assertEqual(matrix.durations_s[3], (None, None, None))
        self.assertEqual(matrix.durations_s[0][0], 0.0)
        for i, origin in enumerate(points[:3]):
            for j, destination in enumerate(points[:3]):
                route = self.graph.route(origin, destination)
                self.assertAlmostEqual(matrix.durations_s[i][j], route.duration_s)
                self.assertAlmostEqual(matrix.distances_m[i][j], route.distance_m)

    def test_accessibility_and_nearest_destinations(self):
        points = [(48.0, 11.0), (48.001, 11.0), (48.0005, 11.0), (10.0, 10.0)]
        matrix = self.graph.travel_time_matrix(points, points)
        scores = self.graph.accessibility(points, points, minutes=[1, 60], weights=[1, 2, 3, 4])
        self.assertIsNone(scores[3])
        for i in range(3):
            within = [w for t, w in zip(matrix.durations_s[i], [1, 2, 3, 4]) if t is not None and t <= 60]
            self.assertAlmostEqual(scores[i][0], sum(within))
            self.assertAlmostEqual(scores[i][1], 6)
        decayed = self.graph.accessibility(points, points, minutes=[2], decay="exponential")
        self.assertTrue(0 < decayed[0][0] < 3)
        with self.assertRaises(ValueError):
            self.graph.accessibility(points, points, minutes=[5], decay="cubic")
        with self.assertRaises(ValueError):
            self.graph.accessibility(points, points, minutes=[5], weights=[1])

        nearest = self.graph.nearest_destinations(points, points[1:3], k=1)
        self.assertEqual(nearest[3], [])
        index, seconds, metres = nearest[0][0]
        self.assertEqual(seconds, min(t for t in matrix.durations_s[0][1:3] if t is not None))
        self.assertGreater(metres, 0)

    def test_with_transit_rides_the_fixture_line(self):
        walk_time = self.graph.route((48.0, 11.0), (48.003, 11.0)).duration_s
        transit = self.graph.with_transit(FIXTURES / "tiny_gtfs", date="2026-10-06")
        self.assertEqual(transit.transit_summary, {"stops": 2, "patterns": 1, "unlinked_stops": 0})
        self.assertIsNone(self.graph.transit_summary)
        # Wait 2.5 min, ride 30 s, a few metres of walking.
        by_transit = transit.route((48.0, 11.0), (48.003, 11.0)).duration_s
        self.assertLess(by_transit, walk_time)
        self.assertAlmostEqual(by_transit, 150 + 30, delta=20)
        self.assertIn("transit_stops=2", repr(transit))
        with self.assertRaises(ValueError):
            self.graph.with_transit(FIXTURES / "tiny_gtfs", date="2026-02-30")
        drive = gw.SpatialGraph.from_osm((FIXTURES / "tiny_map.osm").read_text(encoding="utf-8"), "drive")
        with self.assertRaises(ValueError):
            drive.with_transit(FIXTURES / "tiny_gtfs", date="2026-10-06")

    def test_turn_cost_keywords_are_accepted(self):
        xml = (FIXTURES / "tiny_map.osm").read_text(encoding="utf-8")
        plain = gw.SpatialGraph.from_osm(xml, "drive", turn_penalty_s=0, u_turn_penalty_s=0)
        priced = gw.SpatialGraph.from_osm(xml, "drive", u_turn_penalty_s=100, traffic_signal_s=5)
        self.assertEqual(plain.node_count(), priced.node_count())
        with self.assertRaises(TypeError):
            gw.SpatialGraph.from_osm(xml, "drive", turn_penalty=1)

    def test_profile_keywords_change_travel_times(self):
        xml = (FIXTURES / "tiny_map.osm").read_text(encoding="utf-8")
        slow = gw.SpatialGraph.from_osm(xml, "walk", walk_speed_kph=2.5)
        normal = self.graph.route((48.0, 11.0), (48.001, 11.0))
        halved = slow.route((48.0, 11.0), (48.001, 11.0))
        self.assertAlmostEqual(halved.duration_s, 2 * normal.duration_s, places=6)

        with self.assertRaises(TypeError):
            gw.SpatialGraph.from_osm(xml, "walk", walk_speed=3)

    def test_invalid_osm_raises_value_error(self):
        with self.assertRaises(ValueError):
            gw.SpatialGraph.from_osm("not xml", "walk")

    def test_no_path_raises_lookup_error(self):
        xml = """
        <osm>
          <node id="1" lat="0" lon="0" />
          <node id="2" lat="0" lon="0.001" />
          <node id="3" lat="1" lon="1" />
          <node id="4" lat="1" lon="1.001" />
          <way id="10"><nd ref="1" /><nd ref="2" /><tag k="highway" v="residential" /></way>
          <way id="20"><nd ref="3" /><nd ref="4" /><tag k="highway" v="residential" /></way>
        </osm>
        """
        graph = gw.SpatialGraph.from_osm(xml, "walk", retain_all=True)

        with self.assertRaises(LookupError):
            graph.route((0, 0), (1, 1))


if __name__ == "__main__":
    unittest.main()
