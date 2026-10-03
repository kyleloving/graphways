"""
Type stubs for graphways -- the compiled Rust extension.

Geometry results are structured Python objects implementing
``__geo_interface__``, so GeoPandas and Shapely accept them directly. Use
``to_geojson()`` when you need serialized GeoJSON for Folium or web maps.
"""

from __future__ import annotations

from os import PathLike
from typing import Any, Sequence

# ---------------------------------------------------------------------------
# Result objects
# ---------------------------------------------------------------------------

class SnapResult:
    """Where a coordinate joined the road network: the closest point on any road."""

    @property
    def input_lat(self) -> float: ...

    @property
    def input_lon(self) -> float: ...

    @property
    def snapped_lat(self) -> float:
        """Latitude of the point on the road the input snapped to."""
        ...

    @property
    def snapped_lon(self) -> float:
        """Longitude of the point on the road the input snapped to."""
        ...

    @property
    def node_id(self) -> int:
        """OSM id of the nearer end of the snapped road segment."""
        ...

    @property
    def node_lat(self) -> float: ...

    @property
    def node_lon(self) -> float: ...

    @property
    def distance_m(self) -> float:
        """Distance in metres from the input coordinate to the road."""
        ...

    def as_dict(self) -> dict[str, float | int]: ...

    def __repr__(self) -> str: ...

class RouteResult:
    """Fastest route result with metrics, snap diagnostics, and GeoJSON export."""

    @property
    def coordinates(self) -> list[tuple[float, float]]: ...

    @property
    def cumulative_times_s(self) -> list[float]: ...

    @property
    def distance_m(self) -> float: ...

    @property
    def duration_s(self) -> float: ...

    @property
    def origin_snap(self) -> SnapResult: ...

    @property
    def destination_snap(self) -> SnapResult: ...

    def as_dict(self) -> dict[str, object]: ...

    def to_geojson(self) -> str:
        """Return this route as a GeoJSON ``Feature`` string."""
        ...

    @property
    def __geo_interface__(self) -> dict[str, Any]:
        """The route as a GeoJSON ``LineString`` mapping."""
        ...

    def to_shapely(self) -> Any:
        """The route as a ``shapely.LineString`` (requires shapely)."""
        ...

    def __repr__(self) -> str: ...

class IsochroneResult:
    """
    The area reachable within one travel-time threshold: a MultiPolygon that
    may have several parts and holes for unreachable pockets.
    """

    @property
    def minutes(self) -> float: ...

    def as_dict(self) -> dict[str, object]: ...

    def to_geojson(self) -> str:
        """Return this isochrone as a GeoJSON ``MultiPolygon`` geometry string."""
        ...

    @property
    def __geo_interface__(self) -> dict[str, Any]:
        """The isochrone as a GeoJSON ``MultiPolygon`` mapping."""
        ...

    def to_shapely(self) -> Any:
        """The isochrone as a ``shapely.MultiPolygon`` (requires shapely)."""
        ...

    def __repr__(self) -> str: ...

class Poi:
    """OpenStreetMap point of interest returned by ``SpatialGraph.fetch_pois``."""

    @property
    def id(self) -> int: ...

    @property
    def lat(self) -> float: ...

    @property
    def lon(self) -> float: ...

    @property
    def tags(self) -> dict[str, str]: ...

    def as_dict(self) -> dict[str, object]: ...

    def __repr__(self) -> str: ...

class PoiCollection:
    """Collection of POIs with structured access and GeoJSON export."""

    @property
    def count(self) -> int: ...

    @property
    def pois(self) -> list[Poi]: ...

    def as_dict(self) -> dict[str, object]: ...

    def to_geojson(self) -> str:
        """Return POIs as a GeoJSON ``FeatureCollection`` string."""
        ...

    @property
    def __geo_interface__(self) -> dict[str, Any]:
        """The POIs as a GeoJSON ``FeatureCollection`` mapping."""
        ...

    def __len__(self) -> int: ...

    def __repr__(self) -> str: ...

class TravelTimeMatrix:
    """Travel times between every origin and every destination."""

    @property
    def durations_s(self) -> tuple[tuple[float | None, ...], ...]:
        """
        ``durations_s[i][j]``: seconds from origin ``i`` to destination ``j``;
        ``None`` when there is no route or either point could not be snapped.
        Immutable, and converted only once, so repeated access is cheap.
        """
        ...

    @property
    def distances_m(self) -> tuple[tuple[float | None, ...], ...]:
        """
        ``distances_m[i][j]``: length in metres of the fastest route from
        origin ``i`` to destination ``j``; ``None`` exactly where
        ``durations_s`` is.
        """
        ...

    @property
    def origin_snaps(self) -> list[SnapResult | None]:
        """Where each origin joined the network (``None`` if too far away)."""
        ...

    @property
    def destination_snaps(self) -> list[SnapResult | None]:
        """Where each destination joined the network (``None`` if too far away)."""
        ...

    @property
    def shape(self) -> tuple[int, int]:
        """``(len(origins), len(destinations))``."""
        ...

    def __repr__(self) -> str: ...

# ---------------------------------------------------------------------------
# Graph views
# ---------------------------------------------------------------------------

class ReachableGraph:
    """
    Travel-time-labeled graph view produced by ``SpatialGraph.reachable``.

    Cheap inspection methods operate on the parent graph plus reachable labels.
    Constrained routing and isochrones materialize a bounded subgraph internally.
    """

    @property
    def max_time_s(self) -> float: ...

    def node_count(self) -> int: ...

    def edge_count(self) -> int: ...

    def contains_node(self, node_id: int) -> bool: ...

    def nearest_node(
        self, lat: float, lon: float
    ) -> tuple[int, float, float] | None: ...

    def travel_time_to_node_id(self, node_id: int) -> float | None: ...

    def nodes(self) -> list[dict[str, float | int]]:
        """
        Return reachable nodes with ``node_id``, ``lat``, ``lon``, and
        ``travel_time_s``.
        """
        ...

    def nodes_geojson(self) -> str:
        """
        Return reachable nodes as a GeoJSON ``FeatureCollection`` of points.
        """
        ...

    def edges_geojson(self) -> str:
        """
        Return edges whose source and target nodes are both reachable.
        """
        ...

    def to_geojson(self) -> str:
        """
        Return reachable nodes and edges in one GeoJSON ``FeatureCollection``.
        """
        ...

    def isochrone(
        self,
        origin: tuple[float, float],
        minutes: list[float],
        max_snap_m: float | None = 100.0,
    ) -> list[IsochroneResult]:
        """
        Compute isochrones within this reachable subgraph.
        """
        ...

    def route(
        self,
        origin: tuple[float, float],
        destination: tuple[float, float],
        max_snap_m: float | None = 100.0,
    ) -> RouteResult:
        """
        Find the fastest route constrained to this reachable subgraph.
        """
        ...

    def __repr__(self) -> str: ...

class PrismGraph:
    """
    Network-time prism produced by ``SpatialGraph.prism``.

    Cheap inspection methods operate on the parent graph plus prism labels.
    Constrained routing and isochrones materialize a bounded subgraph internally.
    """

    @property
    def max_time_s(self) -> float: ...

    @property
    def traversal_budget_s(self) -> float: ...

    @property
    def stop_time_s(self) -> float: ...

    @property
    def buffer_s(self) -> float: ...

    @property
    def direct_time_s(self) -> float: ...

    def node_count(self) -> int: ...

    def edge_count(self) -> int: ...

    def contains_node(self, node_id: int) -> bool: ...

    def nearest_node(
        self, lat: float, lon: float
    ) -> tuple[int, float, float] | None: ...

    def slack_at_node_id(self, node_id: int) -> float | None: ...

    def nodes(self) -> list[dict[str, float | int]]:
        """
        Return nodes with ``node_id``, ``lat``, ``lon``, ``inbound_time_s``,
        ``outbound_time_s``, and ``slack_s``.
        """
        ...

    def nodes_geojson(self) -> str:
        """
        Return prism nodes as a GeoJSON ``FeatureCollection`` of points.
        """
        ...

    def edges_geojson(self) -> str:
        """
        Return edges whose source and target nodes are both inside the prism.
        """
        ...

    def to_geojson(self) -> str:
        """
        Return prism nodes and edges in one GeoJSON ``FeatureCollection``.
        """
        ...

    def slack_polygon(self, min_slack_s: float = 0.0) -> str | None:
        """
        Build a GeoJSON polygon enclosing nodes with at least ``min_slack_s``.
        """
        ...

    def slack_polygons(self, min_slack_values: list[float]) -> list[str | None]:
        """
        Build one slack polygon per minimum-slack threshold.
        """
        ...

    def isochrone(
        self,
        origin: tuple[float, float],
        minutes: list[float],
        max_snap_m: float | None = 100.0,
    ) -> list[IsochroneResult]:
        """
        Compute isochrones constrained to this prism subgraph.
        """
        ...

    def route(
        self,
        origin: tuple[float, float],
        destination: tuple[float, float],
        max_snap_m: float | None = 100.0,
    ) -> RouteResult:
        """
        Find the fastest route constrained to this prism subgraph.
        """
        ...

    def __repr__(self) -> str: ...

class SpatialGraph:
    """
    A road-network graph loaded from OpenStreetMap.

    Construct once with :meth:`from_place`, :meth:`from_pbf`, or :meth:`from_osm`.
    Reuse the same object across multiple queries to avoid rebuilding the graph.

    Attributes are read-only; all mutations happen inside Rust.
    """

    @staticmethod
    def from_pbf(
        path: str | PathLike[str],
        network: str,
        retain_all: bool = False,
        **profile: Any,
    ) -> SpatialGraph:
        """
        Load a local OSM PBF file into a reusable ``SpatialGraph``.

        ``network`` accepts ``"drive"``, ``"drive_service"``, ``"walk"``,
        ``"bike"``, ``"all"``, or ``"all_private"``. Speed-profile keywords:
        ``walk_speed_kph``, ``bike_speed_kph``, ``drive_speeds_kph`` (a
        ``{highway_class: kph}`` dict), ``default_drive_speed_kph``,
        ``use_maxspeed``, ``merge_distance_m``, ``traffic_signal_s``, and the
        driving turn costs ``turn_penalty_s``, ``turn_bias``,
        ``u_turn_penalty_s`` and ``left_hand_traffic``.
        """
        ...

    @staticmethod
    def from_osm(
        xml: str,
        network: str,
        retain_all: bool = False,
        **profile: Any,
    ) -> SpatialGraph:
        """
        Parse an OSM XML string into a reusable ``SpatialGraph``.

        Accepts the same ``network`` values and speed-profile keywords as
        :meth:`from_pbf`.
        """
        ...

    @staticmethod
    def from_place(
        place: str,
        network: str,
        max_dist: float | None = None,
        retain_all: bool = False,
        **profile: Any,
    ) -> SpatialGraph:
        """
        Geocode a place name and download the road network within
        ``max_dist`` metres of it (default 5 km) from Overpass.

        Accepts the same ``network`` values and speed-profile keywords as
        :meth:`from_pbf`.
        """
        ...

    @staticmethod
    def load(path: str | PathLike[str]) -> SpatialGraph:
        """
        Load a graph written by :meth:`save`. Much faster than rebuilding it;
        the routing index comes back ready if it was built before saving.

        Raises ``ValueError`` for a file that is not a graphways graph or was
        written by an incompatible version, ``OSError`` if it cannot be read.
        """
        ...

    def save(self, path: str | PathLike[str], prepare_routing: bool = True) -> None:
        """
        Write the graph to ``path`` for a fast :meth:`load` later.

        By default the routing index is built first (if it isn't already) so
        the loaded graph routes at full speed straight away.
        """
        ...

    def with_transit(
        self,
        gtfs: str | PathLike[str],
        date: str,
        start: str = "07:00",
        end: str = "09:00",
        wait_factor: float = 0.5,
        max_link_m: float = 300.0,
    ) -> SpatialGraph:
        """
        A copy of this walking graph that can also ride public transport,
        from a GTFS feed (``.zip`` or directory).

        The service between ``start`` and ``end`` on ``date``
        (``"2026-10-06"``) is modelled by its frequencies: boarding costs the
        expected wait (``wait_factor`` x headway), riding the average running
        time, and changing lines means walking and waiting again. Routes,
        matrices, isochrones and accessibility then all use transit.
        """
        ...

    @property
    def transit_summary(self) -> dict[str, int] | None:
        """
        What :meth:`with_transit` added (``stops``, ``patterns``,
        ``unlinked_stops``), or ``None`` for a graph without transit.
        """
        ...

    def node_count(self) -> int:
        """Number of nodes in the graph."""
        ...

    def edge_count(self) -> int:
        """Number of directed edges in the graph."""
        ...

    def nearest_node(
        self, lat: float, lon: float
    ) -> tuple[int, float, float] | None:
        """
        Return ``(osm_id, lat, lon)`` for the node nearest to ``(lat, lon)``.

        Uses the internal R-tree spatial index -- O(log n).
        Returns ``None`` if the graph is empty.
        """
        ...

    def isochrone(
        self,
        origin: tuple[float, float],
        minutes: list[float],
        max_snap_m: float | None = 100.0,
    ) -> list[IsochroneResult]:
        """
        Compute isochrones from ``(lat, lon)`` using this graph.

        Parameters
        ----------
        origin:
            ``(lat, lon)`` origin coordinates.
        minutes:
            Travel-time thresholds in minutes.
        Returns
        -------
        list[IsochroneResult]
            One MultiPolygon result per time limit, in the same order as
            ``minutes``. Call ``to_geojson()`` when you need serialized
            GeoJSON.
        """
        ...

    def route(
        self,
        origin: tuple[float, float],
        destination: tuple[float, float],
        max_snap_m: float | None = 100.0,
    ) -> RouteResult:
        """
        Find the fastest route between two coordinates (exactly optimal).

        The network type (drive/walk/bike) is inherited from the ``SpatialGraph``.
        The first call starts building a routing index in the background and
        answers with A* meanwhile; later calls use the index once it is ready.

        Returns
        -------
        RouteResult
            Structured result with ``distance_m``, ``duration_s``,
            ``coordinates``, ``cumulative_times_s``, and snap diagnostics.

        """
        ...

    def prepare_routing(self) -> None:
        """
        Build the routing index for this graph's network type now and wait
        for it, so every subsequent ``route()`` takes the fast path. Optional:
        ``route()`` starts the same build in the background on first use.
        """
        ...

    def is_routing_prepared(self) -> bool:
        """Whether the routing index has been built."""
        ...

    def travel_time_matrix(
        self,
        origins: Sequence[tuple[float, float]],
        destinations: Sequence[tuple[float, float]] | None = None,
        max_snap_m: float | None = 100.0,
        max_minutes: float | None = None,
    ) -> TravelTimeMatrix:
        """
        Fastest travel times from every ``(lat, lon)`` origin to every
        destination, exactly as :meth:`route` would find them.

        Points farther than ``max_snap_m`` from any road get ``None`` times
        instead of failing the whole matrix. If ``destinations`` is omitted,
        the matrix is origin-to-origin. Pairs slower than ``max_minutes`` are
        ``None``, like unreachable ones. Large matrices build the routing
        index first (see :meth:`prepare_routing`), which makes them fast.
        """
        ...

    def accessibility(
        self,
        origins: Sequence[tuple[float, float]],
        opportunities: Sequence[tuple[float, float]],
        minutes: Sequence[float],
        weights: Sequence[float] | None = None,
        decay: str = "step",
        max_snap_m: float | None = 100.0,
    ) -> list[list[float] | None]:
        """
        Accessibility score of every origin: the sum over opportunities of
        ``weight x decay(travel time)``, one score per value in ``minutes``.

        ``decay`` is ``"step"`` (count within ``minutes``), ``"linear"``
        (falling to 0 at ``minutes``), ``"exponential"`` (halving every
        ``minutes``) or ``"gaussian"`` (standard deviation ``minutes``).
        Weights default to 1. Origins too far from any road score ``None``.
        """
        ...

    def nearest_destinations(
        self,
        origins: Sequence[tuple[float, float]],
        destinations: Sequence[tuple[float, float]],
        k: int = 1,
        max_snap_m: float | None = 100.0,
    ) -> list[list[tuple[int, float, float]]]:
        """
        The ``k`` destinations each origin reaches fastest, nearest first, as
        ``(index into destinations, duration_s, distance_m)`` tuples.
        """
        ...

    def fetch_pois(self, isochrone: IsochroneResult | str) -> PoiCollection:
        """
        Fetch OSM points of interest within a given isochrone polygon.

        Parameters
        ----------
        isochrone:
            An ``IsochroneResult`` from :meth:`isochrone`, or a GeoJSON
            geometry string.

        Returns
        -------
        PoiCollection
            Structured POI collection. Call ``to_geojson()`` for a GeoJSON
            ``FeatureCollection``.
        """
        ...

    def snap_point(self, lat: float, lon: float) -> SnapResult | None:
        """
        Where ``(lat, lon)`` joins the road network: the closest point on any
        road, as every query snaps its points. ``None`` for an empty graph.

        Use ``as_dict()`` if you need a plain dictionary.
        """
        ...

    def reachable(
        self,
        origin: tuple[float, float],
        minutes: float,
        max_snap_m: float | None = 100.0,
    ) -> ReachableGraph:
        """
        Compute one-sided reachability from ``(lat, lon)`` within ``minutes``.
        """
        ...

    def prism(
        self,
        origin: tuple[float, float],
        destination: tuple[float, float],
        max_minutes: float,
        stop_minutes: float = 0.0,
        buffer_minutes: float = 0.0,
        max_snap_m: float | None = 100.0,
    ) -> PrismGraph:
        """
        Return the network-time prism for nodes that fit within:

        ``origin -> node -> destination + stop_minutes + buffer_minutes <= max_minutes``.
        """
        ...

    def nodes_geojson(self) -> str:
        """
        All graph nodes as a GeoJSON ``FeatureCollection`` of ``Point`` features.

        Properties per feature: ``id``, ``lat``, ``lon``.
        """
        ...

    def edges_geojson(self) -> str:
        """
        All graph edges as a GeoJSON ``FeatureCollection`` of ``LineString`` features.

        Properties per feature: ``highway``, ``length_m``, ``speed_kph``,
        ``drive_time_s``, ``walk_time_s``, ``bike_time_s``.
        """
        ...

    def __repr__(self) -> str: ...

# ---------------------------------------------------------------------------
# Module-level functions
# ---------------------------------------------------------------------------

def geocode(place: str) -> tuple[float, float]:
    """
    Convert a place name to ``(lat, lon)`` via the Nominatim API.

    Parameters
    ----------
    place:
        Any Nominatim-supported query string (e.g. ``"Marienplatz, Munich, Germany"``).

    Returns
    -------
    tuple[float, float]
        ``(latitude, longitude)``
    """
    ...

def cache_dir() -> str:
    """
    Return the path to the on-disk XML cache directory.

    Override the default by setting the ``GRAPHWAYS_CACHE_DIR`` environment variable.
    """
    ...

def clear_cache() -> None:
    """
    Clear both the in-memory and on-disk XML caches.
    """
    ...

