#!/usr/bin/env python3
"""
Graphways README demo animation.

Produces a looping GIF showing the public Python stack:

1. Pin drop       - establish place (camera tight on origin)
2. Road network   - walkable road graph radiates outward (camera dollies out)
3. Isochrone      - 15-minute boundary settles and fills
4. POIs appear    - cafes materialize inside the isochrone
5. Route plays    - flag plants at destination, route walks itself there
6. Hold           - final state lingers before loop

Dependencies:
    pip install graphways matplotlib contextily shapely pyproj pillow numpy

Run from the repository root:
    python examples/readme_animation.py
"""

from __future__ import annotations

import json
from pathlib import Path

import contextily as ctx
import graphways as gw
import matplotlib
import numpy as np
import pyproj
from PIL import Image
from shapely.geometry import shape
from shapely.ops import transform as shapely_transform

matplotlib.use("Agg")
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402
from matplotlib.path import Path as MarkerPath  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402

ORIGIN_PLACE = "Dupont Circle, Washington DC, USA"
WALK_LIMIT_MIN = 15
NETWORK = "walk"
FPS = 12
DPI = 100
FIGSIZE = (6.0, 6.0)
OUTPUT_PATH = Path("docs/assets/graphways-demo.gif")
FRAME_DIR = Path("target/readme_animation_frames")

# Phase frame budgets - extended pin and hold for breathing room
FRAMES = {
    "pin": (0, 17),  # 18 frames - 1.5s
    "network": (18, 53),  # 36 frames - 3.0s
    "isochrone": (54, 77),  # 24 frames - 2.0s
    "pois": (78, 95),  # 18 frames - 1.5s
    "route": (96, 125),  # 30 frames - 2.5s
    "hold": (126, 149),  # 24 frames - 2.0s
}
TOTAL_FRAMES = FRAMES["hold"][1] + 1  # 150 frames at 12fps = 12.5s

# Camera dollies out from a tight crop on the origin during the pin and
# network phases. TIGHT_RADIUS_M is the half-width of the starting view.
TIGHT_RADIUS_M = 320

# Color palette
C_ISO_FILL = "#4AD98F"
C_ISO_EDGE = "#1A764B"
C_PIN = "#1C2833"
C_CAFE = "#C47A3A"
C_ROAD = "#528AA6"
C_ROUTE = "#1C2833"
C_FLAG = "#A8364B"  # destination flag - sits well with the green/orange/slate palette

WGS84_TO_WEB_MERCATOR = pyproj.Transformer.from_crs(
    "EPSG:4326", "EPSG:3857", always_xy=True
)


def to_mercator(lon: float, lat: float) -> tuple[float, float]:
    return WGS84_TO_WEB_MERCATOR.transform(lon, lat)


def geometry_to_mercator(geojson: str):
    return shapely_transform(
        WGS84_TO_WEB_MERCATOR.transform, shape(json.loads(geojson))
    )


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def ease_out_bounce(t: float) -> float:
    t = max(0.0, min(1.0, t))
    n1, d1 = 7.5625, 2.75
    if t < 1 / d1:
        return n1 * t * t
    if t < 2 / d1:
        t -= 1.5 / d1
        return n1 * t * t + 0.75
    if t < 2.5 / d1:
        t -= 2.25 / d1
        return n1 * t * t + 0.9375
    t -= 2.625 / d1
    return n1 * t * t + 0.984375


def ease_in_out(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def ease_out_quart(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 1 - (1 - t) ** 4


def ease_out_back(t: float, c1: float = 1.70158) -> float:
    """Overshoots slightly past 1, then settles - good for the flag plant."""
    t = max(0.0, min(1.0, t))
    c3 = c1 + 1
    x = t - 1
    return 1 + c3 * x * x * x + c1 * x * x


def progress(frame: int, phase: str) -> float:
    start, end = FRAMES[phase]
    return max(0.0, min(1.0, (frame - start) / max(end - start, 1)))


def in_phase(frame: int, phase: str) -> bool:
    start, end = FRAMES[phase]
    return start <= frame <= end


def feature_coord(feature: dict) -> tuple[float, float]:
    lon, lat = feature["geometry"]["coordinates"]
    return lat, lon


def pin_marker_path() -> MarkerPath:
    """Tear-drop pin shape with point at the bottom."""
    vertices = [
        (0.0, -1.3),
        (-0.75, -0.25),
        (-0.75, 0.45),
        (-0.35, 0.9),
        (0.0, 1.0),
        (0.35, 0.9),
        (0.75, 0.45),
        (0.75, -0.25),
        (0.0, -1.3),
        (0.0, -1.3),
    ]
    codes = [
        MarkerPath.MOVETO,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CURVE3,
        MarkerPath.CLOSEPOLY,
    ]
    return MarkerPath(vertices, codes)


def flag_marker_path() -> MarkerPath:
    """Pennant flag: thin pole + filled triangle, anchored at pole base (0, 0)
    so the flag plants exactly on the data point and extends upward from it."""
    pole_w = 0.05
    vertices = [
        (-pole_w, 0.00),  # pole base, left side
        (-pole_w, 1.60),  # up to top of pole
        (1.00, 1.30),  # out to flag tip
        (pole_w, 1.00),  # back to pole at flag base
        (pole_w, 0.00),  # down to pole base, right side
        (-pole_w, 0.00),  # close
        (-pole_w, 0.00),  # CLOSEPOLY needs a duplicate vertex
    ]
    codes = [MarkerPath.MOVETO] + [MarkerPath.LINETO] * 5 + [MarkerPath.CLOSEPOLY]
    return MarkerPath(vertices, codes)


def stitch_frames_to_gif(frame_paths: list[Path], output_path: Path) -> None:
    """Build one GIF from PNG frames using a shared palette to prevent flicker.
    Dithering is disabled so stable UI colors do not shimmer between frames."""
    frames = [Image.open(path).convert("RGB") for path in frame_paths]
    if not frames:
        return

    thumb_w, thumb_h = 120, 120
    sheet = Image.new("RGB", (thumb_w * len(frames), thumb_h), "white")
    for i, frame in enumerate(frames):
        thumb = frame.copy()
        thumb.thumbnail((thumb_w, thumb_h), Image.Resampling.LANCZOS)
        sheet.paste(thumb, (i * thumb_w, 0))

    palette = sheet.convert("P", palette=Image.Palette.ADAPTIVE, colors=256)
    quantized = [
        frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    quantized[0].save(
        output_path,
        save_all=True,
        append_images=quantized[1:],
        duration=round(1000 / FPS),
        loop=0,
        optimize=False,
    )


# ---------------------------------------------------------------------------
# Data fetch
# ---------------------------------------------------------------------------

print("Geocoding origin...")
lat, lon = gw.geocode(ORIGIN_PLACE)
origin_x, origin_y = to_mercator(lon, lat)
print(f"  {lat:.5f}, {lon:.5f}")

print("Building reusable walking graph...")
graph = gw.SpatialGraph.from_place(ORIGIN_PLACE, NETWORK, max_dist=5_000)
print(f"  {graph.node_count():,} nodes, {graph.edge_count():,} edges")

print("Reading road network geometry...")
edge_collection = json.loads(graph.edges_geojson())
print(f"  {len(edge_collection.get('features', [])):,} directed edges")

print("Computing 1-minute isochrone steps...")
minute_steps = list(range(1, WALK_LIMIT_MIN + 1))
iso_results = graph.isochrone((lat, lon), minutes=minute_steps)
iso_jsons = [iso.to_geojson() for iso in iso_results]
iso_polys = [geometry_to_mercator(geojson) for geojson in iso_jsons]
final_iso_json = iso_jsons[-1]
final_iso = iso_polys[-1]
print(f"  {len(iso_polys)} isochrones computed")

print("Fetching POIs inside the final isochrone...")
poi_collection = json.loads(graph.fetch_pois(final_iso_json).to_geojson())
cafes = []
for feature in poi_collection.get("features", []):
    props = feature.get("properties") or {}
    if props.get("amenity") != "cafe":
        continue
    cafe_lat, cafe_lon = feature_coord(feature)
    cafe_x, cafe_y = to_mercator(cafe_lon, cafe_lat)
    cafes.append(
        {
            "lat": cafe_lat,
            "lon": cafe_lon,
            "x": cafe_x,
            "y": cafe_y,
            "name": props.get("name", "Cafe"),
        }
    )
print(f"  {len(cafes)} cafes found")
if not cafes:
    raise RuntimeError(
        "No cafes found inside the isochrone. Try another origin or radius."
    )

# Pick a cafe at ~65% of max distance - long enough route to feel substantial,
# but clearly inside the isochrone rather than tangent to its edge.
cafes_by_distance = sorted(
    cafes,
    key=lambda c: (c["lat"] - lat) ** 2 + (c["lon"] - lon) ** 2,
)
target_index = int(len(cafes_by_distance) * 0.65)
destination = cafes_by_distance[target_index]
# Other cafes keep their original order during the POI reveal but are filtered
# out of the supporting set so the flag never overlaps a regular dot.
non_dest_cafes = [c for c in cafes if c is not destination]

print(f"Routing to {destination['name']!r}...")
route_result = graph.route((lat, lon), (destination["lat"], destination["lon"]))
route_json = json.loads(route_result.to_geojson())
route_coords = [
    to_mercator(lon_lat[0], lon_lat[1])
    for lon_lat in route_json["geometry"]["coordinates"]
]
route_props = route_json["properties"]
cumulative_times = route_props["cumulative_times_s"]
total_time = route_props["duration_s"]
total_distance = route_props["distance_m"]
print(f"  {total_distance:.0f} m, {total_time / 60:.1f} min")

# Precompute cumulative segment lengths along the route so the reveal can be
# paced by distance rather than segment count - keeps the leading edge moving
# at constant visual speed across uneven OSM segments.
route_array = np.array(route_coords)
seg_vecs = np.diff(route_array, axis=0)
seg_lengths = np.hypot(seg_vecs[:, 0], seg_vecs[:, 1])
cum_lengths = np.concatenate([[0.0], np.cumsum(seg_lengths)])
route_total_m = float(cum_lengths[-1])


def route_state_at(progress_t: float) -> tuple[list, tuple[float, float], float]:
    """Returns (segments, head_xy, distance_m) for a smooth, distance-paced reveal.

    The in-progress segment is partially drawn so the leading edge advances
    smoothly rather than snapping from one OSM node to the next."""
    target = ease_in_out(progress_t) * route_total_m
    segs = []
    head = route_coords[0]
    for i, length in enumerate(seg_lengths):
        if cum_lengths[i + 1] <= target:
            segs.append([route_coords[i], route_coords[i + 1]])
            head = route_coords[i + 1]
        elif cum_lengths[i] < target:
            frac = (target - cum_lengths[i]) / length
            a, b = route_coords[i], route_coords[i + 1]
            partial = (
                a[0] + frac * (b[0] - a[0]),
                a[1] + frac * (b[1] - a[1]),
            )
            segs.append([a, partial])
            head = partial
            break
        else:
            break
    return segs, head, target


# ---------------------------------------------------------------------------
# Map extent + camera + network segment preparation
# ---------------------------------------------------------------------------

minx, miny, maxx, maxy = final_iso.bounds
padding_m = 350
view_bounds = (minx - padding_m, miny - padding_m, maxx + padding_m, maxy + padding_m)
tight_bounds = (
    origin_x - TIGHT_RADIUS_M,
    origin_y - TIGHT_RADIUS_M,
    origin_x + TIGHT_RADIUS_M,
    origin_y + TIGHT_RADIUS_M,
)


def camera_for_frame(frame: int) -> tuple[float, float, float, float]:
    """Tight on the origin during the pin, eases out across the network reveal,
    locked to view_bounds for everything after."""
    if in_phase(frame, "pin"):
        return tight_bounds
    if in_phase(frame, "network"):
        t = ease_in_out(progress(frame, "network"))
        return tuple(lerp(a, b, t) for a, b in zip(tight_bounds, view_bounds))
    return view_bounds


network_edges = []
for feature in edge_collection.get("features", []):
    coords = feature.get("geometry", {}).get("coordinates") or []
    if len(coords) < 2:
        continue

    segment = [to_mercator(lon_lat[0], lon_lat[1]) for lon_lat in coords]
    xs = [xy[0] for xy in segment]
    ys = [xy[1] for xy in segment]
    if max(xs) < view_bounds[0] or min(xs) > view_bounds[2]:
        continue
    if max(ys) < view_bounds[1] or min(ys) > view_bounds[3]:
        continue

    midpoint = segment[len(segment) // 2]
    distance = float(np.hypot(midpoint[0] - origin_x, midpoint[1] - origin_y))
    network_edges.append((distance, segment))

network_edges.sort(key=lambda item: item[0])
network_segments = [segment for _, segment in network_edges]
network_colors_active = [to_rgba(C_ROAD, 0.65) for _ in network_edges]
network_colors_soft = [to_rgba(C_ROAD, 0.20) for _ in network_edges]
network_colors_faint = [to_rgba(C_ROAD, 0.10) for _ in network_edges]
print(f"  {len(network_segments):,} visible road segments")

# ---------------------------------------------------------------------------
# Figure setup
# ---------------------------------------------------------------------------

fig, ax = plt.subplots(figsize=FIGSIZE, dpi=DPI)
fig.patch.set_facecolor("white")
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax.set_position([0, 0, 1, 1])
# Initialize axes to the widest view so the basemap covers everything the
# camera will eventually pan over. The camera helper resets xlim/ylim each frame.
ax.set_xlim(view_bounds[0], view_bounds[2])
ax.set_ylim(view_bounds[1], view_bounds[3])
ax.set_aspect("equal")
ax.axis("off")

print("Adding basemap tiles...")
ctx.add_basemap(
    ax,
    crs="EPSG:3857",
    source=ctx.providers.CartoDB.Positron,
    zoom=16,  # one level above the area's natural zoom so the tight crop stays crisp
    attribution=False,
)

network_lc = LineCollection([], linewidths=0.85, zorder=2, capstyle="round")
ax.add_collection(network_lc)

iso_patches = []
for poly in iso_polys:
    coords = np.array(poly.exterior.coords)
    patch = MplPolygon(
        coords,
        closed=True,
        facecolor=C_ISO_FILL,
        edgecolor=C_ISO_EDGE,
        linewidth=0.6,
        alpha=0.0,
        zorder=3,
    )
    ax.add_patch(patch)
    iso_patches.append(patch)

final_coords = np.array(final_iso.exterior.coords)
(iso_ring,) = ax.plot(
    final_coords[:, 0],
    final_coords[:, 1],
    color=C_ISO_EDGE,
    linewidth=2.2,
    alpha=0.0,
    zorder=4,
)

pin_shadow = ax.scatter([], [], s=280, c="black", alpha=0.0, zorder=6, marker="o")
pin_marker = ax.scatter(
    [],
    [],
    s=260,
    c=C_PIN,
    alpha=0.0,
    zorder=7,
    marker=pin_marker_path(),
    edgecolors="white",
    linewidths=1.2,
)
cafe_scatter = ax.scatter(
    [],
    [],
    s=70,
    c=C_CAFE,
    alpha=0.0,
    zorder=8,
    marker="o",
    edgecolors="white",
    linewidths=0.8,
)
destination_flag = ax.scatter(
    [],
    [],
    s=420,
    c=C_FLAG,
    alpha=0.0,
    zorder=9,
    marker=flag_marker_path(),
    edgecolors="white",
    linewidths=1.0,
)
route_lc = LineCollection([], linewidths=3.5, zorder=10, capstyle="round")
# White halo behind the dark route so it stays legible over any basemap tile.
route_lc.set_path_effects([pe.Stroke(linewidth=5.5, foreground="white"), pe.Normal()])
ax.add_collection(route_lc)
# Leading dot at the front of the drawing route - reads as a person walking.
route_head = ax.scatter(
    [],
    [],
    s=110,
    c=C_ROUTE,
    alpha=0.0,
    zorder=11,
    marker="o",
    edgecolors="white",
    linewidths=1.6,
)

title = ax.text(
    0.5,
    0.965,
    "",
    transform=ax.transAxes,
    ha="center",
    va="top",
    fontsize=11,
    fontweight="bold",
    color="#1C2833",
    zorder=15,
    bbox=dict(
        boxstyle="round,pad=0.35",
        facecolor="white",
        edgecolor="#AEB6BF",
        linewidth=0.8,
        alpha=0.92,
    ),
)
title.set_visible(False)

sublabel = ax.text(
    0.5,
    0.028,
    "",
    transform=ax.transAxes,
    ha="center",
    va="bottom",
    fontsize=9,
    color="#566573",
    zorder=15,
    bbox=dict(
        boxstyle="round,pad=0.3", facecolor="white", edgecolor="none", alpha=0.85
    ),
)

all_artists = iso_patches + [
    network_lc,
    iso_ring,
    pin_shadow,
    pin_marker,
    cafe_scatter,
    destination_flag,
    route_lc,
    route_head,
    sublabel,
]

# ---------------------------------------------------------------------------
# Phase update functions
# ---------------------------------------------------------------------------


def set_origin_pin(alpha: float = 1.0, shadow_alpha: float = 0.15) -> None:
    pin_marker.set_offsets([[origin_x, origin_y]])
    pin_marker.set_alpha(alpha)
    pin_shadow.set_offsets([[origin_x, origin_y]])
    pin_shadow.set_alpha(shadow_alpha)


def set_non_dest_cafes(alpha: float) -> None:
    """Render the supporting cafes (everything except the chosen destination)."""
    if not non_dest_cafes:
        cafe_scatter.set_offsets(np.empty((0, 2)))
        cafe_scatter.set_alpha(0.0)
        return
    cafe_scatter.set_offsets(
        np.column_stack(
            ([c["x"] for c in non_dest_cafes], [c["y"] for c in non_dest_cafes])
        )
    )
    cafe_scatter.set_alpha(alpha)


def set_full_route() -> None:
    segments = [
        [route_coords[i], route_coords[i + 1]] for i in range(len(route_coords) - 1)
    ]
    route_lc.set_segments(segments)
    route_lc.set_color(C_ROUTE)


def phase_pin(frame: int) -> None:
    network_lc.set_segments([])
    t = progress(frame, "pin")
    bounce = ease_out_bounce(t)
    drop = 900 * (1 - bounce)
    pin_marker.set_offsets([[origin_x, origin_y + drop]])
    pin_marker.set_alpha(min(1.0, t * 4))
    pin_shadow.set_offsets([[origin_x, origin_y]])
    pin_shadow.set_alpha(0.18 * bounce)
    sublabel.set_text("Dupont Circle")


def phase_network(frame: int) -> None:
    t = progress(frame, "network")
    visible_count = max(1, round(ease_out_quart(t) * len(network_segments)))
    network_lc.set_segments(network_segments[:visible_count])
    network_lc.set_colors(network_colors_active[:visible_count])

    for patch in iso_patches:
        patch.set_alpha(0.0)

    set_origin_pin()
    sublabel.set_text("Street network")


def phase_isochrone(frame: int) -> None:
    t = ease_in_out(progress(frame, "isochrone"))
    network_lc.set_segments(network_segments)
    network_lc.set_colors(network_colors_soft)

    # Step expansion through the first 70% of the phase, then settle into
    # the final filled polygon over the remaining 30%.
    expansion_t = min(1.0, t / 0.7)
    settle_t = ease_in_out(max(0.0, (t - 0.7) / 0.3))

    raw_step = expansion_t * len(iso_patches)
    current_step = int(raw_step)
    step_fraction = raw_step - current_step

    for i, patch in enumerate(iso_patches):
        if i < current_step:
            patch.set_alpha(0.08)
            patch.set_linewidth(0.4)
        elif i == current_step and i < len(iso_patches):
            patch.set_alpha(0.22 * ease_in_out(step_fraction))
            patch.set_linewidth(0.9)
        else:
            patch.set_alpha(0.0)

    final_alpha = (
        max(0.08, 0.28 * settle_t) if current_step >= len(iso_patches) - 1 else 0.0
    )
    iso_patches[-1].set_alpha(final_alpha)
    iso_patches[-1].set_linewidth(0.0)
    iso_ring.set_alpha(settle_t)

    set_origin_pin()
    sublabel.set_text("15-minute isochrone")


def phase_pois(frame: int) -> None:
    t = ease_out_quart(progress(frame, "pois"))
    network_lc.set_segments(network_segments)
    network_lc.set_colors(network_colors_faint)
    iso_patches[-1].set_alpha(0.28)
    iso_ring.set_alpha(1.0)
    set_origin_pin()

    # Title counts up to the total number of cafes (including the destination,
    # which becomes the flag in the next phase). Render up to that many from
    # non_dest_cafes - so the destination's "slot" appears empty here and is
    # filled by the flag plant at the start of phase_route.
    displayed_count = max(1, round(t * len(cafes)))
    render_count = min(displayed_count, len(non_dest_cafes))
    if render_count > 0:
        visible = non_dest_cafes[:render_count]
        cafe_scatter.set_offsets(
            np.column_stack(([c["x"] for c in visible], [c["y"] for c in visible]))
        )
        cafe_scatter.set_alpha(0.9)
    else:
        cafe_scatter.set_offsets(np.empty((0, 2)))
        cafe_scatter.set_alpha(0.0)

    sublabel.set_text("Cafes within reach")


def phase_route(frame: int) -> None:
    t = progress(frame, "route")
    network_lc.set_segments(network_segments)
    network_lc.set_colors(network_colors_faint)
    iso_patches[-1].set_alpha(0.12)
    iso_ring.set_alpha(0.45)
    set_origin_pin(shadow_alpha=0.12)

    # Supporting cafes step back so the chosen one reads as the answer.
    fade_alpha = lerp(0.9, 0.20, ease_in_out(t))
    set_non_dest_cafes(fade_alpha)

    # Flag plants with a slight overshoot in the first ~20% of the phase.
    flag_t = min(1.0, t * 5)
    destination_flag.set_offsets([[destination["x"], destination["y"]]])
    destination_flag.set_sizes([420 * ease_out_back(flag_t)])
    destination_flag.set_alpha(min(1.0, flag_t * 1.5))

    # Distance-paced route reveal with a leading dot for the "walker".
    segments, head_xy, walked_m = route_state_at(t)
    route_lc.set_segments(segments)
    route_lc.set_color(C_ROUTE)
    route_head.set_offsets([list(head_xy)])
    route_head.set_alpha(1.0 if t > 0.02 else 0.0)

    sublabel.set_text("Route to cafe")


def phase_hold(_frame: int) -> None:
    network_lc.set_segments(network_segments)
    network_lc.set_colors(network_colors_faint)
    iso_patches[-1].set_alpha(0.12)
    iso_ring.set_alpha(0.45)
    set_origin_pin()
    set_non_dest_cafes(0.20)
    destination_flag.set_offsets([[destination["x"], destination["y"]]])
    destination_flag.set_sizes([420])
    destination_flag.set_alpha(1.0)
    set_full_route()
    route_head.set_offsets([[destination["x"], destination["y"]]])
    route_head.set_alpha(1.0)
    sublabel.set_text("Graphways")


DISPATCH = [
    ("pin", phase_pin),
    ("network", phase_network),
    ("isochrone", phase_isochrone),
    ("pois", phase_pois),
    ("route", phase_route),
    ("hold", phase_hold),
]


def update(frame: int):
    for phase_name, phase_fn in DISPATCH:
        if in_phase(frame, phase_name):
            phase_fn(frame)
            break
    # Apply the camera last so it overrides any axis state phases might touch.
    cam_minx, cam_miny, cam_maxx, cam_maxy = camera_for_frame(frame)
    ax.set_xlim(cam_minx, cam_maxx)
    ax.set_ylim(cam_miny, cam_maxy)
    return all_artists


# ---------------------------------------------------------------------------
# Render + save
# ---------------------------------------------------------------------------

OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
FRAME_DIR.mkdir(parents=True, exist_ok=True)
for old_frame in FRAME_DIR.glob("frame_*.png"):
    old_frame.unlink()

print(f"Rendering {TOTAL_FRAMES} PNG frames at {FPS} fps...")
frame_paths = []
for frame in range(TOTAL_FRAMES):
    update(frame)
    frame_path = FRAME_DIR / f"frame_{frame:04d}.png"
    fig.savefig(frame_path, facecolor="white", bbox_inches=None, pad_inches=0)
    frame_paths.append(frame_path)
    if (frame + 1) % 25 == 0 or frame + 1 == TOTAL_FRAMES:
        print(f"  rendered {frame + 1}/{TOTAL_FRAMES}")

plt.close(fig)
print(f"Stitching {len(frame_paths)} frames to {OUTPUT_PATH}...")
stitch_frames_to_gif(frame_paths, OUTPUT_PATH)
print(f"Done: {OUTPUT_PATH}")
