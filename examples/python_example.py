import json
import time

import branca.colormap
import folium
import graphways as gw

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def minutes(seconds: float) -> str:
    return f"{seconds / 60:.0f} min"


# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------


def isochrone_example():
    """Walking isochrones from Marienplatz with graduated colour by time band."""
    place = "Marienplatz, Munich, Germany"
    print(f"Geocoding '{place}'...")
    lat, lon = gw.geocode(place)
    print(f"  -> ({lat:.6f}, {lon:.6f})")

    time_limits = [300, 600, 900, 1200, 1500, 1800]  # 5–30 min in 5-min steps
    colors = ["#2ecc71", "#27ae60", "#f1c40f", "#e67e22", "#e74c3c", "#c0392b"]
    labels = [f"{minutes(t)} walk" for t in time_limits]

    t0 = time.time()
    graph = gw.SpatialGraph.from_place(place, network="walk", max_dist=5_000)
    isochrones = graph.isochrone((lat, lon), minutes=[t / 60 for t in time_limits])
    print(f"  Computed {len(isochrones)} isochrones in {time.time() - t0:.2f}s")

    m = folium.Map(location=[lat, lon], zoom_start=14, tiles="Cartodb Positron")
    folium.Marker(location=[lat, lon], tooltip=place).add_to(m)

    # Add largest isochrone first so smaller ones render on top
    for iso, color, label in reversed(list(zip(isochrones, colors, labels))):
        folium.GeoJson(
            iso.__geo_interface__,
            name=label,
            style_function=lambda _, c=color: {
                "fillColor": c,
                "color": c,
                "weight": 1.5,
                "fillOpacity": 0.2,
            },
            tooltip=label,
        ).add_to(m)

    folium.LayerControl().add_to(m)
    m.save("isochrone_example.html")
    print("  Saved to isochrone_example.html")


def routing_example():
    """Drive route with gradient coloring to visualise travel time along the path."""
    origin_place = "Marienplatz, Munich, Germany"
    dest_place = "English Garden, Munich, Germany"

    print("Geocoding origin and destination...")
    origin = gw.geocode(origin_place)
    dest = gw.geocode(dest_place)
    print(f"  Origin: {origin}")
    print(f"  Dest:   {dest}")

    t0 = time.time()
    graph = gw.SpatialGraph.from_place(origin_place, network="drive", max_dist=10_000)
    route_geojson = graph.route(origin, dest).to_geojson()
    elapsed = time.time() - t0

    route = json.loads(route_geojson)
    props = route["properties"]
    coords = route["geometry"]["coordinates"]  # each entry: [lon, lat]
    cum_times = props["cumulative_times_s"]  # parallel to coords
    total_time = props["duration_s"]
    total_dist = props["distance_m"]

    print(f"  Distance:  {total_dist:.0f} m")
    print(f"  Duration:  {total_time:.0f} s ({minutes(total_time)})")
    print(f"  Waypoints: {len(coords)}")
    print(f"  Compute:   {elapsed:.2f}s")

    m = folium.Map(
        location=[origin[0], origin[1]], zoom_start=14, tiles="Cartodb Positron"
    )

    # Colormap: green (start) → yellow → red (end), legend in minutes
    colormap = branca.colormap.LinearColormap(
        colors=["#2ecc71", "#f1c40f", "#e74c3c"],
        vmin=0,
        vmax=total_time / 60,
        caption="Travel time (minutes)",
    )
    colormap.add_to(m)

    # One PolyLine per segment, coloured by midpoint travel time
    for i in range(len(coords) - 1):
        segment = [
            [coords[i][1], coords[i][0]],  # [lat, lon]
            [coords[i + 1][1], coords[i + 1][0]],
        ]
        t_mid = (cum_times[i] + cum_times[i + 1]) / 2
        color = colormap(t_mid / 60)
        tooltip = f"t = {t_mid:.0f}s ({minutes(t_mid)})"

        folium.PolyLine(
            segment,
            color=color,
            weight=6,
            opacity=0.85,
            tooltip=tooltip,
        ).add_to(m)

    # Origin marker
    folium.Marker(
        location=list(origin),
        tooltip=f"<b>Start</b><br>{origin_place}",
        icon=folium.Icon(color="green", icon="play", prefix="fa"),
    ).add_to(m)

    # Destination marker — show total journey time in tooltip
    folium.Marker(
        location=list(dest),
        tooltip=f"<b>End</b><br>{dest_place}<br>{total_dist:.0f} m · {minutes(total_time)}",
        icon=folium.Icon(color="red", icon="stop", prefix="fa"),
    ).add_to(m)

    m.save("routing_example.html")
    print("  Saved to routing_example.html")


def isochrone_and_route_example():
    """Combined: walking isochrones from origin, plus a drive route to a destination."""
    origin_place = "Marienplatz, Munich, Germany"
    dest_place = "Olympiapark, Munich, Germany"

    print("Geocoding...")
    origin = gw.geocode(origin_place)
    dest = gw.geocode(dest_place)

    walk_graph = gw.SpatialGraph.from_place(origin_place, network="walk", max_dist=5_000)
    isochrones = walk_graph.isochrone(origin, minutes=[5, 10, 15])
    drive_graph = gw.SpatialGraph.from_place(origin_place, network="drive", max_dist=10_000)
    route_geojson = drive_graph.route(origin, dest).to_geojson()
    route = json.loads(route_geojson)
    props = route["properties"]
    coords = route["geometry"]["coordinates"]
    cum_times = props["cumulative_times_s"]
    total_time = props["duration_s"]

    m = folium.Map(
        location=[origin[0], origin[1]], zoom_start=13, tiles="Cartodb Positron"
    )

    # Isochrones
    iso_colors = ["#2ecc71", "#f1c40f", "#e74c3c"]
    iso_labels = ["5 min walk", "10 min walk", "15 min walk"]
    for iso, color, label in reversed(
        list(zip(isochrones, iso_colors, iso_labels))
    ):
        folium.GeoJson(
            iso.__geo_interface__,
            name=label,
            style_function=lambda _, c=color: {
                "fillColor": c,
                "color": c,
                "weight": 1,
                "fillOpacity": 0.2,
            },
            tooltip=label,
        ).add_to(m)

    # Route gradient
    colormap = branca.colormap.LinearColormap(
        colors=["#3498db", "#9b59b6"],
        vmin=0,
        vmax=total_time / 60,
        caption="Drive time (minutes)",
    )
    colormap.add_to(m)

    for i in range(len(coords) - 1):
        segment = [
            [coords[i][1], coords[i][0]],
            [coords[i + 1][1], coords[i + 1][0]],
        ]
        t_mid = (cum_times[i] + cum_times[i + 1]) / 2
        folium.PolyLine(
            segment,
            color=colormap(t_mid / 60),
            weight=5,
            opacity=0.9,
            tooltip=f"t = {minutes(t_mid)}",
        ).add_to(m)

    folium.Marker(location=list(origin), tooltip=origin_place).add_to(m)
    folium.Marker(
        location=list(dest),
        tooltip=f"{dest_place}<br>{props['distance_m']:.0f} m · {minutes(total_time)}",
        icon=folium.Icon(color="blue"),
    ).add_to(m)

    folium.LayerControl().add_to(m)
    m.save("combined_example.html")
    print("  Saved to combined_example.html")


def network_example():

    graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="drive", max_dist=3_000)
    edges = json.loads(graph.edges_geojson())

    m = folium.Map(location=[48.137, 11.575], zoom_start=14, tiles="Cartodb Positron")
    folium.GeoJson(
        edges,
        style_function=lambda f: {
            "color": (
                "#e74c3c" if f["properties"]["highway"] == "motorway" else "#3498db"
            ),
            "weight": 2,
            "opacity": 0.6,
        },
        tooltip=folium.GeoJsonTooltip(["highway", "length_m", "speed_kph"]),
    ).add_to(m)
    m.save("network_example.html")


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== Isochrone example ===")
    isochrone_example()

    # print("\n=== Routing example ===")
    # routing_example()

    # print("\n=== Combined example ===")
    # isochrone_and_route_example()

    # print("\n=== Network example ===")
    # network_example()
