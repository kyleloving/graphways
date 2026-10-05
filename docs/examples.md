# Examples

All examples use [folium](https://python-visualization.github.io/folium/) for map output and
[branca](https://python-visualization.github.io/branca/) for colormaps.

```bash
pip install folium branca
```

Full runnable source: [`examples/python_example.py`](https://github.com/kyleloving/graphways/blob/main/examples/python_example.py)

---

## Food access screening

Notebook: [`examples/food_deserts_dc.ipynb`](https://github.com/kyleloving/graphways/blob/main/examples/food_deserts_dc.ipynb)

This notebook shows a network-based food-access workflow in Washington, DC:
build a walking graph, fetch grocery-like OSM POIs, sample neighborhood points,
and mark places where no grocery store is reachable within a chosen walking
time. It is framed as a screening analysis rather than an official food-desert
classification.

---

## Walking isochrones

Graduated colour bands from a central point, rendered largest-first so smaller
bands appear on top.

```python
import json, graphways as gw, folium

lat, lon = gw.geocode("Marienplatz, Munich, Germany")
time_limits = [300, 600, 900, 1200, 1500, 1800]   # 5-30 min in 5-min steps
colors     = ["#2ecc71", "#27ae60", "#f1c40f", "#e67e22", "#e74c3c", "#c0392b"]
labels     = [f"{t // 60} min walk" for t in time_limits]

graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="walk", max_dist=5_000)
isos = graph.isochrone((lat, lon), minutes=[t / 60 for t in time_limits])

m = folium.Map(location=[lat, lon], zoom_start=14, tiles="Cartodb Positron")
folium.Marker(location=[lat, lon], tooltip="Marienplatz").add_to(m)

for iso, color, label in reversed(list(zip(isos, colors, labels))):
    folium.GeoJson(
        json.loads(iso.to_geojson()),
        name=label,
        style_function=lambda _, c=color: {
            "fillColor": c, "color": c, "weight": 1.5, "fillOpacity": 0.2,
        },
        tooltip=label,
    ).add_to(m)

folium.LayerControl().add_to(m)
m.save("isochrones.html")
```

---

## Gradient route colouring

Each road segment is coloured by midpoint travel time, from green (departure)
to red (arrival), with a legend bar.

```python
import json, graphways as gw, folium, branca.colormap

origin = gw.geocode("Marienplatz, Munich, Germany")
dest   = gw.geocode("English Garden, Munich, Germany")

graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="drive", max_dist=10_000)
route = graph.route(origin, dest)
route_geojson = json.loads(route.to_geojson())
coords = route_geojson["geometry"]["coordinates"]   # [lon, lat] per GeoJSON spec
times  = route.cumulative_times_s
total  = route.duration_s

m = folium.Map(location=list(origin), zoom_start=14, tiles="Cartodb Positron")

colormap = branca.colormap.LinearColormap(
    ["#2ecc71", "#f1c40f", "#e74c3c"], vmin=0, vmax=total / 60,
    caption="Travel time (minutes)",
)
colormap.add_to(m)

for i in range(len(coords) - 1):
    segment = [[coords[i][1], coords[i][0]], [coords[i+1][1], coords[i+1][0]]]
    t_mid   = (times[i] + times[i+1]) / 2
    folium.PolyLine(
        segment, color=colormap(t_mid / 60), weight=6, opacity=0.85,
        tooltip=f"t = {t_mid:.0f} s",
    ).add_to(m)

folium.Marker(list(origin), tooltip="Start", icon=folium.Icon(color="green")).add_to(m)
folium.Marker(list(dest),   tooltip="End",   icon=folium.Icon(color="red")).add_to(m)

m.save("route.html")
```

---

## POI discovery within an isochrone

Restaurants reachable within 10 minutes on foot, rendered as map markers.

```python
import json, graphways as gw, folium

lat, lon = gw.geocode("Marienplatz, Munich, Germany")

# Build graph once, reuse for isochrone + POIs
graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="walk", max_dist=3_000)
isos  = graph.isochrone((lat, lon), minutes=[10])                     # 10-minute walk
pois = graph.fetch_pois(isos[0])

m = folium.Map(location=[lat, lon], zoom_start=15, tiles="Cartodb Positron")
folium.GeoJson(json.loads(isos[0].to_geojson()), style_function=lambda _: {
    "fillColor": "#3498db", "color": "#2980b9", "weight": 1.5, "fillOpacity": 0.15,
}).add_to(m)

for poi in pois.pois:
    tags = poi.tags
    if tags.get("amenity") != "restaurant":
        continue
    lat_p, lon_p = poi.lat, poi.lon
    name = tags.get("name", "Restaurant")
    cuisine = tags.get("cuisine", "")
    folium.Marker(
        [lat_p, lon_p],
        tooltip=f"<b>{name}</b><br>{cuisine}",
        icon=folium.Icon(color="orange", icon="cutlery", prefix="fa"),
    ).add_to(m)

m.save("pois.html")
```

---

## Graph visualisation

Render the raw road network (simplified) as a thin line layer.

```python
import json, graphways as gw, folium

graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="drive", max_dist=3_000)
edges = json.loads(graph.edges_geojson())

m = folium.Map(location=[48.137, 11.575], zoom_start=14, tiles="Cartodb Positron")
folium.GeoJson(
    edges,
    style_function=lambda f: {
        "color": "#e74c3c" if f["properties"]["highway"] == "motorway" else "#3498db",
        "weight": 2,
        "opacity": 0.6,
    },
    tooltip=folium.GeoJsonTooltip(["highway", "length_m", "speed_kph"]),
).add_to(m)
m.save("network.html")
```

---

## Multi-origin isochrones with a shared graph

Compute isochrones from several stops on the same network without re-fetching the graph.

```python
import json, graphways as gw, folium

stops = {
    "Marienplatz":  (48.137144, 11.575399),
    "Hauptbahnhof": (48.140232, 11.558335),
    "Ostbahnhof":   (48.127264, 11.602636),
}

# One graph fetch covers all three stops
graph = gw.SpatialGraph.from_place("Marienplatz, Munich, Germany", network="walk", max_dist=5_000)

m = folium.Map(location=[48.137, 11.575], zoom_start=13, tiles="Cartodb Positron")

for name, (lat, lon) in stops.items():
    isos = graph.isochrone((lat, lon), minutes=[10])
    folium.GeoJson(
        json.loads(isos[0].to_geojson()),
        style_function=lambda _: {
            "fillColor": "#3498db", "color": "#2980b9",
            "weight": 1.5, "fillOpacity": 0.15,
        },
        tooltip=f"{name}: 10-min walk",
    ).add_to(m)
    folium.Marker([lat, lon], tooltip=name).add_to(m)

m.save("multi_origin.html")
```
