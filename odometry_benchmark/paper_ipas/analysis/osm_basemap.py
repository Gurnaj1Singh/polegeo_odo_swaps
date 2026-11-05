"""OpenStreetMap-based map background for matplotlib axes in UTM 33N (or km offsets from a UTM origin).

Raster tiles (Web Mercator) are downloaded once into figures/map_cache/<provider>/ and resampled onto the axes' UTM
grid, so the axes keep true metres (Web Mercator would stretch distances ~2.2x at 63 deg N). Default provider: CARTO
"Positron" (OpenStreetMap data, light style; free for non-commercial use with attribution). The attribution is drawn on
every axes that uses it. openstreetmap.org's own tile server is not used: its policy does not allow scripted downloads.
"""
import io
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image
from pyproj import Transformer

CACHE = Path(__file__).resolve().parents[1] / "figures" / "map_cache"
PROVIDERS = {"carto_positron": ("https://basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png",  # now needs an API key
                                "© OpenStreetMap contributors © CARTO"),
             "esri_imagery": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/"
                              "{z}/{y}/{x}",
                              "Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community")}
UA = "odometry-benchmark-research-figure/1.0 (offline plotting script, cached tiles)"
_to_ll = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True)


def _tile(provider, z, x, y):
    p = CACHE / provider / str(z) / str(x) / f"{y}.png"
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        req = urllib.request.Request(PROVIDERS[provider][0].format(z=z, x=x, y=y), headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=30) as r:
            if r.status != 200 or not r.headers.get("Content-Type", "").startswith("image/"):
                raise RuntimeError(f"tile {z}/{x}/{y}: HTTP {r.status} {r.headers.get('Content-Type')}")
            p.write_bytes(r.read())
        time.sleep(0.2)
    return np.asarray(Image.open(io.BytesIO(p.read_bytes())).convert("RGB"))


def _pix(lon, lat, z):
    """Global Web Mercator pixel coordinates at zoom z."""
    n = 256 * 2 ** z
    lat = np.radians(lat)
    return (lon + 180) / 360 * n, (1 - np.log(np.tan(lat) + 1 / np.cos(lat)) / np.pi) / 2 * n


def add(ax, zoom, origin=(0.0, 0.0), scale=1.0, px=1000, alpha=0.9, fontsize=5.5, grey=True,
        provider="carto_positron", credit=True):
    """Draw the map under everything on `ax`. Axes coordinates are (UTM - origin) / scale, e.g. scale=1000 for km.
    Call after the data and the axes limits are final; the limits are then frozen. grey=True renders the map in
    greyscale so that coloured data lines do not clash with coloured roads. credit=False skips the per-axes
    attribution, for figures that print PROVIDERS[provider][1] once themselves."""
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_autoscale_on(False)
    ny = max(2, int(px * (y1 - y0) / (x1 - x0)))
    gx, gy = np.meshgrid(np.linspace(x0, x1, px), np.linspace(y1, y0, ny))  # row 0 = top
    lon, lat = _to_ll.transform(gx * scale + origin[0], gy * scale + origin[1])
    mx, my = _pix(lon, lat, zoom)
    tx0, tx1 = int(mx.min() // 256), int(mx.max() // 256)
    ty0, ty1 = int(my.min() // 256), int(my.max() // 256)
    mosaic = np.vstack([np.hstack([_tile(provider, zoom, tx, ty) for tx in range(tx0, tx1 + 1)])
                        for ty in range(ty0, ty1 + 1)])
    ix = np.clip((mx - tx0 * 256).astype(int), 0, mosaic.shape[1] - 1)
    iy = np.clip((my - ty0 * 256).astype(int), 0, mosaic.shape[0] - 1)
    img = mosaic[iy, ix]
    if grey:
        img = np.repeat((img @ [0.299, 0.587, 0.114])[..., None], 3, axis=2).astype(np.uint8)
    ax.imshow(img, extent=(x0, x1, y0, y1), origin="upper", aspect="auto", alpha=alpha, zorder=0,
              interpolation="bilinear")
    if credit:
        ax.text(0.995, 0.005, PROVIDERS[provider][1], transform=ax.transAxes, ha="right", va="bottom", fontsize=fontsize,
                color="#3d3d3a", bbox=dict(fc="white", ec="none", alpha=0.7, pad=1), zorder=10)
    return (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
