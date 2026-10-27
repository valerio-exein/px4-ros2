#!/usr/bin/env python3
"""Download the aerial imagery the simulated camera uses, once, into camera/ortho/.

    python3 -I camera/fetch_ortho.py

Source: Regione Lazio geoportal WMS, layer "Ortofoto AGEA v. 2023" (20 cm/px), CC BY 4.0
(Regione Lazio public layers; AGEA orthophotos). Credit required: "Ortofoto AGEA 2023 -
Regione Lazio, CC BY 4.0", plus a note when modified (the camera crops and rotates it).

Area: 4 x 4 km centred on the Colosseum (the LLM pilot's 2 km range), at 0.5 m/px, fetched
as 4 x 4 requests of 2000 x 2000 px and stitched into ortho.jpg + ortho.json (corners).
Requested in the layer's native EPSG:25833 (UTM 33N): this server returns swapped colour bands
when asked to reproject to EPSG:4326. The mosaic is therefore a UTM grid; see geo.py.
"""
import json
import pathlib
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # geo.py, also under python3 -I
from geo import to_utm33  # noqa: E402

WMS = "https://geoportale.regione.lazio.it/geoserver/ows"
LAYER = "geonode:2023_AGEA_25833_COG"
CREDIT = "Ortofoto AGEA 2023 - Regione Lazio, CC BY 4.0"
CENTER = (41.8902, 12.4922)      # Colosseum
HALF_SIZE_M = 2000.0
M_PER_PX = 0.5
TILES = 4

OUT = pathlib.Path(__file__).resolve().parent / "ortho"


def main() -> None:
    OUT.mkdir(exist_ok=True)
    (OUT / "tiles").mkdir(exist_ok=True)
    ce, cn = to_utm33(*CENTER)
    e_min, n_max = round(ce - HALF_SIZE_M), round(cn + HALF_SIZE_M)
    span = 2 * HALF_SIZE_M
    size = int(round(2 * HALF_SIZE_M / M_PER_PX))     # 8000 px each way
    tile = size // TILES
    mosaic = np.zeros((size, size, 3), np.uint8)
    for row in range(TILES):            # row 0 = north
        for col in range(TILES):
            t_n_max = n_max - span * row / TILES
            t_n_min = n_max - span * (row + 1) / TILES
            t_e_min = e_min + span * col / TILES
            t_e_max = e_min + span * (col + 1) / TILES
            path = OUT / "tiles" / f"r{row}c{col}.jpg"
            if not path.exists():
                # EPSG:25833 is easting,northing in WMS 1.3.0.
                url = (f"{WMS}?service=WMS&version=1.3.0&request=GetMap&layers={LAYER}&styles="
                       f"&crs=EPSG:25833&bbox={t_e_min},{t_n_min},{t_e_max},{t_n_max}"
                       f"&width={tile}&height={tile}&format=image/jpeg")
                with urllib.request.urlopen(url, timeout=120) as resp:
                    if not resp.headers.get("Content-Type", "").startswith("image/jpeg"):
                        raise SystemExit(f"tile r{row}c{col}: not a JPEG ({resp.headers.get('Content-Type')})")
                    path.write_bytes(resp.read())
                time.sleep(0.5)  # be gentle with the public server
            img = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if img is None or img.shape[:2] != (tile, tile):
                raise SystemExit(f"tile r{row}c{col}: unreadable or wrong size")
            mosaic[row * tile:(row + 1) * tile, col * tile:(col + 1) * tile] = img
            print(f"tile r{row}c{col} ok", flush=True)
    cv2.imwrite(str(OUT / "ortho.jpg"), mosaic, [cv2.IMWRITE_JPEG_QUALITY, 92])
    meta = {"crs": "EPSG:25833", "easting_min": e_min, "northing_max": n_max,
            "width": size, "height": size, "m_per_px": M_PER_PX, "layer": LAYER, "credit": CREDIT,
            "source": WMS, "license": "CC BY 4.0", "fetched": time.strftime("%Y-%m-%d")}
    (OUT / "ortho.json").write_text(json.dumps(meta, indent=2))
    print(f"wrote {OUT / 'ortho.jpg'} ({size}x{size}) and ortho.json")


if __name__ == "__main__":
    main()
