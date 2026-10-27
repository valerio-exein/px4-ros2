"""Latitude/longitude -> UTM zone 33N (EPSG:25833, ETRS89; same as WGS84 at this scale).

Transverse Mercator forward projection with Krueger's series (3 terms: sub-millimetre at
UTM distances). The Lazio orthophotos are served correctly only in their native EPSG:25833;
requesting EPSG:4326 from that server swaps the colour bands.
"""
import math

A_GRS80 = 6378137.0
F_GRS80 = 1 / 298.257222101
K0 = 0.9996
LON0 = 15.0          # zone 33 central meridian
E0 = 500000.0

_n = F_GRS80 / (2 - F_GRS80)
_A = A_GRS80 / (1 + _n) * (1 + _n ** 2 / 4 + _n ** 4 / 64)
_ALPHA = (_n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16,
          13 * _n ** 2 / 48 - 3 * _n ** 3 / 5,
          61 * _n ** 3 / 240)
_C = 2 * math.sqrt(_n) / (1 + _n)


def to_utm33(lat: float, lon: float):
    """(easting, northing) in metres, EPSG:25833."""
    phi = math.radians(lat)
    dlam = math.radians(lon - LON0)
    t = math.sinh(math.atanh(math.sin(phi)) - _C * math.atanh(_C * math.sin(phi)))
    xi = math.atan2(t, math.cos(dlam))
    eta = math.atanh(math.sin(dlam) / math.sqrt(1 + t * t))
    e = eta + sum(a * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j, a in enumerate(_ALPHA, 1))
    n = xi + sum(a * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j, a in enumerate(_ALPHA, 1))
    return E0 + K0 * _A * e, K0 * _A * n
