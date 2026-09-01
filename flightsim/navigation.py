"""Great-circle navigation and the wind triangle. Positions are (lat, lon).

A sphere, not WGS-84: over a 200 NM leg the difference is under half a mile
and the autopilot takes it out anyway.
"""
import math

EARTH_R_M, M_PER_NM, M_PER_FT, MS_PER_KT, G = 6371008.8, 1852.0, 0.3048, 0.5144444, 9.80665


def normalize(deg):
    return deg % 360.0


def angle_diff(target, current):
    """Shortest signed turn, in (-180, 180]. Positive is right."""
    return (target - current + 540.0) % 360.0 - 180.0


def distance_nm(a, b):
    """Haversine: well conditioned on short legs, unlike the law of cosines."""
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlat, dlon = lat2 - lat1, math.radians(b[1] - a[1])
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(h))) / M_PER_NM


def bearing(a, b):
    """Initial true course a to b. Initial, because a great circle changes
    course as it runs — so the autopilot recomputes it every cycle."""
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return normalize(math.degrees(math.atan2(y, x)))


def destination(start, bearing_deg, dist_nm):
    ang, brg = dist_nm * M_PER_NM / EARTH_R_M, math.radians(bearing_deg)
    lat1, lon1 = math.radians(start[0]), math.radians(start[1])
    lat2 = math.asin(math.sin(lat1) * math.cos(ang) +
                     math.cos(lat1) * math.sin(ang) * math.cos(brg))
    lon2 = lon1 + math.atan2(math.sin(brg) * math.sin(ang) * math.cos(lat1),
                             math.cos(ang) - math.sin(lat1) * math.sin(lat2))
    return (math.degrees(lat2), (math.degrees(lon2) + 540.0) % 360.0 - 180.0)


def cross_track_nm(position, leg_start, leg_end):
    """Signed distance from the leg; positive is right of course."""
    d13 = distance_nm(leg_start, position) * M_PER_NM / EARTH_R_M
    delta = math.radians(bearing(leg_start, position) - bearing(leg_start, leg_end))
    return math.asin(math.sin(d13) * math.sin(delta)) * EARTH_R_M / M_PER_NM


def wind_triangle(tas_kt, heading_deg, wind_from_deg, wind_kt):
    """(ground speed kt, true track deg). Wind direction is meteorological: the
    direction it blows *from*, as every METAR reports it."""
    hdg, blowing_to = math.radians(heading_deg), math.radians(wind_from_deg + 180.0)
    north = tas_kt * math.cos(hdg) + wind_kt * math.cos(blowing_to)
    east = tas_kt * math.sin(hdg) + wind_kt * math.sin(blowing_to)
    return math.hypot(north, east), normalize(math.degrees(math.atan2(east, north)))
