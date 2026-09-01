"""International Standard Atmosphere (ISO 2533) and airspeed conversions."""
import math

T0, P0, RHO0 = 288.15, 101325.0, 1.225          # sea level: K, Pa, kg/m3
LAPSE, R, GAMMA, G = 0.0065, 287.05287, 1.4, 9.80665
TROPOPAUSE_M = 11000.0
T_TROP = T0 - LAPSE * TROPOPAUSE_M
P_TROP = P0 * (T_TROP / T0) ** (G / (LAPSE * R))
A0 = math.sqrt(GAMMA * R * T0)                  # sea-level speed of sound


def conditions(altitude_m):
    """(temperature K, pressure Pa, density kg/m3, speed of sound m/s)."""
    h = max(-1000.0, min(altitude_m, 20000.0))
    if h <= TROPOPAUSE_M:                       # troposphere: linear lapse
        t = T0 - LAPSE * h
        p = P0 * (t / T0) ** (G / (LAPSE * R))
    else:                                       # stratosphere: isothermal
        t = T_TROP
        p = P_TROP * math.exp(-G * (h - TROPOPAUSE_M) / (R * t))
    return t, p, p / (R * t), math.sqrt(GAMMA * R * t)


def cas_from_tas(tas_ms, altitude_m):
    """Calibrated airspeed, via the impact pressure the pitot tube sees. Equals
    true airspeed at sea level, and falls well below it up high."""
    _, p, _, a = conditions(altitude_m)
    m = tas_ms / a
    qc = p * ((1.0 + 0.2 * m * m) ** 3.5 - 1.0)
    return A0 * math.sqrt(5.0 * ((qc / P0 + 1.0) ** (2.0 / 7.0) - 1.0))
