"""Performance envelopes, and a very small navigation database."""
from collections import namedtuple

Aircraft = namedtuple("Aircraft", "key name cruise_cas_kt cruise_mach cruise_alt_ft ceiling_ft "
                                  "min_cas_kt max_bank_deg roll_rate_dps max_vs_fpm "
                                  "vs_rate_fpm_s max_accel_ms2 drag fuel_flow_kgs fuel_kg")
Fix = namedtuple("Fix", "ident name lat lon elevation_ft")

# max_accel_ms2 is the acceleration at full thrust at sea level; with drag it
# sets the climb performance, so climb is a consequence of the energy balance
# rather than a separate table that can drift out of step with it.
AIRCRAFT = {a.key: a for a in [
    Aircraft("b738", "Narrow-body jet", 250.0, 0.78, 35000.0, 41000.0, 160.0,
             25.0, 5.0, 4000.0, 500.0, 1.70, 0.36, 1.05, 20000.0),
    Aircraft("at72", "Regional turboprop", 230.0, 0.52, 20000.0, 25000.0, 110.0,
             25.0, 6.0, 2500.0, 400.0, 1.40, 0.40, 0.20, 5000.0),
    Aircraft("c172", "Light single", 110.0, 0.22, 6500.0, 14000.0, 50.0,
             25.0, 10.0, 800.0, 300.0, 0.90, 0.55, 0.011, 150.0)]}

FIXES = {f.ident: f for f in [
    Fix("ESGG", "Goteborg Landvetter", 57.6628, 12.2798, 506.0),
    Fix("ESSA", "Stockholm Arlanda", 59.6519, 17.9186, 137.0),
    Fix("ESMS", "Malmo Sturup", 55.5363, 13.3762, 236.0),
    Fix("EKCH", "Copenhagen Kastrup", 55.6180, 12.6560, 17.0),
    Fix("ENGM", "Oslo Gardermoen", 60.1939, 11.1004, 681.0),
    Fix("EFHK", "Helsinki Vantaa", 60.3172, 24.9633, 179.0),
    Fix("EGLL", "London Heathrow", 51.4706, -0.4619, 83.0),
    Fix("NILEN", "NILEN", 58.3000, 13.6000, 0.0),
    Fix("SKUBI", "SKUBI", 55.9000, 12.9000, 0.0)]}


def parse_route(route):
    """Identifiers ("ESGG NILEN ESSA") or raw "lat,lon" pairs, DCT ignored."""
    fixes = []
    for i, token in enumerate(route.upper().split()):
        if token == "DCT":
            continue
        if "," in token:
            try:
                lat, lon = (float(v) for v in token.split(",", 1))
                assert -90 <= lat <= 90 and -180 <= lon <= 180
            except (ValueError, AssertionError):
                raise ValueError(f"bad coordinate '{token}'") from None
            fixes.append(Fix(f"WPT{i:02d}", token, lat, lon, 0.0))
        elif token in FIXES:
            fixes.append(FIXES[token])
        else:
            raise ValueError(f"unknown fix '{token}'")
    if len(fixes) < 2:
        raise ValueError("a route needs at least two fixes")
    return fixes
