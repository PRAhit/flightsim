"""Command line: fly a route and print the flight log."""
import argparse
import sys

from .aircraft import AIRCRAFT
from .simulator import Simulation


def main(argv=None):
    parser = argparse.ArgumentParser(prog="flightsim", description="Fly a route.")
    parser.add_argument("route", nargs="+", help="e.g. ESGG NILEN ESSA, or 58.5,14.0")
    parser.add_argument("--aircraft", default="b738", choices=sorted(AIRCRAFT))
    parser.add_argument("--altitude", type=float, default=0.0, help="cruise altitude, ft")
    parser.add_argument("--speed", type=float, default=0.0, help="cruise airspeed, kt CAS")
    parser.add_argument("--wind-from", type=float, default=0.0, help="degrees it blows from")
    parser.add_argument("--wind-kt", type=float, default=0.0)
    parser.add_argument("--interval", type=float, default=300.0, help="seconds per line")
    args = parser.parse_args(argv)
    try:
        sim = Simulation(args.aircraft, " ".join(args.route), args.altitude, args.speed,
                         args.wind_from, args.wind_kt)
    except ValueError as exc:
        return f"flightsim: {exc}"

    print(f"{sim.ac.name} · {' '.join(f.ident for f in sim.fixes)} · "
          f"{sim.distance_to_go_nm():.0f} NM · wind {args.wind_from:.0f}/{args.wind_kt:.0f} kt")
    print("   TIME     ALT   CAS  MACH  HDG    V/S  THR   TOGO   FUEL  PHASE")
    while sim.time_s < 6 * 3600:
        sim.advance(args.interval)
        print(f"{sim.time_s / 60:6.0f}m {sim.altitude_ft:7.0f} {sim.cas_kt:5.0f} "
              f"{sim.mach:5.3f} {sim.heading_deg:4.0f} {sim.vertical_speed_fpm:6.0f} "
              f"{sim.thrust:4.2f} {sim.distance_to_go_nm():6.1f} {sim.fuel_kg:6.0f}  "
              f"{sim.phase}")
        if sim.completed and abs(sim.vertical_speed_fpm) < 50.0:
            break
    print()
    for event in sim.events:
        print(f"{event['t'] / 60:6.0f}m  {event['kind']:<6s} {event['message']}")
    print(f"\n{sim.time_s / 60:.0f} min · {sim.distance_flown_nm:.0f} NM · "
          f"{sim.ac.fuel_kg * 0.5 - sim.fuel_kg:.0f} kg burned")
    return 0


if __name__ == "__main__":
    sys.exit(main())
