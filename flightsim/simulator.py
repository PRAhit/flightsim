"""The aircraft, its autopilot, and the fixed-step loop that runs them.

Three degrees of freedom: a point with a heading that banks to turn and trades
thrust against drag and gravity. Nothing reads a clock or a random number, so
identical inputs give identical flights.
"""
import math

from . import atmosphere as atm
from .aircraft import AIRCRAFT, parse_route
from .navigation import (M_PER_FT, MS_PER_KT, G, angle_diff, bearing, cross_track_nm,
                         destination, distance_nm, normalize, wind_triangle)

DT = 0.05                             # seconds per cycle: 20 Hz
FT_PER_NM_3DEG = 318.4                # a three degree path, the airline standard
BANK_PER_DEG, XTK_PER_NM = 1.4, 12.0  # bank per degree of error; intercept per NM off
VS_PER_FT, CLIMB_TRIM = 5.0, 25.0     # vs per foot of error; climb traded per knot


class Simulation:
    """One aircraft flying one route with the autopilot engaged."""

    def __init__(self, aircraft="b738", route="ESGG ESSA", cruise_altitude_ft=0.0,
                 cruise_cas_kt=0.0, wind_from_deg=0.0, wind_kt=0.0, fuel_kg=0.0):
        if aircraft not in AIRCRAFT:
            raise ValueError(f"unknown aircraft '{aircraft}', try: {', '.join(AIRCRAFT)}")
        self.ac, self.fixes = AIRCRAFT[aircraft], parse_route(route)
        self.wind = (wind_from_deg, wind_kt)
        self.cruise_ft = cruise_altitude_ft or self.ac.cruise_alt_ft
        if self.cruise_ft > self.ac.ceiling_ft:
            raise ValueError(f"cruise altitude {self.cruise_ft:.0f} ft is above the "
                             f"{self.ac.ceiling_ft:.0f} ft ceiling")
        self.cruise_cas = cruise_cas_kt or self.ac.cruise_cas_kt
        self.arrival_ft = self.fixes[-1].elevation_ft + 1500.0
        self.lateral, self.vertical, self.phase = "NAV", "VNAV", "CLIMB"
        self.target_altitude_ft, self.active, self.completed = self.cruise_ft, 1, False
        self.events, self._last_fix_nm, self._throttle_i = [], None, 0.0
        start = self.fixes[0]
        self.time_s, self.distance_flown_nm = 0.0, 0.0
        self.lat, self.lon, self.altitude_ft = start.lat, start.lon, start.elevation_ft
        self.heading_deg = bearing(_pos(start), _pos(self.fixes[1]))
        self.target_heading = self.heading_deg
        self.tas_kt, self.vertical_speed_fpm, self.bank_deg = self.ac.min_cas_kt * 1.3, 0.0, 0.0
        self.thrust, self.fuel_kg = 0.6, fuel_kg or self.ac.fuel_kg * 0.5
        self._derive()

    @property
    def position(self):
        return (self.lat, self.lon)

    def _log(self, kind, message):
        self.events.append({"t": round(self.time_s, 1), "kind": kind, "message": message})

    def _derive(self):
        """The values that follow from the integrated ones."""
        alt_m = self.altitude_ft * M_PER_FT
        self.cas_kt = atm.cas_from_tas(self.tas_kt * MS_PER_KT, alt_m) / MS_PER_KT
        self.mach = self.tas_kt * MS_PER_KT / atm.conditions(alt_m)[3]
        self.ground_speed_kt, self.track_deg = wind_triangle(
            self.tas_kt, self.heading_deg, *self.wind)

    def excess_accel(self, thrust, climb_deg):
        """Thrust minus drag minus the climb, m/s^2 — the whole energy model.
        Shared by the integrator and the autopilot, so they cannot disagree."""
        ac = self.ac
        sigma = atm.conditions(self.altitude_ft * M_PER_FT)[2] / atm.RHO0
        return (ac.max_accel_ms2 * max(0.0, min(1.0, thrust)) * sigma ** 0.7
                - ac.max_accel_ms2 * ac.drag * (self.cas_kt / ac.cruise_cas_kt) ** 2
                - G * math.sin(math.radians(climb_deg)))

    def target_cas(self):
        """Constant CAS low down, constant Mach above the crossover altitude —
        why indicated airspeed drops near the top of a climb."""
        alt_m = self.altitude_ft * M_PER_FT
        mach_cas = atm.cas_from_tas(self.ac.cruise_mach * atm.conditions(alt_m)[3], alt_m)
        return min(self.cruise_cas, mach_cas / MS_PER_KT)

    def _bank_command(self):
        """Bank angle to fly: hold a heading, or track the active leg."""
        if self.lateral == "NAV" and not self.completed:
            start, end = _pos(self.fixes[self.active - 1]), _pos(self.fixes[self.active])
            if distance_nm(self.position, end) < 1.0:     # too close for a leg course
                track = bearing(self.position, end)
            else:
                intercept = -XTK_PER_NM * cross_track_nm(self.position, start, end)
                track = normalize(bearing(start, end) + max(-45.0, min(45.0, intercept)))
            # Fly the track: take out the drift the wind is already causing.
            heading = normalize(track - angle_diff(self.track_deg, self.heading_deg))
        else:
            heading = self.target_heading
        limit = self.ac.max_bank_deg
        return max(-limit, min(limit, BANK_PER_DEG * angle_diff(heading, self.heading_deg)))

    def _sequence(self):
        """Pass a fix early by the turn anticipation distance, or the moment the
        distance to it stops falling. The README explains why both are needed."""
        target = _pos(self.fixes[self.active])
        remaining = distance_nm(self.position, target)
        passed = (self._last_fix_nm is not None and self._last_fix_nm < 3.0
                  and remaining > self._last_fix_nm)
        self._last_fix_nm, anticipation = remaining, 0.3
        if self.active + 1 < len(self.fixes):
            radius = (self.tas_kt * MS_PER_KT) ** 2 / (
                G * math.tan(math.radians(self.ac.max_bank_deg))) / 1852.0
            turn = abs(angle_diff(bearing(target, _pos(self.fixes[self.active + 1])),
                                  bearing(_pos(self.fixes[self.active - 1]), target)))
            anticipation = max(0.3, radius * math.tan(math.radians(min(turn, 170.0) / 2)))
        if remaining <= anticipation or passed:
            reached, self._last_fix_nm = self.fixes[self.active].ident, None
            self.active += 1
            if self.active >= len(self.fixes):
                self.completed, self.lateral = True, "HDG"
                self.target_heading = self.heading_deg
                self._log("route", f"{reached} reached, route complete")
            else:
                self._log("route",
                          f"{reached} sequenced, direct {self.fixes[self.active].ident}")

    def _vertical_command(self):
        """Vertical speed to fly, after the performance and envelope limits."""
        if self.vertical != "VNAV":
            command = VS_PER_FT * (self.target_altitude_ft - self.altitude_ft)
        elif self.completed:
            self.phase = "ARRIVAL"
            command = VS_PER_FT * (self.arrival_ft - self.altitude_ft)
        else:
            to_lose, togo = self.altitude_ft - self.arrival_ft, self.distance_to_go_nm()
            if self.phase != "DESCENT" and to_lose > (togo - 4.0) * FT_PER_NM_3DEG > 0:
                self.phase = "DESCENT"      # the path is now steeper than 3 degrees
                self._log("vnav", f"top of descent, {togo:.1f} NM to go")
            if self.phase == "DESCENT":     # the rate that arrives on profile
                minutes = togo / max(60.0, self.ground_speed_kt) * 60.0
                command = -to_lose / max(0.5, minutes)
            else:
                error = self.target_altitude_ft - self.altitude_ft
                if abs(error) < 50.0 and self.phase != "CRUISE":
                    self.phase = "CRUISE"
                    self._log("vnav", f"level at {self.target_altitude_ft:.0f} ft")
                command = VS_PER_FT * error
        limit = self.ac.max_vs_fpm
        command = max(-limit, min(limit, command))
        if command > 0:
            # Speed on the elevator: ask for more climb than the engines can pay
            # for and the aircraft gives back climb rate, not airspeed.
            available = self.tas_kt * MS_PER_KT * self.excess_accel(1.0, 0.0) / G \
                / M_PER_FT * 60.0
            command = min(command, max(0.0, available + CLIMB_TRIM *
                                       (self.cas_kt - self.target_cas())))
            command *= max(0.0, min(1.0, (self.cas_kt - self.ac.min_cas_kt) / 10.0))
            command = 0.0 if self.altitude_ft > self.ac.ceiling_ft else command
        return command

    def _thrust_command(self, vs_command):
        """Climbing, the levers hold the limit and the elevator owns the speed;
        level or descending, that inverts."""
        if vs_command > 200.0:
            self._throttle_i = 0.0
            return 1.0
        error = self.target_cas() - self.cas_kt
        raw = 0.55 + vs_command / 12000.0 + 0.020 * error + 0.004 * self._throttle_i
        thrust = max(0.0, min(1.0, raw))
        if raw == thrust:                   # freeze the integrator while saturated
            self._throttle_i += error * DT
        return thrust

    def _integrate(self, bank, vertical_speed, thrust):
        ac = self.ac
        roll, vs_rate = ac.roll_rate_dps * DT, ac.vs_rate_fpm_s * DT
        self.bank_deg += max(-roll, min(roll, bank - self.bank_deg))
        self.vertical_speed_fpm += max(-vs_rate,
                                       min(vs_rate, vertical_speed - self.vertical_speed_fpm))
        # A coordinated turn's rate is set by bank and speed, so the same bank
        # turns a slow aircraft twice as fast as a fast one.
        tas_ms = max(self.tas_kt * MS_PER_KT, 1.0)
        self.heading_deg = normalize(self.heading_deg + DT * math.degrees(
            G * math.tan(math.radians(self.bank_deg)) / tas_ms))
        self.thrust = 0.0 if self.fuel_kg <= 0.0 else thrust
        climb = math.degrees(math.asin(max(-1.0, min(
            1.0, self.vertical_speed_fpm * M_PER_FT / 60.0 / tas_ms))))
        self.tas_kt = max(0.0, (tas_ms + self.excess_accel(self.thrust, climb) * DT)
                          / MS_PER_KT)
        gs, track = wind_triangle(self.tas_kt, self.heading_deg, *self.wind)
        leg_nm = gs * DT / 3600.0
        self.lat, self.lon = destination(self.position, track, leg_nm)  # track, not heading
        self.distance_flown_nm += leg_nm
        self.altitude_ft = max(0.0, self.altitude_ft + self.vertical_speed_fpm * DT / 60.0)
        burn = ac.fuel_flow_kgs * (0.12 + 0.88 * self.thrust) * DT
        self.fuel_kg = max(0.0, self.fuel_kg - burn)
        self.time_s += DT
        self._derive()

    def advance(self, seconds):
        """Run whole 50 ms cycles; an hour in one call equals an hour in sixty."""
        if not 0 <= seconds <= 6 * 3600:
            raise ValueError("advance by between 0 and 21600 seconds")
        for _ in range(int(round(seconds / DT))):
            if self.lateral == "NAV" and not self.completed:
                self._sequence()
            vertical = self._vertical_command()
            self._integrate(self._bank_command(), vertical, self._thrust_command(vertical))
        return self

    def distance_to_go_nm(self):
        if self.completed:
            return distance_nm(self.position, _pos(self.fixes[-1]))
        legs = zip(self.fixes[self.active:], self.fixes[self.active + 1:])
        return distance_nm(self.position, _pos(self.fixes[self.active])) + sum(
            distance_nm(_pos(a), _pos(b)) for a, b in legs)


def _pos(fix):
    return (fix.lat, fix.lon)
