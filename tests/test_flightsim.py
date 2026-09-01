"""Tests. Run with: python3 -m unittest discover -s tests -t ."""
import math
import unittest

from flightsim import atmosphere as atm
from flightsim.aircraft import parse_route
from flightsim.navigation import (MS_PER_KT, angle_diff, bearing, cross_track_nm,
                                  destination, distance_nm, wind_triangle)
from flightsim.simulator import Simulation


def flown(minutes=120.0, **kwargs):
    sim = Simulation(**kwargs)
    while sim.time_s < minutes * 60.0:
        sim.advance(30.0)
        if sim.completed and abs(sim.vertical_speed_fpm) < 50.0:
            break
    return sim


class AtmosphereTest(unittest.TestCase):

    def test_sea_level_is_the_isa_definition(self):
        t, p, rho, a = atm.conditions(0.0)
        self.assertAlmostEqual(t, 288.15, places=2)
        self.assertAlmostEqual(p, 101325.0, places=1)
        self.assertAlmostEqual(rho, 1.225, places=3)
        self.assertAlmostEqual(a, 340.29, places=1)

    def test_published_table_values(self):
        for altitude, temp, press, dens in [(5000.0, 255.65, 54019.9, 0.7361),
                                            (11000.0, 216.65, 22632.0, 0.3639),
                                            (15000.0, 216.65, 12044.6, 0.1937)]:
            with self.subTest(altitude=altitude):
                t, p, rho, _ = atm.conditions(altitude)
                self.assertAlmostEqual(t, temp, places=2)
                self.assertAlmostEqual(p, press, delta=1.0)
                self.assertAlmostEqual(rho, dens, places=4)

    def test_calibrated_equals_true_airspeed_at_sea_level(self):
        self.assertAlmostEqual(atm.cas_from_tas(128.6, 0.0), 128.6, places=6)

    def test_calibrated_falls_below_true_airspeed_up_high(self):
        self.assertLess(atm.cas_from_tas(220.0, 10668.0), 140.0)


class NavigationTest(unittest.TestCase):

    def test_known_great_circle_distances(self):
        self.assertAlmostEqual(distance_nm((51.4706, -0.4619), (40.6413, -73.7781)),
                               2991.0, delta=15.0)          # Heathrow to JFK
        self.assertAlmostEqual(distance_nm((57.6628, 12.2798), (59.6519, 17.9186)),
                               212.7, delta=2.0)            # Gothenburg to Stockholm

    def test_one_minute_of_latitude_is_one_nautical_mile(self):
        self.assertAlmostEqual(distance_nm((0.0, 0.0), (1 / 60.0, 0.0)), 1.0, places=2)

    def test_cardinal_bearings(self):
        self.assertAlmostEqual(bearing((0.0, 0.0), (1.0, 0.0)), 0.0, places=6)
        self.assertAlmostEqual(bearing((0.0, 0.0), (0.0, 1.0)), 90.0, places=6)

    def test_destination_inverts_distance_and_bearing(self):
        for brg in (0.0, 137.0, 271.0):
            end = destination((57.66, 12.28), brg, 120.0)
            self.assertAlmostEqual(distance_nm((57.66, 12.28), end), 120.0, places=4)
            # Compared through angle_diff: 0 and 359.9999 are the same bearing.
            self.assertAlmostEqual(angle_diff(bearing((57.66, 12.28), end), brg),
                                   0.0, places=4)

    def test_angle_diff_takes_the_short_way_round(self):
        self.assertAlmostEqual(angle_diff(10.0, 350.0), 20.0, places=9)
        self.assertAlmostEqual(angle_diff(350.0, 10.0), -20.0, places=9)

    def test_cross_track_is_zero_on_the_leg_and_signed_off_it(self):
        start, end = (0.0, 0.0), (0.0, 10.0)               # due east
        self.assertAlmostEqual(cross_track_nm((0.0, 5.0), start, end), 0.0, places=6)
        self.assertLess(cross_track_nm((1.0, 5.0), start, end), 0.0)      # north is left

    def test_wind_triangle(self):
        self.assertAlmostEqual(wind_triangle(250.0, 90.0, 90.0, 50.0)[0], 200.0, places=6)
        self.assertAlmostEqual(wind_triangle(250.0, 90.0, 270.0, 50.0)[0], 300.0, places=6)
        speed, track = wind_triangle(250.0, 0.0, 270.0, 50.0)   # crosswind from the west
        self.assertGreater(track, 0.0)                          # drifts east
        self.assertGreater(speed, 250.0)

    def test_route_parsing_accepts_idents_and_coordinates(self):
        self.assertEqual([f.ident for f in parse_route("ESGG DCT ESSA")], ["ESGG", "ESSA"])
        self.assertAlmostEqual(parse_route("ESGG 58.5,14.0 ESSA")[1].lat, 58.5, places=6)
        for route in ("ESGG ZZZZ", "ESGG", "ESGG 91.0,14.0"):
            with self.subTest(route=route), self.assertRaises(ValueError):
                parse_route(route)


class DynamicsTest(unittest.TestCase):

    def test_turn_rate_follows_the_coordinated_turn_relation(self):
        """rate = g tan(phi) / V, which is the whole of turn performance."""
        sim = Simulation(route="ESGG ESSA")
        sim.tas_kt, sim.bank_deg, sim.lateral = 300.0, 25.0, "HDG"
        sim.target_heading = (sim.heading_deg + 90.0) % 360.0
        before = sim.heading_deg
        expected = math.degrees(9.80665 * math.tan(math.radians(25.0)) /
                                (300.0 * MS_PER_KT))
        sim.advance(1.0)
        self.assertAlmostEqual(angle_diff(sim.heading_deg, before), expected, delta=0.05)

    def test_climb_rate_decays_with_altitude(self):
        """Thrust lapses, drag at constant CAS does not, so the excess that
        becomes climb rate runs out on the way to the ceiling."""
        sim = Simulation(route="ESGG ESSA EFHK")
        rates = []
        for _ in range(5):
            sim.advance(300.0)
            if sim.phase == "CLIMB":
                rates.append(sim.vertical_speed_fpm)
        self.assertEqual(rates, sorted(rates, reverse=True))
        self.assertGreater(rates[0], 1500.0)

    def test_the_climb_holds_its_speed(self):
        """Once established — the first minutes are spent accelerating from the
        initial speed, which is below the climb speed on purpose."""
        sim = Simulation(route="ESGG ESSA EFHK")
        for _ in range(20):
            sim.advance(60.0)
            if sim.phase == "CLIMB" and sim.time_s > 300.0:
                self.assertAlmostEqual(sim.cas_kt, sim.target_cas(), delta=12.0)

    def test_fuel_burns_and_distance_accumulates(self):
        sim = Simulation(route="ESGG ESSA")
        start = sim.fuel_kg
        sim.advance(600.0)
        self.assertLess(sim.fuel_kg, start)
        self.assertGreater(sim.distance_flown_nm, 40.0)


class AutopilotTest(unittest.TestCase):

    def test_it_holds_the_leg_against_a_crosswind(self):
        sim = flown(30.0, route="ESGG ESSA", wind_from_deg=330.0, wind_kt=60.0)
        offset = cross_track_nm(sim.position, (57.6628, 12.2798), (59.6519, 17.9186))
        self.assertLess(abs(offset), 2.0)

    def test_it_sequences_waypoints_in_order(self):
        sim = flown(route="ESGG NILEN ESSA")
        messages = [e["message"] for e in sim.events]
        self.assertTrue(any("NILEN sequenced" in m for m in messages), messages)
        self.assertTrue(any("ESSA reached" in m for m in messages), messages)

    def test_a_crosswind_does_not_stop_the_last_fix_sequencing(self):
        """The bug this exists for: with only a distance threshold, a crosswind
        that leaves the aircraft a few hundred metres to one side sails past the
        last fix outside the radius and tracks the finished leg for ever."""
        sim = flown(route="ESGG ESSA", wind_from_deg=340.0, wind_kt=70.0)
        self.assertTrue(sim.completed, "the last fix never sequenced")

    def test_top_of_descent_is_further_out_from_a_higher_cruise(self):
        def descent_at(cruise_ft):
            sim = flown(route="ESGG ESSA EFHK", cruise_altitude_ft=cruise_ft)
            return next(e for e in sim.events if e["kind"] == "vnav"
                        and "descent" in e["message"])["message"]
        high = float(descent_at(35000.0).split()[3])
        low = float(descent_at(20000.0).split()[3])
        self.assertGreater(high, low)

    def test_a_full_flight_climbs_cruises_and_arrives(self):
        sim = flown(route="ESGG ESSA EFHK", wind_from_deg=250.0, wind_kt=50.0)
        kinds = [e["message"] for e in sim.events]
        self.assertTrue(sim.completed)
        self.assertTrue(any("level at" in m for m in kinds), kinds)
        self.assertTrue(any("top of descent" in m for m in kinds), kinds)
        self.assertAlmostEqual(sim.altitude_ft, sim.arrival_ft, delta=100.0)

    def test_a_headwind_makes_the_flight_take_longer(self):
        tail = flown(route="ESGG ESSA", wind_from_deg=235.0, wind_kt=70.0)
        head = flown(route="ESGG ESSA", wind_from_deg=55.0, wind_kt=70.0)
        self.assertGreater(head.time_s, tail.time_s * 1.1)


class DeterminismTest(unittest.TestCase):

    def test_identical_flights_agree_exactly(self):
        a = flown(40.0, route="ESGG NILEN ESSA", wind_from_deg=250.0, wind_kt=45.0)
        b = flown(40.0, route="ESGG NILEN ESSA", wind_from_deg=250.0, wind_kt=45.0)
        self.assertEqual(vars(a).keys(), vars(b).keys())
        self.assertEqual(a.events, b.events)
        for key in ("lat", "lon", "altitude_ft", "tas_kt", "fuel_kg", "time_s"):
            self.assertEqual(getattr(a, key), getattr(b, key), key)

    def test_an_hour_in_one_call_equals_an_hour_in_sixty(self):
        one = Simulation(route="ESGG ESSA")
        one.advance(600.0)
        many = Simulation(route="ESGG ESSA")
        for _ in range(60):
            many.advance(10.0)
        for key in ("lat", "lon", "altitude_ft", "tas_kt", "fuel_kg"):
            self.assertEqual(getattr(one, key), getattr(many, key), key)


class ValidationTest(unittest.TestCase):

    def test_an_unknown_aircraft_is_refused(self):
        with self.assertRaises(ValueError):
            Simulation(aircraft="concorde")

    def test_a_cruise_altitude_above_the_ceiling_is_refused(self):
        with self.assertRaises(ValueError):
            Simulation(aircraft="c172", cruise_altitude_ft=30000.0)

    def test_advancing_backwards_or_absurdly_is_refused(self):
        for seconds in (-1.0, 48 * 3600.0):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                Simulation().advance(seconds)


if __name__ == "__main__":
    unittest.main()
