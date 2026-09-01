# flightsim

A small Python project I built to understand how an airliner's flight
management computer decides what to do next — when to stop climbing, how fast
to fly, and where to start down.

You give it an aircraft, a route and a wind. It takes off, climbs, cruises,
works out for itself where the descent has to begin, and arrives over the
destination. 400 lines, no dependencies, no game engine.

```
Narrow-body jet · ESGG ESSA EFHK · 428 NM · wind 250/50 kt
   TIME     ALT   CAS  MACH  HDG    V/S  THR   TOGO   FUEL  PHASE
     5m   11428   241 0.449   51   2025 1.00  402.4   9685  CLIMB
    10m   20251   241 0.531   52   1520 1.00  372.9   9370  CLIMB
    15m   26801   241 0.606   53   1116 1.00  340.2   9055  CLIMB
    20m   31559   242 0.670   54    802 1.00  304.7   8740  CLIMB
    25m   34935   244 0.724   55    325 1.00  266.9   8425  CLIMB
    30m   35000   250 0.741   56      0 0.81  227.4   8154  CRUISE
    35m   35000   250 0.741   78      0 0.82  187.8   7890  CRUISE
    40m   35000   250 0.741   79      0 0.82  148.0   7625  CRUISE
    45m   34975   250 0.741   80  -1200 0.61  108.3   7361  DESCENT
    50m   23354   263 0.613   82  -2155 0.00   70.5   7317  DESCENT
    55m   13356   259 0.498   83  -1860 0.00   38.0   7279  DESCENT
    60m    4606   256 0.420   84  -1651 0.00    9.5   7241  DESCENT
    65m    1679   250 0.389  103     -0 0.37   15.8   7139  ARRIVAL

    25m  vnav   level at 35000 ft
    31m  route  ESSA sequenced, direct EFHK
    45m  vnav   top of descent, 108.6 NM to go
    62m  route  EFHK reached, route complete

65 min · 444 NM · 2861 kg burned
```

**Watch the V/S column.** 2 025 feet per minute at 11 000 feet, down to 325 at
35 000. I never wrote those numbers anywhere. Thrust falls off as the air
thins, drag at a constant indicated airspeed does not, and whatever is left
over is climb rate — so the climb decays on its own, and would reach exactly
zero at the aircraft's ceiling.

**Now watch the CAS column.** It sits on 241 knots all the way up, then *rises*
to 250 near the top. That is the Mach limit letting go. Low down the aircraft
is limited by indicated airspeed; high up it is limited by Mach number, and the
handover between them is a real altitude that falls out of the atmosphere
model.

Neither of those is scripted. They are what you get when you write down the
energy balance once and let the loop run.

**Try it in thirty seconds:**

```bash
python3 -m flightsim ESGG ESSA EFHK --wind-from 250 --wind-kt 50   # the flight above
python3 -m unittest discover -s tests -t .                         # the 27 tests
```

Python 3.9 or newer, and nothing to install — the whole thing is the standard
library. Every block of output in this README is printed by the code.

---

## The trick, in one function

Everything the aircraft can do comes out of one line of arithmetic: how much
thrust is left after drag and the climb have taken their share.

```python
def excess_accel(self, thrust, climb_deg):      # simulator.py, terms named
    return (thrust_available            # falls away with air density
            - drag                      # rises with the square of the speed
            - G * sin(climb_deg))       # what the climb costs
```

(The real one spells those three terms out in full; it is still one return
statement.)

Spend the surplus on going faster and you accelerate. Spend it on going up and
you climb. Ask for more of both than you have and something has to give.

The part I like is that **this same function is called from two places**. The
integrator calls it to move the aircraft. The autopilot calls it with the
throttles at the stop to ask *how much climb can I afford right now?* — and
then commands no more than that.

That is why the climb rate in the table above decays smoothly instead of
stepping. It is also insurance: a simulator that keeps a hand-tuned climb table
next to its physics has two answers to the same question, and they drift apart.
Here there is only one answer, so they cannot.

---

## How it works

Four small files, each doing one job.

### `atmosphere.py` — the air

The International Standard Atmosphere in thirty lines: temperature, pressure,
density and the speed of sound at any altitude. Everything else in the project
leans on this. Thrust falls off with density, Mach number needs the speed of
sound, and the indicated airspeed the pilot reads is not the speed the aircraft
is actually travelling.

That last one caught me out. At 35 000 feet the aircraft in the table above is
showing 250 knots on the dial and moving through the air at about 430. The
airspeed indicator measures the pressure of the air hitting it, and up high
there is much less of it. Everything the autopilot does is in terms of the
indicated number, because that is what the aircraft's limits are written in.

### `navigation.py` — where things are

Great-circle distances and bearings, cross-track error, and the wind triangle.
Positions are latitude and longitude on a sphere rather than a proper WGS-84
ellipsoid — over a 200 mile leg that is worth less than half a mile, and the
autopilot corrects it away anyway.

The wind triangle is the interesting one: point the aircraft one way, and where
it actually goes is the vector sum of its own motion through the air and the
motion of the air over the ground. So there are two directions to keep straight
at all times — the heading it is *pointing*, and the track it is *making*.

### `aircraft.py` — three aeroplanes and nine places

A jet, a turboprop and a light single, each as a dozen numbers: cruise speed,
ceiling, bank limit, drag, fuel flow. No performance tables — the numbers feed
`excess_accel` and the climb comes out the other end.

Plus nine fixes to fly between, and a parser that takes either an identifier or
a raw coordinate, so `ESGG 58.5,14.0 ESSA` is a valid route.

### `simulator.py` — the aircraft and its autopilot

A point with a heading, banking to turn, nudged forward 50 milliseconds at a
time. On top of it sit three autopilot channels: **lateral** follows the route,
**vertical** runs the climb, cruise and descent, and the **autothrottle**
handles the levers.

Four things in here took me longer than they should have.

**Only one channel is allowed to control the airspeed.** Climb rate and
airspeed are bought out of the same surplus, so if the throttles and the
elevator both chase the speed, they fight. In the climb the throttles go to the
limit and stay there, and the elevator owns the speed: ask for more climb than
the engines can pay for and the aircraft gives back climb rate, not knots. Level
or descending, the two swap jobs.

**Fly the track, not the heading.** The position is advanced along the ground
track from the wind triangle, and the lateral channel takes the drift it is
actually measuring back out of the heading it commands. Miss the second half of
that and every leg in a crosswind quietly finishes downwind of where it should.

**A waypoint is passed when the distance to it starts growing again.** More on
this in the tests — it is my favourite bug in the project.

**Top of descent is worked out, not configured.** Every cycle it compares the
height still to lose against the distance still to run, and starts down when
that gets steeper than a three degree path — about 318 feet per mile. Raise the
cruise altitude and the descent starts further out, on its own:

```
cruise 20000 ft  →  top of descent 61.5 NM to go
cruise 30000 ft  →  92.9 NM
cruise 39000 ft  →  121.2 NM
```

The wind does not move that distance — the geometry does not know about wind.
What the wind changes is *when* you arrive at the point, and the rate you then
have to fly to stay on the path. A real FMS also pushes the top of descent out
into a headwind; this one does not, and that is on the list below.

---

## Running it

```bash
python3 -m flightsim ESGG ESSA                            # the basic case
python3 -m flightsim ESGG NILEN ESSA                      # via a waypoint
python3 -m flightsim EKCH ENGM --aircraft at72            # a turboprop
python3 -m flightsim ESGG 58.5,14.0 ESSA --interval 60    # a raw coordinate, minutely
python3 -m flightsim --help
```

| Flag | Meaning |
|---|---|
| `--aircraft` | `b738` narrow-body jet, `at72` turboprop, `c172` light single |
| `--altitude` | cruise altitude in feet (default: the aircraft's own) |
| `--speed` | cruise airspeed in knots indicated (default: the aircraft's own) |
| `--wind-from`, `--wind-kt` | wind direction (the way it blows *from*) and strength |
| `--interval` | seconds of flight per printed line |

Worth trying: fly the same route into a headwind and then a tailwind
(`--wind-from 70` against `--wind-from 250`) and watch the flight time move by
twenty minutes. Or put the light single on a long leg and watch it take two
hours to do what the jet does in one.

`make docker` builds and runs a container image if you would rather not use
your own Python.

| File | Lines | Purpose |
|---|---:|---|
| `flightsim/atmosphere.py` | 30 | ISA layers, and indicated airspeed from true |
| `flightsim/navigation.py` | 61 | Haversine, bearings, cross-track, the wind triangle |
| `flightsim/aircraft.py` | 51 | Three performance envelopes, nine fixes, route parsing |
| `flightsim/simulator.py` | 212 | The flight model, the autopilot, and the loop |
| `flightsim/__main__.py` | 45 | The command line |
| **Total** | **400** | plus 216 lines of tests |

---

## The tests

There are 27. A few check the physics against published values, and the rest
are mostly mistakes I actually made:

```
test_published_table_values
test_calibrated_falls_below_true_airspeed_up_high
test_one_minute_of_latitude_is_one_nautical_mile
test_turn_rate_follows_the_coordinated_turn_relation
test_climb_rate_decays_with_altitude
test_the_climb_holds_its_speed
test_it_holds_the_leg_against_a_crosswind
test_a_crosswind_does_not_stop_the_last_fix_sequencing
test_top_of_descent_is_further_out_from_a_higher_cruise
test_an_hour_in_one_call_equals_an_hour_in_sixty
```

**The one that earned its place is
`test_a_crosswind_does_not_stop_the_last_fix_sequencing`.** The obvious way to
decide you have passed a waypoint is to check whether the distance to it has
dropped below some threshold. That works everywhere except the last fix on the
route — it has no following turn to anticipate, so its threshold is small, and
a crosswind that leaves the aircraft a few hundred metres off to one side
carries it past the fix *outside* the circle. The distance never goes below the
threshold. The waypoint is never sequenced, the route never completes, and the
aircraft flies a leg it has already finished for ever.

The fix is to detect passage the way it really is detected: within a few miles
of the fix, the moment the distance stops falling and starts rising, the thing
is behind you. Two rules now, not one.

`test_an_hour_in_one_call_equals_an_hour_in_sixty` is the other one I would
keep. The physics runs on a fixed 50 millisecond step and never reads a clock
or a random number, so a flight advanced in one big call has to come out
identical, digit for digit, to the same flight advanced in sixty small ones. It
is a cheap test that catches an entire category of mistake — anything that
sneaks a dependency on wall time or on how the caller happened to chop the run
up.

---

## What I learned

- **Write the energy balance once and let everything read from it.** The climb
  rates, the ceiling and the speed the autopilot settles on are all
  consequences of one line, and I never had to tune any of them.
- **Decide who owns the airspeed.** Two controllers reaching for the same
  quantity is not a physics problem, it is a design problem, and it is the
  classic way to get an autoflight system oscillating against itself.
- **A "distance is less than X" test is usually hiding a bug.** Distance is a
  scalar; it throws away the direction you needed. The crosswind bug is entirely
  that mistake.
- **Determinism is a feature you have to defend.** It is easy to keep and very
  annoying to get back, so there is a test guarding it.
- **Heading is not track.** Almost every navigation bug I hit was some version
  of confusing the way it is pointing with the way it is going.

## What I would do next

- **Make the top of descent wind-aware.** It is pure geometry right now, so a
  60 knot headwind leaves it in exactly the same place. Descending into a
  headwind eats more track miles per foot, so it ought to move further out — and
  I would like to see how much.
- **Let the fuel burn feed back into the weight.** At the moment fuel is
  counted but the aircraft never gets lighter, so it never gets the climb
  performance back that a real one does as it burns down.
- **Wind that varies with altitude**, instead of one wind for the whole
  airspace. It would change the climb, the descent and where the top of descent
  belongs, all at once.
- **Altitude and speed constraints** — "cross this fix at or below 10 000 feet"
  is what turns a descent path into an actual arrival procedure, and it is the
  first thing a real FMS does that this one cannot.
- **Actually land it.** The flight currently stops 1 500 feet over the
  destination and calls it a day.

---

## What it is not

Small on purpose, and written to be read. It models an aircraft's *performance*
along a route, the way a flight-planning backend does, not the flying of one:

- **Not a flight dynamics model.** No attitude integration, no yaw or sideslip,
  no lift equation, no stall, no control surfaces, no ground handling.
- **Not real performance data.** The three aircraft are plausible shapes, not
  real types, and the numbers were picked so the climb rates and speeds come out
  in roughly the right neighbourhood. **Nothing here may be used to plan a real
  flight.**
- **Not a navigation database.** Nine fixes, typed in by hand, rounded to about
  a hundred metres. The real thing is an AIRAC-cycled product of hundreds of
  thousands of records held to DO-200A data quality requirements.
- **No procedures.** No SIDs, STARs, approaches or holds, and no airspace,
  terrain or traffic. On a short sector you will see the descent begin before
  the cruise level is reached — which is also what happens in practice.
- **One wind everywhere**, no turbulence, no ISA deviation, and a fuel burn that
  never feeds back into the weight.

---

## Where the maths comes from

I did not invent any of this. The atmosphere, the great-circle formulae and the
turn anticipation are all standard, and these are where they come from:

- **ISO 2533:1975, *Standard Atmosphere*** — the source of every constant in
  `atmosphere.py`: the sea-level values, the 6.5 K/km lapse rate and the
  tropopause at 11 km.
  [ISO catalogue entry](https://www.iso.org/standard/7472.html) ·
  [free overview on Wikipedia](https://en.wikipedia.org/wiki/International_Standard_Atmosphere)

- **Ed Williams, *Aviation Formulary*** — the reference everyone uses for
  great-circle distance, bearing, the point-at-distance-and-bearing problem and
  cross-track error. All four are in `navigation.py`.
  [edwilliams.org/avform147.htm](https://edwilliams.org/avform147.htm)

- **ICAO Doc 9613, *Performance-based Navigation (PBN) Manual*** — where the
  fly-by turn idea comes from: a waypoint is passed early by an amount that
  depends on the turn radius and how sharp the turn is, so the aircraft rolls
  onto the next leg instead of overshooting it.
  [ICAO PBN pages](https://www.icao.int/safety/pbn)

**No compliance with any of the above is claimed, implied or attempted.**

## Licence

MIT — see [LICENSE](LICENSE).

---

*A learning project. The physics and the formulae are standard and come from
the sources above; the code, the bugs, and the tests are mine.*
