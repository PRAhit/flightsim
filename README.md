# flightsim

A small flight simulator in Python. **400 lines**, no dependencies, no game
engine — give it an aircraft, a route and a wind, and it flies the whole
profile: takes off, climbs, cruises, works out for itself where the descent has
to begin, and arrives over the destination.

```bash
python3 -m flightsim ESGG ESSA EFHK --wind-from 250 --wind-kt 50
```

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

Two columns are worth staring at.

**The climb rate**: 2 025 fpm at FL115 falling to 325 fpm at FL350. Nobody wrote
those numbers down. Thrust lapses with air density, drag at constant indicated
airspeed does not, and what is left over is climb rate — so the climb decays on
its own, and would reach zero at the aircraft's ceiling.

**The indicated airspeed**: it holds at 241 knots all the way up and then *rises*
to 250 at the top. That is the Mach limit letting go. Below the crossover
altitude the aircraft is speed-limited; above it, Mach-limited.

## Run it

```bash
python3 -m flightsim ESGG NILEN ESSA                      # a route through a waypoint
python3 -m flightsim EKCH ENGM --aircraft at72            # a turboprop
python3 -m flightsim ESGG 58.5,14.0 ESSA --interval 60    # a raw coordinate, minutely
python3 -m flightsim --help

python3 -m unittest discover -s tests -t . -v             # 27 tests
```

Python 3.9 or newer. Nothing to install. `make docker` builds and runs a
container image if you would rather.

| Flag | Meaning |
|---|---|
| `--aircraft` | `b738` narrow-body jet, `at72` turboprop, `c172` light single |
| `--altitude` | cruise altitude in feet (default: the aircraft's own) |
| `--speed` | cruise airspeed in knots CAS (default: the aircraft's own) |
| `--wind-from`, `--wind-kt` | wind direction (the way it blows *from*) and strength |
| `--interval` | seconds of flight per printed line |

## How it works

| File | Lines | Purpose |
|---|---:|---|
| `flightsim/atmosphere.py` | 30 | ISA layers, and calibrated airspeed from true |
| `flightsim/navigation.py` | 61 | Haversine, bearings, cross-track, the wind triangle |
| `flightsim/aircraft.py` | 51 | Three performance envelopes, nine fixes, route parsing |
| `flightsim/simulator.py` | 212 | The flight model, the autopilot, and the loop |
| `flightsim/__main__.py` | 45 | The command line |
| **Total** | **400** | plus 216 lines of tests |

The aircraft is a point mass with a heading. It banks to turn, and spends
thrust on drag, on accelerating, and on climbing. Three autopilot channels run
on top of it: a lateral one that tracks the route, a vertical one that manages
the climb, cruise and descent, and an autothrottle.

## Design notes

**One energy model, used by both the aircraft and its autopilot.**
`excess_accel(thrust, climb)` returns what is left of the thrust once drag and
the climb have been paid for. That one function is the entire performance
model. The integrator uses it to accelerate the aircraft. The autopilot calls
it with the throttles at the stop to ask *how much climb can I afford right
now*, and never commands more than that. Because it is literally the same
function, the autopilot's idea of the aircraft cannot drift away from the
aircraft — which is what happens when a simulator keeps a hand-tuned climb
table alongside its physics.

**Only one channel controls the airspeed at a time.** Climb rate and airspeed
are bought with the same surplus thrust, so if the throttles and the elevator
both chase the speed they fight each other. Climbing, the throttles stay at the
limit and the elevator owns the speed: ask for more climb than the engines can
pay for and the aircraft gives back climb rate, not knots. Level or descending,
the roles swap — pitch holds the altitude, the throttles hold the speed.

**The aircraft flies its track, not its heading.** Position is advanced along
the ground track that falls out of the wind triangle, and the lateral channel
subtracts the drift it is actually measuring from the heading it commands.
Without that second half, every leg flown in a crosswind ends up downwind of
where it should.

**A fix is passed when the distance to it starts growing again.** The usual
rule — sequence once the distance drops below a threshold — breaks on the last
fix. It has no following turn to anticipate, so its threshold is small, and a
crosswind that leaves the aircraft a few hundred metres to one side sails past
it *outside* that radius. The route then never completes: the aircraft tracks a
leg it has already flown, for ever. So there are two tests for passage —
inside the turn-anticipation distance, or within three miles and getting
further away. `test_a_crosswind_does_not_stop_the_last_fix_sequencing` pins
that bug.

**Top of descent is computed every cycle, not configured.** Compare the height
still to lose against the distance still to run; when that works out steeper
than a three degree path, start down. A headwind cuts the ground speed and the
top of descent moves back on its own, which is the whole reason to compute it
instead of hard-coding 100 miles.

**A fixed 50 ms step, and nothing in the physics reads a clock.**
`advance(seconds)` runs a whole number of cycles and returns. No wall time and
no random numbers, so identical inputs give an identical flight, bit for bit.
Two tests hold this down, including one that an hour advanced in a single call
must match an hour advanced in sixty.

## Scope

Small on purpose, and written to be read. It models an aircraft's *performance*
along a route, the way a flight-planning backend does — not the flying of one.
So:

* **Not a flight dynamics model.** No attitude integration, no yaw or sideslip,
  no lift equation, no stall, no control surfaces, no ground handling.
* **Not real performance data.** The three aircraft are plausible shapes, not
  real types; the numbers were chosen so that the climb rates and speeds come
  out in the right neighbourhood. Nothing here may be used to plan a real
  flight.
* **Not a navigation database.** Nine fixes, typed in by hand, rounded to about
  a hundred metres. The real thing is an AIRAC-cycled product of hundreds of
  thousands of records held to DO-200A data quality requirements.
* **No procedures.** No SIDs, STARs, approaches or holds; no altitude or speed
  constraints, no airspace, terrain or traffic. The flight ends levelled off
  1 500 feet above the destination — it does not land. On a short sector the
  descent begins before the cruise level is reached, which is also what happens
  in practice.
* **One wind for the whole airspace.** No turbulence, no ISA deviation, and a
  fuel burn that never feeds back into the weight.

## References

* **ISO 2533** — Standard Atmosphere; the source of every constant in
  `atmosphere.py`.
* **Ed Williams, "Aviation Formulary"** — the standard reference for the
  great-circle formulae, including cross-track distance.
* **ICAO Doc 9613, PBN Manual** — fly-by turn anticipation, the idea behind
  waypoint sequencing.

**No compliance with any of the above is claimed, implied or attempted.**

## Licence

MIT — see [LICENSE](LICENSE).
