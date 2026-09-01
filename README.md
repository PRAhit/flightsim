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

**One energy model, two consumers.** `excess_accel` — thrust minus drag minus
the climb — is the entire performance model. The integrator moves the aircraft
with it, and the autopilot evaluates it at full thrust to decide how much climb
it may ask for. Because it is the same function, the autopilot's belief about
the aircraft's performance cannot drift away from the aircraft's actual
performance, which is exactly what happens when a simulator carries a separate
hand-tuned climb table.

**Speed on the elevator in the climb, on the thrust levers otherwise.** The
airspeed and the climb rate want to spend the same excess thrust, so exactly one
channel owns the speed at any moment. Climbing, the thrust levers go to the
limit and stay there while the vertical channel gives up climb rate to hold the
speed. Level or descending, that inverts. Letting both chase the speed at once
is the classic way to get an autoflight system fighting itself.

**Fix passage is detected, not assumed.** Sequencing a waypoint when the
distance to it drops below a threshold works until it does not: the last fix has
no turn to anticipate, so it gets a small threshold, and a crosswind that leaves
the aircraft a few hundred metres to one side sails straight past it *outside*
that radius. The flight then tracks a leg it has already flown, for ever. So
passage is also detected the way it is in practice — within a few miles, the
moment the distance stops falling and starts rising, the fix is behind you.
`test_a_crosswind_does_not_stop_the_last_fix_sequencing` is that bug, pinned.

**Top of descent is computed, not configured.** It is wherever the descent still
to be made becomes steeper than a three degree path: height to lose over current
ground speed. A headwind moves it automatically, which is the whole reason to
compute it rather than hard-code 100 miles.

**A fixed 50 ms step, and no clock in the physics.** `advance(seconds)` runs a
whole number of cycles and returns. Identical inputs give identical flights, bit
for bit — two tests hold this down, including one that an hour advanced in one
call must equal an hour advanced in sixty.

**Fly the track, not the heading.** The position is advanced along the ground
track from the wind triangle, and the lateral channel takes the measured drift
out of the heading it commands. Without that, every crosswind leg ends downwind
of where it should.

## Scope

Written to be read, and small on purpose. Some things it deliberately is not:

**Not a flight dynamics model.** No attitude integration, no yaw or sideslip, no
lift equation, no stall, no control surfaces, no ground handling. It models
*performance*, the way a flight-planning backend does, not handling.

**Not certified performance data.** The three aircraft are plausible shapes, not
real types; every number was chosen so the resulting climb rates and speeds land
in the right neighbourhood. Nothing here may be used to plan a real flight.

**Not a navigation database.** Nine hand-entered fixes rounded to about a
hundred metres. A real one is an AIRAC-cycled product of hundreds of thousands
of records under DO-200A data quality requirements.

**No procedures.** No SIDs, STARs, approaches, holds, altitude or speed
constraints, airspace, terrain or traffic. The flight ends levelled off 1 500
feet over the destination; it does not land. On a short sector you will see the
descent begin before the cruise level is reached — which is what happens in
practice.

**One wind for the whole airspace**, no turbulence, no ISA deviation, and fuel
burn that does not feed back into weight.

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
