# gflib

Odometry and motion control for VEX robots. The core is platform independent —
`src/` and `include/` contain no `pros/` and no `Arduino.h`. Everything
hardware-specific lives in four small classes you write, which is why the same
library runs on a V5 brain and an ESP32.

## Conventions

Get these wrong and nothing above them works.

| | |
|---|---|
| Distance | inches |
| Angle | degrees |
| Zero heading | +Y axis |
| Heading direction | **clockwise positive** |
| Voltage | -12.0 .. +12.0 |
| Time | milliseconds |
| Scalar | `gflib::real` — `float` by default |

0° faces +Y, 90° faces +X. Same frame as LemLib, so waypoints carry across.

**Headings are unwrapped.** A robot that has spun twice reads 720, not 0.

**The library computes in `float`.** The ESP32-S3's FPU is single-precision
only, so a `double` here runs in software while the FPU idles. Build with
`-D GFLIB_DOUBLE_PRECISION` to switch the whole library to `double`; the test
suite runs both ways (`pio test -e native`, `pio test -e native_double`).

Over a full 105-second match this costs about 0.01" of position and 0.1° of
heading against the double build — an order of magnitude under V5 inertial
drift, and far under wheel slip.

**The HAL stays `double`.** `IEncoder::getCounts()` accumulates without
bound and `float` holds exact integers only to 2²⁴, so the four classes you
write below are unaffected by any of this. Deltas are narrowed after the
subtraction.

---

# Part 1 — Integration

## 1. Add the library

**V5 (PROS)** — vendor it in as a submodule:

```bash
git submodule add https://github.com/ronanhawkins/glfrclib.git gflib
```

Then in your `Makefile`:

```make
SRCDIRS += gflib/src
INCDIRS += gflib/include
```

**ESP32 (PlatformIO)** — add it to `lib_deps`:

```ini
lib_deps = https://github.com/ronanhawkins/glfrclib.git#v0.1
build_flags = -std=gnu++17
```

Pin a tag either way, so a teammate's push cannot change your build mid-session.

> ESP32-C6 users: stock `platform = espressif32` is espidf-only for the C6. Use
> the [pioarduino fork](https://github.com/pioarduino/platform-espressif32) to
> get `framework = arduino`.

## 2. Write the HAL

Four classes. This is the only file in your project that mentions both your
platform's SDK and gflib.

```cpp
#include "gflib/api.hpp"

class MyEncoder : public gflib::IEncoder {
    double getCounts() const override;        // raw counts, any unit
};

class MyImu : public gflib::IImu {
    double getHeadingDeg() const override;    // unwrapped, CW+, 0 = +Y
};

class MyDrive : public gflib::IDriveOutput {
    void setLeft(double volts) override;
    void setRight(double volts) override;
    void stop() override;                     // optional, defaults to 0V
};

class MyClock : public gflib::IClock {
    uint32_t millisNow() const override;
    void sleepMs(uint32_t ms) override;       // must yield, never spin
};
```

**V5:**

```cpp
double getCounts() const override { return sensor_.get_position(); }        // pros::Rotation
double getHeadingDeg() const override { return imu_.get_rotation(); }       // NOT get_heading()
void setLeft(double volts) override { left_.move_voltage(volts * 1000); }   // millivolts
void sleepMs(uint32_t ms) override { pros::delay(ms); }
```

Use `get_rotation()`, never `get_heading()` — the latter wraps at 360 and throws
your pose across the field once per revolution.

**ESP32:**

```cpp
double getCounts() const override { return (double)enc_.getCount(); }       // ESP32Encoder
void sleepMs(uint32_t ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }
```

Two things to handle in the IMU class and nowhere else:

- A BNO085 in RVC mode reports **counter-clockwise positive** — negate it.
- RVC **wraps** at ±180 — accumulate deltas instead of reporting the raw value,
  and treat the first frame as the origin rather than as motion.

Scale drive output by the measured pack voltage rather than a nominal 12, or
your gains will appear to drift as the battery drains.

## 3. Wire it up

`Drivetrain` consumes a pose; it does not produce one. What produces it is an
`IPoseSource`, composed explicitly and handed in.

**On a robot with its own sensors:**

```cpp
MyEncoder vertical(7), horizontal(8);
MyImu     imu(9);
MyDrive   drive(leftMotors, rightMotors);
MyClock   clock;

gflib::OdomPoseSource pose(vertical, horizontal, imu, clock, makeOdomConfig());
gflib::Drivetrain chassis(pose, drive, clock, makeConfig());

void initialize() {
    imu.calibrate();        // robot must be still
    chassis.calibrate();    // seeds the encoder baselines
}
```

**On a Brain that receives its pose over the link** — no encoders, no IMU, so
there is nothing to build an `OdomPoseSource` from:

```cpp
gflib::LinkPoseSource pose(serial, clock);
gflib::Drivetrain chassis(pose, drive, clock, makeConfig());

void initialize() {
    if (!pose.begin(2000)) { /* the far end is not talking */ }
}
```

Everything above `IPoseSource` is identical either way, and `makeConfig()` is
the same struct on both robots.

`chassis.calibrate()` is not optional on the sensor path. Without it the first
`update()` reads the whole boot-time encoder value as one enormous delta.

`Drivetrain` is **not thread safe** — keep `update()` and all motion calls on
one task.

## 4. Calibrate the odometry

**Do this before tuning anything.** Motors disconnected, push the robot by hand.

**Step 1 — `inchesPerCount`.** From `setPose(0,0,0)`, push exactly 3 m (118.11")
straight. `getPose()` should read `y ≈ 118`. Off by a constant factor? Scale
`vertInchesPerCount` by it.

```cpp
gflib::OdomSourceConfig o;
o.odom.vertInchesPerCount = (M_PI * 2.75) / 36000.0;   // V5 Rotation, 2.75" wheel
o.odom.vertInchesPerCount = (M_PI * 2.75) / (1024.0 * 4.0);   // 1024 PPR quadrature
```

**Step 2 — the offsets.** Spin exactly 360° clockwise in place:

```
offsetInches = (inches read over one full turn) / (2 * pi)
```

This gives magnitude *and* sign. Don't guess signs.

Then confirm:

| Test | Expect |
|---|---|
| 3 m straight | y ≈ +118", x ≈ 0 |
| Turn right 90°, then 3 m | x ≈ +118", y ≈ 0 |
| 360° clockwise | θ ≈ +360, x/y ≈ 0 |
| 1 m square, back to start | within ~2 cm |

Row 2 is the one that matters — the others pass even with a transposed frame.

**Do not proceed until the square closes.** Tuning PID on bad odometry is tuning
against lies.

---

# Part 2 — Autonomous

## 5. Configure

Everything robot-specific lives in one struct, in your project:

```cpp
// Geometry and the plausibility bounds live with the pose source, so a Brain
// with no encoders never sees them
gflib::OdomSourceConfig makeOdomConfig() {
    gflib::OdomSourceConfig o;

    o.odom.vertInchesPerCount  = ...;   // from step 4
    o.odom.horizInchesPerCount = ...;
    o.odom.vertOffsetInches    = ...;
    o.odom.horizOffsetInches   = ...;

    return o;
}

gflib::DrivetrainConfig makeConfig() {
    gflib::DrivetrainConfig c;

    c.lateral.kP = 0.0;                 // tuned below
    c.lateral.kD = 0.0;
    c.angular.kP = 0.0;
    c.angular.kD = 0.0;

    // smallErr, smallTimeMs, largeErr, largeTimeMs, timeoutMs
    c.lateralExit = gflib::ExitConditions{1.0, 100, 3.0, 500, 4000};
    c.angularExit = gflib::ExitConditions{1.0, 100, 3.0, 500, 2000};

    return c;
}
```

**All five `ExitConditions` fields must be set.** Zero means something different
in each: `largeErr = 0` disables that band, `smallTimeMs = 0` exits on the first
tick in tolerance, `timeoutMs = 0` gives up before the robot moves.

Other knobs worth knowing, all with sane defaults:

| Field | Does |
|---|---|
| `move.minVolts` | Floor on drive output. Raise if the robot stops 1–2" short |
| `turn.minVolts` | Same for turning. Usually higher — turning scrubs the wheels |
| `move.settleRadiusInches` | Inside this, stop steering. Too small and the robot orbits |
| `move.allowReverse` | `false` = turn around instead of backing up |
| `move.maxVolts` | Speed cap |
| `move.lead` | `moveToPose` carrot distance. Higher swings wider |
| `move.chainMinVolts` | Speed floor while chaining (see §7) |
| `loopMs` | Motion loop period. 10 = 100Hz |

## 6. Tune

**kP first, kD second, kI last or never. Angular before lateral**, because
`moveToPoint` depends on heading control.

1. kI = kD = 0. Raise kP until it just oscillates, back off ~30%.
2. Raise kD until the oscillation damps.
3. kI only for a gap friction won't let kP close. Most VEX drivetrains never
   need it.

Angular gains are in **degrees**, so ~57× smaller than the radian equivalent.
Divide by 57.3 if you lift them from a LemLib config.

## 7. The functions

All motions **block** and return `false` if they gave up on the timeout.

### Pose

```cpp
chassis.setPose(0, 24, 90);      // x, y, heading — place the robot on the field
gflib::Pose p = chassis.getPose();   // p.x, p.y, p.thetaDeg
```

Call `setPose` at the start of auton to tell the robot where it is.

### `turnToHeading(headingDeg, timeoutMs)`

Turns in place to an absolute field heading.

```cpp
chassis.turnToHeading(90, 1000);     // face +X
chassis.turnToHeading(-90, 1000);    // face -X
```

### `moveToPoint(x, y, timeoutMs, chainRadius = 0)`

Drives to a coordinate. Final heading is whatever the approach leaves.

```cpp
chassis.moveToPoint(24, 24, 2000);
```

Backs up automatically if the target is behind you. Set
`config().move.allowReverse = false` to turn around instead.

### `moveToPose(x, y, headingDeg, timeoutMs)`

Drives to a coordinate **and arrives facing a heading**, by curving in.

```cpp
chassis.moveToPose(24, 24, 180, 3000);   // arrive at (24,24) facing -Y
```

Use it when the approach angle matters — lining up on a goal. Slower than
`moveToPoint`, so don't use it where heading doesn't matter.

### `driveDistance(inches, timeoutMs, chainRadius = 0)`

Straight line relative to where you are now. Negative reverses without turning.

```cpp
chassis.driveDistance(12, 1500);     // forward 12"
chassis.driveDistance(-12, 1500);    // back up 12"
```

### Motion chaining

Add a **chain radius** to exit that far from a waypoint *without stopping*, so
the next move picks up at speed:

```cpp
chassis.moveToPoint(-24, -36, 1500, 8.0);   // exit 8" out, still rolling
chassis.moveToPoint(  0, -24, 1500, 8.0);   // same
chassis.moveToPoint( 24, -12, 2000);        // no radius -> settles and stops
```

Rules:

- **The last move must omit the radius**, or the motors stay powered.
- **Only works move-to-move.** Never chain into `turnToHeading` or
  `moveToPose` — a turn entered with speed arcs instead of pivoting.
- **Chained waypoints are hints, not gates.** The robot rounds the corner and
  never touches the point. Don't chain a waypoint that exists to clear a field
  element.
- Set `config().move.chainMinVolts` to 4–6 V, or the PID has already slowed the
  robot down by the time it reaches the radius.

Pick the radius from how much speed you want to carry — roughly 6–10" at full
speed. Too small and you've already decelerated; too large and the corner
rounds off badly.

Code between chained moves runs **while the robot is still driving**:

```cpp
chassis.moveToPoint(-24, -36, 1500, 8.0);
intake.move(127);      // good — spins up while still approaching
pros::delay(500);      // bad — 500ms of driving blind
```

### Running a mechanism at the same time

Motions block, but motor commands don't. If the mechanism just needs to be
running, start it first — no task needed:

```cpp
intake.move(127);                        // returns instantly, keeps spinning
chassis.moveToPose(24, 24, 180, 3000);   // blocks here, intake keeps going
intake.move(0);
```

If the mechanism has **timing of its own**, put it in a task:

```cpp
void scoreTask(void*) {
    intake.move(127);
    pros::delay(400);
    lift.move_absolute(600, 100);
    pros::delay(300);
    intake.move(-60);
}

void autonomous() {
    chassis.setPose(0, 0, 0);

    pros::Task t(scoreTask);                  // starts alongside
    chassis.moveToPose(24, 24, 180, 3000);    // robot drives while it runs
    t.remove();                               // in case the move finished first
}
```

Two rules:

- **Never call `chassis.*` from that task.** `Drivetrain` is not thread safe.
  Mechanism motors only — those are fine.
- **Call `t.remove()`** after the motion. Without it a half-finished sequence
  keeps running into your next move.

If instead the mechanism needs to react to *where the robot is*, tick the
motion yourself: construct a `MoveToPose` directly, call `start()`, then
`tick(chassis.getPose(), dt, drive, now)` in your own loop until it stops
returning `MotionStatus::Running`.

### Handling failure

```cpp
if (!chassis.moveToPoint(24, 24, 2000)) {
    // timed out — skip the scoring step rather than flailing
}
```

A *chained* move that fails is still moving, so stop it yourself:

```cpp
if (!chassis.moveToPoint(24, 24, 2000, 8.0)) {
    chassis.stop();
    return;
}
```

### Emergency stop

```cpp
chassis.cancelMotion();   // safe from another task, ends the motion next tick
chassis.clearCancel();    // required before any further motion will run
```

The flag **latches** — every later motion fails immediately until you clear it,
so an estop raised between motions is not swallowed.

### Driver control

```cpp
void opcontrol() {
    while (true) {
        chassis.update();                    // or the pose goes stale
        chassis.arcade(throttleVolts, turnVolts);
        pros::delay(10);
    }
}
```

`arcade(throttle, turn)` or `tank(left, right)`, both in volts. Autonomous pumps
`update()` for you; driver control must call it itself, at 100Hz (50Hz floor).

### Putting it together

```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);              // facing +Y

    chassis.moveToPoint(0, 24, 2000, 8.0); // chained
    chassis.moveToPoint(24, 36, 2000);     // settles

    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(48, 36, 180, 3000);
    chassis.driveDistance(-12, 1500);
}
```

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| Turn runs away instead of settling | left/right signs swapped in your drive HAL |
| Robot takes the long way round | IMU sign inverted — negate in the IMU class only |
| Pose jumps 360° once per revolution | using `get_heading()` instead of `get_rotation()` |
| Straight lines fine, square won't close | frame transposed, or an offset sign wrong |
| Pure rotation walks x/y | tracking wheel offset wrong — redo the spin test |
| Stops 1–2" short every time | drivetrain deadband — raise `move.minVolts` |
| Turn times out ~2° short | `kD` too heavy, or `timeoutMs` too tight |
| Robot orbits the target | `settleRadiusInches` too small |
| Stops short of the point | `settleRadiusInches` too large |
| Chaining buys no speed | `chainMinVolts` is 0, or the radius is too small |
| Robot won't stop after auton | last chained move still has a radius |
| "kI does nothing" | `iZone` is 0, so `fabs(err) < 0` is never true |
| Distance off by a constant factor | `inchesPerCount` |
| Everything is 57× off | degrees vs radians in the angular gains |
