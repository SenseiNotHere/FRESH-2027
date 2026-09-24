# FRESH 2027

FRC Team 1811's 2027 competition robot codebase.

Same brain as last year.
New body, still under construction.

Right now the robot can drive, reset her pose, X-brake, and sing.
Everything else gets added as states, one at a time, on purpose.

---

# 🚧 Heads Up: This Is a Rough Cut

Let's be honest: this was a pretty bad attempt at turning a full season codebase into drivetrain-only code.
There are MANY, MANY leftovers for stuff that doesn't exist anymore: constants, helpers, readiness flags, and references to mechanisms that were stripped out. If something looks like it belongs to a robot that isn't here, it probably does.

This repo is mostly about getting ready for 2027, because FIRST and WPILib decided, out of nowhere, to change 100,000 things at once.
Expect things to break when we upgrade. Expect cleanup. Expect more cleanup after that.
After the cleanup come the fixes. Then more fixes. Then even more fixes.

Status: MAYBE test ready. Emphasis on maybe.

---

# 🧠 Architecture Philosophy

Buttons don't talk to motors here.

Buttons ask for a state. The superstructure reads that state every loop and tells the subsystems what to do.

A state describes what the robot is trying to do: `IDLE`, `PLAYING_SONG`, and later whatever this game asks of us. One handler per state. One place to look when something acts weird.

Teleop and auto set the same states, so they run the same code.
If auto works, teleop works. If one breaks, they both break, and you only fix it once.

The loop, every 20 ms:

1. `robotPeriodic()` runs the command scheduler
2. `Superstructure.update()` refreshes readiness
3. The handler for the current state runs

That's the whole trick.

---

# ➕ Adding a New State

States are how this robot thinks.

Adding one takes about five minutes of typing and ten minutes of thinking. Spend the ten.

------------------------------------------------------------------------

## 1️⃣ Define the State

Open:

    superstructure/robot_state.py

Add a value to `RobotState`:

``` python
class RobotState(Enum):

    # General
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2

    # Your new stuff
    INTAKING = 3
```

Name it after what the robot is doing, not which motor spins.

Bad:

``` python
RUN_ROLLERS_FAST
```

Good:

``` python
INTAKING
PREP_SHOT
ALIGNING_TO_TARGET
```

------------------------------------------------------------------------

## 2️⃣ Route It in `__init__()`

Open:

    superstructure/superstructure.py

Add your state to the `_state_handlers` dictionary:

``` python
self._state_handlers = {
    RobotState.IDLE: self._handle_idle,
    ...
    RobotState.INTAKING: self._handle_intaking,
}
```

A state with no entry here does nothing. No error, no warning, just vibes. Don't forget this step.

------------------------------------------------------------------------

## 3️⃣ Write the Handler

Open:

    superstructure/superstructure_states.py

Add a method to `SuperstructureStates`:

``` python
def _handle_intaking(self: "Superstructure"): # type: ignore
    if not self.hasIntake:
        return

    self.intake.deploy()
    self.intake.runRollers()
```

(`hasIntake` and `intake` don't exist yet. Add the subsystem first, then its availability flag next to `hasOrchestra` in `superstructure.py`.)

Keep the `self: "Superstructure"` type hint. Without it your editor can't see `self.drivetrain`, `self.orchestra`, or anything else, and autocomplete gives up on you.

Handlers:

-   Coordinate subsystems
-   Never touch hardware directly
-   Never skip a subsystem's own safety checks

Subsystems do the work.
The superstructure decides which work.

The handler runs every loop while the state is active, so write it to be safe to call 50 times a second.

------------------------------------------------------------------------

## 4️⃣ Helpers

Shared logic that more than one handler needs goes in:

    superstructure/superstructure_helpers.py

`_stop_orchestra()` and `_rumble_controller()` already live there. Copy their shape.

------------------------------------------------------------------------

## 5️⃣ (Optional) Readiness

If your state depends on a condition (shooter at speed, intake deployed, target locked), that condition is a readiness flag.

Readiness flags live in one place: the `RobotReadiness` dataclass in `superstructure/robot_state.py`. Please don't scatter them across the codebase.

To add one:

``` python
@dataclass
class RobotReadiness:
    intakeDeployed: bool = False
```

Then update it in `_update_readiness()` in `superstructure.py`. That method runs before every handler, so handlers always see fresh values.

Want to use `setRobotReadiness()` / `getRobotReadiness()`? Add a matching entry to `ReadinessList`. The value has to be the exact field name, because that's what `setattr` looks up:

``` python
class ReadinessList(Enum):
    INTAKE_DEPLOYED = "intakeDeployed"
```

Totally optional.

------------------------------------------------------------------------

## 6️⃣ Enter the State

From a button binding in `button_bindings.py`:

``` python
self.driverController.button(XboxController.Button.kA).whileTrue(
    self.superstructure.createStateCommand(RobotState.INTAKING)
)
```

`createStateCommand` sets the state when the command starts and drops back to `IDLE` when it ends, as long as nothing else changed the state in between. Hold the button, the robot intakes. Let go, she stops.

For autonomous, use `autoCreateStateCommand`. It sets the state and finishes right away so the path keeps moving:

``` python
NamedCommands.registerCommand(
    "Intake", superstructure.autoCreateStateCommand(RobotState.INTAKING)
)
```

Don't:

-   Bind buttons straight to motor outputs
-   Put superstructure logic inside commands

------------------------------------------------------------------------

## 🧠 Is This a State?

Ask:

-   Is this something the robot is *trying to do*?
-   Would auto want it too?

Yes to both: it's a state.
No: it's probably a subsystem method or a helper.

---

# ⚙ Subsystem Overview

## 🔄 Drivetrain: Kraken Swerve

Located in `subsystems/drive/`

- SDS MK5n L2 modules (5.36:1 drive, 18.75:1 steer)
- TalonFX (Kraken) drive and steer motors, CANcoder absolute encoders
- Pigeon 2 gyro
- Built on the Phoenix 6 `SwerveDrivetrain`
- Torque-current FOC on the drive motors
- 250 Hz odometry
- Field-relative driving with slew rate limiting
- 26.5" × 26.5" wheelbase, capped at 3.0 m/s

Pose shows up on the dashboard `Field2d`, and the selected auto path gets drawn on it while disabled.

---

## 🤖 Autonomous

Located in `subsystems/drive/autonomous_subsystem.py`

- PathPlanner `AutoBuilder` with a holonomic controller
- Paths flip automatically on red alliance
- Named commands and event triggers get registered in `registerNamedCommands()` / `registerEventTriggers()` (both empty for now)
- Assets in `deploy/pathplanner/`

Autos set superstructure states, same as teleop. One brain.

Heads up: the auto chooser in `robot_container.py` exists but has no options and isn't on the dashboard yet. Wire that up before our first match or auto will do nothing.

---

## 🎵 Orchestra

Located in `subsystems/orchestra/`

- Phoenix 6 Orchestra
- Takes any subsystem with a `getMotors()` method and turns its Krakens into instruments (right now: the drivetrain)
- Song picker on the dashboard under `Song Selection`
- Plays `.chrp` files from `deploy/files/`

Current setlist:

- Yes, And? (Ariana Grande), the default
- Espresso (Sabrina Carpenter)
- Needy (Ariana Grande)
- Dandelion (Ariana Grande)
- When Did You Get Hot (Sabrina Carpenter)
- Tití Me Preguntó (Bad Bunny)
- Stateside (PinkPantheress)
- Despacito (Luis Fonsi)

There's also a championship song. It's locked behind `_championship_mode`, which is `False` and has no toggle.
We'll unlock it when we earn it.

Yes, the robot sings.
Yes, it's mostly Ariana.
No, we will not be taking requests. (We might be taking requests.)

---

# 🎮 Controls

Driver controller on port 0. Operator controller on port 1 (nothing bound yet).

| Input | What it does |
|---|---|
| Left stick | Drive (field-relative) |
| Right stick X | Rotate |
| Left trigger | Hold for full speed. Otherwise she drives at half. |
| B (hold) | X-brake: wheels lock in an X so nobody pushes you |
| D-pad up | Reset pose to (0, 0), heading 0° |
| D-pad down | Reset robot front |

---

# 📊 Logging

Logging runs through pykit.

- Real robot: writes a `.wpilog` and publishes to NetworkTables
- Simulation: publishes to NetworkTables only
- Replay: set up, not implemented yet

The PDH gets logged too. If the console fills up with `CAN: Message not Found`, set `RobotConstants.kLogPDHChannels = False`. To turn PDH logging off completely, set `kEnablePDHLogging = False`.

---

# 🧰 Utils

`utils/` has a few decorators you should actually use:

- `@teleop_only`: blocks the call outside teleop and logs that it did
- `@throttle(seconds)`: runs at most once per cooldown
- `@fail_safe(fallback)`: catches exceptions so one broken sensor can't crash the loop, logs the error, and returns the fallback
- `@experimental`: warns the driver station the first time it runs

Plus `InterpolatingMap` for lookup tables (distance → shooter speed, that kind of thing), and `log()` / `print_banner()` for console output.

---

# 🛠 Tech Stack

- Python
- RobotPy 2026.2.2 + WPILib
- Commands v2
- Phoenix 6 (26.1.2)
- REVLib
- PathPlanner
- pykit

---

# ⚠ IMPORTANT WARNINGS (PLEASE READ BEFORE YOU FAFO)

## 🔧 Robot-Specific Constants

Every constant in this repo is tuned for our robot.

That means everything in `constants/`:

- Gear ratios
- Motor inversions
- Encoder offsets
- CAN IDs
- Robot dimensions
- PID gains
- Current limits

If you copy this onto your robot without changing them... respectfully... that's wild.

Before you enable, you have to:

- Verify every CAN ID
- Verify every inversion
- Re-zero your encoder offsets
- Retune every PID loop
- Measure your own dimensions

Skip that and you risk:

- Broken mechanisms
- A robot that drives somewhere you didn't tell it to
- Wrong field position in auto
- Someone getting hurt

Yes, hurt.

A misconfigured control loop can make the robot move fast and without warning.

By using this repository, you accept full responsibility for implementing it safely.
FRC Team 1811 is not liable for damage, injury, or misuse.

This isn't a plug-and-play template. It's our robot's code.

---

## 🔐 Phoenix 6 Pro Requirement

The drivetrain uses torque-current FOC, and that needs Phoenix 6 Pro.

Without:

- A valid Phoenix Pro license on your devices
- Phoenix 6 installed correctly
- Firmware that matches

the swerve modules will not behave the way the code expects.

Set up your CTRE stack before you deploy.

We are not debugging your licensing.

---

# 🧪 Development Setup

## 1. Create a virtual environment

    python -m venv .venv

Activate it:

**Windows (PowerShell):**

    .\.venv\Scripts\Activate.ps1

**Windows (Command Prompt):**

    .\.venv\Scripts\activate.bat

**macOS/Linux:**

    source .venv/bin/activate

## 2. Install dependencies

    pip install -r requirements.txt

## 3. Sync RobotPy dependencies

    robotpy sync

## 4. Run the simulator

    robotpy sim

`physics.py` moves the simulated robot by integrating whatever speeds the drivetrain commands. It's simple, and it's enough to check that field-relative driving points the right way.

## 5. Run the tests

    robotpy test

This boots the robot in disabled, auto, and teleop for a moment each. If anything crashes on startup, you'll find out here instead of on the field.

## 6. DEPLOY AND ENJOY!!

    robotpy deploy --skip-tests

---

# 📁 Project Structure

    robot.py              # Robot lifecycle + logging setup
    robot_container.py    # Builds subsystems, superstructure, default commands
    button_bindings.py    # Controller → command mappings
    physics.py            # Simulation
    pyproject.toml        # RobotPy version + roboRIO packages
    requirements.txt
    commands/             # Commands, grouped by subsystem
    constants/            # constants.py, field_constants.py
    deploy/               # PathPlanner assets + .chrp songs
    subsystems/           # drive/, orchestra/
    superstructure/       # State machine
    tests/
    utils/

---

# 🤝 How to Contribute

Hi. If you're reading this, you care enough to build it right. We appreciate you.

## 🧠 Before You Add Code

- Go through the superstructure.
- Don't bind buttons to motor outputs.
- Don't hardcode numbers outside `constants/`.
- Not sure where something goes? Ask.

---

## 🗂 Where Things Go

- `subsystems/`: hardware logic (currently `drive/` and `orchestra/`)
- `commands/`: behaviors, grouped by subsystem
- `superstructure/`: robot intent
    - `superstructure.py`: the state machine and its public API
    - `superstructure_states.py`: one handler per state
    - `superstructure_helpers.py`: shared helper methods
    - `robot_state.py`: `RobotState`, `RobotReadiness`, `ReadinessList`
    - `auxiliary_actions.py`: side actions that run outside the state machine (empty for now)
- `constants/`: every tunable number
- `deploy/`: PathPlanner files and songs
- `utils/`: decorators, `InterpolatingMap`, logging helpers
- `button_bindings.py`: every controller binding, in one file

Adding a subsystem? Construct it in `robot_container.py` before the superstructure. The superstructure has to be built last because it takes every subsystem as an argument.

If it feels like a hack, it probably is.

---

## 🧪 Test First

- Run `robotpy test` and the sim before you deploy.
- Check motor directions after every mechanical change.
- Put the robot on blocks the first time new code runs.

The robot moves fast.
Your mistakes move with it.

---

Keep it readable.
Keep it intentional.
Keep it FRESH.

---

Built with love by FRC Team 1811, FRESH.
