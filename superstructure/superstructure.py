from enum import Enum

from commands2 import InstantCommand, StartEndCommand
from commands2.button import CommandGenericHID
from wpilib import DriverStation, Timer, SendableChooser, SmartDashboard
from wpimath.filter import Debouncer
from pykit.logger import Logger

from subsystems import (
    DriveSubsystem,
    IntakeSubsystem,
    ShooterSubsystem,
    ShotCalculator,
    IndexerSubsystem,
    AgitatorSubsystem,
    LimelightCamera,
    OrchestraSubsystem,
)

from .robot_state import MusicState, IntakeState, ScoringState
from .auxiliary_actions import AuxiliaryActions

from utils import log, LoggedChooser

# Releasing a command that set PREP_SHOT also stops SHOOTING, since PREP_SHOT advances there on its own
LINKED_STATES = {ScoringState.PREP_SHOT: ScoringState.SHOOTING}


class Superstructure:
    """
    Holds one state per enum and runs each state's handler every loop,
    so subsystems never command each other directly. Every subsystem is optional.

    Single instance: construct once, then use Superstructure.getInstance().
    """
    _instance: "Superstructure | None" = None

    def __init__(
            self,
            drivetrain: DriveSubsystem | None = None,
            intake: IntakeSubsystem | None = None,
            shooter: ShooterSubsystem | None = None,
            shooter2: ShooterSubsystem | None = None,
            indexer: IndexerSubsystem | None = None,
            agitator: AgitatorSubsystem | None = None,
            shotCalculator: ShotCalculator | None = None,
            vision: LimelightCamera | None = None,
            orchestra: OrchestraSubsystem | None = None,
            driverController: CommandGenericHID | None = None,
            operatorController: CommandGenericHID | None = None,
    ):
        if Superstructure._instance is not None:
            raise RuntimeError(
                "Superstructure is a single-instance class but was constructed twice. "
                "Use Superstructure.getInstance() to reuse the existing instance."
            )

        self.drivetrain = drivetrain
        self.intake = intake
        self.shooter = shooter
        self.shooter2 = shooter2
        self.indexer = indexer
        self.agitator = agitator
        self.shotCalculator = shotCalculator
        self.vision = vision
        self.orchestra = orchestra
        self.driverController = driverController
        self.operatorController = operatorController
        self.shooters = [s for s in (shooter, shooter2) if s is not None]

        # Add a new state variable here: its enum (with an IDLE) and nothing else
        self.states: dict[type[Enum], Enum] = {
            MusicState: MusicState.IDLE,
            IntakeState: IntakeState.IDLE,
            ScoringState: ScoringState.IDLE,
        }
        now = Timer.getTimestamp()
        self._entered_at = {enum: now for enum in self.states}

        self._handlers = {
            MusicState.IDLE: self._handle_music_idle,
            MusicState.PLAYING_SONG: self._handle_playing_song,
            MusicState.PLAYING_CHAMPIONSHIP_SONG: self._handle_playing_championship_song,

            IntakeState.IDLE: self._handle_intake_idle,
            IntakeState.DEPLOYED: self._handle_intake_deployed,
            IntakeState.STOWED: self._handle_intake_stowed,
            IntakeState.INTAKING: self._handle_intaking,
            IntakeState.REVERSE: self._handle_intake_reverse,

            ScoringState.IDLE: self._handle_scoring_idle,
            ScoringState.PREP_SHOT: self._handle_prep_shot,
            ScoringState.SHOOTING: self._handle_shooting,
            ScoringState.PASSING_FUEL: self._handle_passing_fuel,
            ScoringState.AGITATOR_OPPOSITE: self._handle_agitator_reverse,
        }

        # Feeding starts once the shooter has been at speed for 0.1s and only stops after
        # it has been off speed for 0.1s, so neither a lucky sample nor each ball's RPM dip flips it
        self._feed_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.shooterReady = False
        self.canFeed = False

        self.auxiliary_actions = AuxiliaryActions(self.driverController)

        self.shotCalcChooser = LoggedChooser("Shot Calculator")
        self.shotCalcChooser.setDefaultOption("on", True)
        self.shotCalcChooser.addOption("off", False)

        Superstructure._instance = self

    @classmethod
    def getInstance(cls) -> "Superstructure":
        if cls._instance is None:
            raise RuntimeError("Superstructure has not been constructed yet.")
        return cls._instance

    def update(self):
        """Call from robotPeriodic()."""
        # Nothing latched in auto carries into teleop; music keeps playing while disabled
        if DriverStation.isDisabled():
            self.setState(IntakeState.IDLE)
            self.setState(ScoringState.IDLE)

        self._update_readiness()

        for enum, state in self.states.items():
            Logger.recordOutput(f"Superstructure/{enum.__name__}/State", state.name)
            Logger.recordOutput(f"Superstructure/{enum.__name__}/TimeInState", self._time_in_state(enum))
            self._handlers[state]()

        self.auxiliary_actions.update()

    def createStateCommand(self, state: Enum):
        """Holds `state` while scheduled, then returns its variable to IDLE unless another command has taken over."""
        enum = type(state)

        def on_end():
            if self.states[enum] in (state, LINKED_STATES.get(state)):
                self.setState(enum.IDLE)

        return StartEndCommand(lambda: self.setState(state), on_end)

    def autoCreateStateCommand(self, state: Enum):
        """Sets `state` and leaves it there until something else changes that variable."""
        return InstantCommand(lambda: self.setState(state))

    def getState(self, enum: type[Enum]) -> Enum:
        return self.states[enum]

    def setState(self, newState: Enum, force: bool = False):
        """
        Sets the variable `newState` belongs to; the other variables are untouched.

        :param force: Re-enter the state even if it's already active (resets its timer).
        """
        enum = type(newState)
        oldState = self.states[enum]
        if not force and newState == oldState:
            return

        self.states[enum] = newState
        self._entered_at[enum] = Timer.getTimestamp()

        log("Superstructure", f"{enum.__name__}: {oldState.name} -> {newState.name}")
        Logger.recordOutput(f"Superstructure/{enum.__name__}/LastTransition", f"{oldState.name} -> {newState.name}")

    def _time_in_state(self, enum: type[Enum]) -> float:
        return Timer.getTimestamp() - self._entered_at[enum]

    def _update_readiness(self):
        self.shooterReady = len(self.shooters) > 0 and all(s.atSpeed(tolerance_rpm=50) for s in self.shooters)
        self.canFeed = self._feed_debouncer.calculate(self.shooterReady)

        for i, shooter in enumerate(self.shooters, start=1):
            Logger.recordOutput(f"Superstructure/Shooter{i}/CurrentRPS", shooter.getCurrentRPS())
            Logger.recordOutput(f"Superstructure/Shooter{i}/TargetRPS", shooter.getTargetRPS())
        if self.shotCalculator is not None:
            Logger.recordOutput("Superstructure/ShotCalc/TargetRPS", self.shotCalculator.getTargetSpeedRPS())
            Logger.recordOutput("Superstructure/ShotCalc/Distance", self.shotCalculator.getTargetDistance())
        Logger.recordOutput("Superstructure/Readiness/ShooterReady", self.shooterReady)
        Logger.recordOutput("Superstructure/Readiness/CanFeed", self.canFeed)

    # Music

    def _handle_music_idle(self):
        if self.orchestra is not None:
            self.orchestra.stop()

    def _handle_playing_song(self):
        if self.orchestra is not None:
            self.orchestra.play_selected_song()

    def _handle_playing_championship_song(self):
        if self.orchestra is not None:
            self.orchestra.play_championship_song()

    # Intake

    def _handle_intake_idle(self):
        # Nobody is using the intake, so shooting may pulse it. Pivot otherwise holds its last position.
        if self.intake is None:
            return
        if self.states[ScoringState] == ScoringState.SHOOTING:
            self._pulse_intake()
        else:
            self.intake.stop_intake()

    def _handle_intaking(self):
        if self.intake is not None:
            self.intake.deploy()
            self.intake.intake()

    def _handle_intake_reverse(self):
        if self.intake is not None:
            self.intake.stow()
            self.intake.intake_reverse()

    def _handle_intake_deployed(self):
        if self.intake is not None:
            self.intake.deploy()
            self.intake.stop_intake()

    def _handle_intake_stowed(self):
        if self.intake is not None:
            self.intake.stow()
            self.intake.stop_intake()

    def _pulse_intake(self):
        """Alternate pulse position (rollers on) and deploy position (rollers off) every 1.5s."""
        if (self._time_in_state(ScoringState) % 3.0) > 1.5:
            self.intake.go_to_pulse_position()
            self.intake.intake()
        else:
            self.intake.deploy()
            self.intake.stop_intake()

    # Scoring

    def _handle_scoring_idle(self):
        for shooter in self.shooters:
            shooter.stop()
        self._stop_feeders()

    def _handle_prep_shot(self):
        # Give the shooters time to reach the shot calculator's speed before any fuel goes in
        self._spin_up_shooters()
        self._stop_feeders()
        if self.canFeed:
            self.setState(ScoringState.SHOOTING)

    def _handle_shooting(self):
        # Feeds until released; once at speed, losing RPM mid-volley doesn't matter
        self._spin_up_shooters()
        self._feed_shooters()

    def _handle_passing_fuel(self):
        for shooter in self.shooters:
            shooter.useDashboardPercent()
        # Checked every loop, so feeding stops if the shooter falls off speed
        if self.canFeed:
            self._feed_shooters()
        else:
            self._stop_feeders()

    def _handle_agitator_reverse(self):
        if self.agitator is not None:
            self.agitator.reverse()

    def _spin_up_shooters(self):
        """Use the shot calculator's speed when available and enabled, else the dashboard percent."""
        calc = self.shotCalculator if self.shotCalcChooser.getSelected() else None
        target_rps = calc.getTargetSpeedRPS() if calc is not None else None
        for shooter in self.shooters:
            if target_rps is None:
                shooter.useDashboardPercent()
            else:
                shooter.setTargetRPS(target_rps)

    def _feed_shooters(self):
        if self.indexer is not None:
            self.indexer.feed()
        if self.agitator is not None:
            self.agitator.feed()

    def _stop_feeders(self):
        if self.indexer is not None:
            self.indexer.stop()
        if self.agitator is not None:
            self.agitator.stop()
