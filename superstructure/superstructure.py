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

from utils import log

# State enum -> attribute holding it. Logged under Superstructure/<EnumName>/.
STATE_VARIABLES = {
    MusicState: "music_state",
    IntakeState: "intake_state",
    ScoringState: "scoring_state",
}

# States that hand off to a partner state on their own. A command holding either one
# still counts as holding its state, so releasing the button returns to IDLE.
LINKED_STATES = {
    ScoringState.PREP_SHOT: ScoringState.SHOOTING,
}

# Reset to IDLE whenever the robot is disabled, so nothing latched in auto carries into teleop.
# Music is left alone so songs can play while disabled.
RESET_ON_DISABLE = (IntakeState, ScoringState)


class Superstructure:
    """
    Owns one state variable per enum in STATE_VARIABLES and runs each one's
    handler every loop, so subsystems never command each other directly.
    Every subsystem is optional.

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

        self.music_state = MusicState.IDLE
        self.intake_state = IntakeState.IDLE
        self.scoring_state = ScoringState.IDLE

        # Called every loop while in the given state
        self._state_handlers = {
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

        now = Timer.getFPGATimestamp()
        self._state_start_time = {enum: now for enum in STATE_VARIABLES}

        # Feeding starts once the shooter has been at speed for 0.1s and only stops after
        # it has been off speed for 0.1s, so neither a lucky sample nor each ball's RPM dip flips it
        self._feed_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.shooterReady = False
        self.canFeed = False

        self.auxiliary_actions = AuxiliaryActions(self.driverController)

        self.shotCalcChooser = SendableChooser()
        self.shotCalcChooser.setDefaultOption("on", True)
        self.shotCalcChooser.addOption("off", False)
        SmartDashboard.putData("Shot Calculator", self.shotCalcChooser)

        Superstructure._instance = self

    @classmethod
    def getInstance(cls) -> "Superstructure":
        if cls._instance is None:
            raise RuntimeError("Superstructure has not been constructed yet.")
        return cls._instance

    def update(self):
        """Call from robotPeriodic()."""
        if DriverStation.isDisabled():
            for enum in RESET_ON_DISABLE:
                self.setState(enum.IDLE)

        self._update_readiness()

        now = Timer.getFPGATimestamp()
        for enum in STATE_VARIABLES:
            state = self.getState(enum)
            Logger.recordOutput(f"Superstructure/{enum.__name__}/State", state.name)
            Logger.recordOutput(f"Superstructure/{enum.__name__}/TimeInState", now - self._state_start_time[enum])
            SmartDashboard.putString(f"Superstructure/{enum.__name__}/State", state.name)
            SmartDashboard.putNumber(f"Superstructure/{enum.__name__}/TimeInState", now - self._state_start_time[enum])
            self._state_handlers[state]()

        self.auxiliary_actions.update()

    def createStateCommand(self, state: Enum):
        """Holds `state` while scheduled, then returns its variable to IDLE unless another command has taken over."""
        def on_end():
            if self.getState(type(state)) in (state, LINKED_STATES.get(state)):
                self.setState(type(state).IDLE)

        return StartEndCommand(lambda: self.setState(state), on_end)

    def autoCreateStateCommand(self, state: Enum):
        """Sets `state` and leaves it there until something else changes that variable."""
        return InstantCommand(lambda: self.setState(state))

    def getState(self, enum: type[Enum]) -> Enum:
        return getattr(self, STATE_VARIABLES[enum])

    def setState(self, newState: Enum, force: bool = False):
        """
        Sets the variable `newState` belongs to; the other variables are untouched.

        :param force: Re-enter the state even if it's already active (resets its timer).
        """
        enum = type(newState)
        oldState = self.getState(enum)
        if not force and newState == oldState:
            return

        setattr(self, STATE_VARIABLES[enum], newState)
        self._state_start_time[enum] = Timer.getFPGATimestamp()

        log("Superstructure", f"{enum.__name__}: {oldState.name} -> {newState.name}")
        Logger.recordOutput(f"Superstructure/{enum.__name__}/LastTransition", f"{oldState.name} -> {newState.name}")

    def _time_in_state(self, enum: type[Enum]) -> float:
        return Timer.getFPGATimestamp() - self._state_start_time[enum]

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

    # Readiness

    def _update_readiness(self):
        # Shooter is ready when every shooter we have is at speed
        shooters = self._shooters()
        self.shooterReady = len(shooters) > 0 and all(s.atSpeed(tolerance_rpm=50) for s in shooters)
        self.canFeed = self._feed_debouncer.calculate(self.shooterReady)

        # Logging
        if self.shooter is not None:
            Logger.recordOutput("Superstructure/Shooter1/CurrentRPS", self.shooter.getCurrentRPS())
            Logger.recordOutput("Superstructure/Shooter1/TargetRPS", self.shooter.getTargetRPS())
        if self.shooter2 is not None:
            Logger.recordOutput("Superstructure/Shooter2/CurrentRPS", self.shooter2.getCurrentRPS())
            Logger.recordOutput("Superstructure/Shooter2/TargetRPS", self.shooter2.getTargetRPS())
        if self.shotCalculator is not None:
            Logger.recordOutput("Superstructure/ShotCalc/TargetRPS", self.shotCalculator.getTargetSpeedRPS())
            Logger.recordOutput("Superstructure/ShotCalc/Distance", self.shotCalculator.getTargetDistance())

        Logger.recordOutput("Superstructure/Readiness/ShooterReady", self.shooterReady)
        Logger.recordOutput("Superstructure/Readiness/CanFeed", self.canFeed)

    # Intake handlers

    def _handle_intake_idle(self):
        # Nobody is using the intake, so shooting may pulse it. Pivot otherwise holds its last position.
        if self.scoring_state == ScoringState.SHOOTING:
            self._pulse_intake()
        else:
            self._stop_intake_rollers()

    def _handle_intaking(self):
        self._deploy_intake_pivot()
        self._start_intake_rollers()

    def _handle_intake_reverse(self):
        self._stow_intake_pivot()
        self._reverse_intake_rollers()

    def _handle_intake_deployed(self):
        self._deploy_intake_pivot()
        self._stop_intake_rollers()

    def _handle_intake_stowed(self):
        self._stow_intake_pivot()
        self._stop_intake_rollers()

    # Scoring handlers

    def _handle_scoring_idle(self):
        self._stop_shooter()
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
        self._spin_up_shooters_dashboard()
        self._feed_when_ready()

    def _handle_agitator_reverse(self):
        if self.agitator is not None:
            self.agitator.reverse()

    # Intake

    def _stow_intake_pivot(self):
        if self.intake is not None:
            self.intake.stow()

    def _deploy_intake_pivot(self):
        if self.intake is not None:
            self.intake.deploy()

    def _start_intake_rollers(self):
        if self.intake is not None:
            self.intake.intake()

    def _stop_intake_rollers(self):
        if self.intake is not None:
            self.intake.stop_intake()

    def _reverse_intake_rollers(self):
        if self.intake is not None:
            self.intake.intake_reverse()

    def _pulse_intake(self):
        """Alternate pulse position (rollers on) and deploy position (rollers off) every 1.5s."""
        if self.intake is None:
            return
        if (self._time_in_state(ScoringState) % 3.0) > 1.5:
            self.intake.go_to_pulse_position()
            self.intake.intake()
        else:
            self.intake.deploy()
            self.intake.stop_intake()

    # Shooter

    def _shooters(self) -> list[ShooterSubsystem]:
        return [s for s in (self.shooter, self.shooter2) if s is not None]

    def _stop_shooter(self):
        for shooter in self._shooters():
            shooter.stop()

    def _spin_up_shooters(self):
        """Use the shot calculator's speed when available and enabled, else the dashboard percent."""
        calc = self.shotCalculator if self.shotCalcChooser.getSelected() else None
        target_rps = calc.getTargetSpeedRPS() if calc is not None else None
        for shooter in self._shooters():
            if target_rps is None:
                shooter.useDashboardPercent()
            else:
                shooter.setTargetRPS(target_rps)

    def _spin_up_shooters_dashboard(self):
        for shooter in self._shooters():
            shooter.useDashboardPercent()

    # Feeders

    def _feed_when_ready(self):
        """Checked every loop, so feeding stops if the shooter falls off speed."""
        if self.canFeed:
            self._feed_shooters()
        else:
            self._stop_feeders()

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
