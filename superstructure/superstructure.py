from commands2 import InstantCommand, StartEndCommand
from commands2.button import CommandGenericHID
from wpilib import Timer, SendableChooser, SmartDashboard
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

from .robot_state import RobotState, RobotReadiness
from .auxiliary_actions import AuxiliaryActions

from utils import log

# States that advance on their own once ready
NEXT_STATE = {
    RobotState.PREP_SHOT: RobotState.SHOOTING,
    RobotState.PREP_SHOT_AUTONOMOUS: RobotState.SHOOTING_AUTONOMOUS,
}


class Superstructure:
    """
    Owns the robot-wide RobotState and runs the matching subsystem logic each loop,
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


        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

        # Called every loop while in the given state
        self._state_handlers = {
            RobotState.IDLE: self._handle_idle,

            RobotState.INTAKING: self._handle_intaking,
            RobotState.INTAKING_AUTONOMOUS: self._handle_intaking,
            RobotState.INTAKE_DEPLOYED: self._handle_intake_deployed,
            RobotState.INTAKE_STOWED: self._handle_intake_stowed,
            RobotState.INTAKE_REVERSE: self._handle_intake_reverse,

            RobotState.PREP_SHOT: self._handle_prep_shot,
            RobotState.PREP_SHOT_AUTONOMOUS: self._handle_prep_shot,
            RobotState.SHOOTING: self._handle_shooting,
            RobotState.SHOOTING_AUTONOMOUS: self._handle_shooting,

            RobotState.PASSING_FUEL: self._handle_passing_fuel,
            RobotState.AGITATOR_OPPOSITE: self._handle_agitator_reverse,

            RobotState.PLAYING_SONG: self._handle_playing_song,
            RobotState.PLAYING_CHAMPIONSHIP_SONG: self._handle_playing_championship_song,
        }

        self._can_feed_since: float | None = None
        self._state_start_time = Timer.getFPGATimestamp()

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
        Logger.recordOutput("Superstructure/State", self.robot_state.name)
        Logger.recordOutput("Superstructure/StateValue", self.robot_state.value)
        Logger.recordOutput("Superstructure/TimeInState", Timer.getFPGATimestamp() - self._state_start_time)

        self._update_readiness()

        handler = self._state_handlers.get(self.robot_state)
        if handler:
            handler()

        self._handle_music_cleanup()
        self.auxiliary_actions.update()

    def _update_readiness(self):
        shooters = self._shooters()
        shooter_ready = bool(shooters) and all(s.atSpeed(tolerance_rpm=50) for s in shooters)
        self.robot_readiness.shooterReady = shooter_ready

        # 0.12s debounce so one at-speed sample doesn't start feeding
        now = Timer.getFPGATimestamp()
        if shooter_ready:
            if self._can_feed_since is None:
                self._can_feed_since = now
            can_feed = (now - self._can_feed_since) >= 0.12
        else:
            self._can_feed_since = None
            can_feed = False
        self.robot_readiness.canFeed = can_feed

        intake_deployed = self.intake is not None and self.intake.is_deployed()
        self.robot_readiness.intakeDeployed = intake_deployed

        if self.shooter is not None:
            Logger.recordOutput("Superstructure/Shooter1/CurrentRPS", self.shooter.getCurrentRPS())
            Logger.recordOutput("Superstructure/Shooter1/TargetRPS", self.shooter.getTargetRPS())
        if self.shooter2 is not None:
            Logger.recordOutput("Superstructure/Shooter2/CurrentRPS", self.shooter2.getCurrentRPS())
            Logger.recordOutput("Superstructure/Shooter2/TargetRPS", self.shooter2.getTargetRPS())
        if self.shotCalculator is not None:
            Logger.recordOutput("Superstructure/ShotCalc/TargetRPS", self.shotCalculator.getTargetSpeedRPS())
            Logger.recordOutput("Superstructure/ShotCalc/Distance", self.shotCalculator.getTargetDistance())

        Logger.recordOutput("Superstructure/Readiness/ShooterReady", shooter_ready)
        Logger.recordOutput("Superstructure/Readiness/CanFeed", can_feed)
        Logger.recordOutput("Superstructure/Readiness/IntakeDeployed", intake_deployed)

    def createStateCommand(self, state: RobotState):
        """Holds `state` while scheduled, then returns to IDLE unless another command has taken over."""
        def on_end():
            if self.robot_state in (state, NEXT_STATE.get(state)):
                self.setState(RobotState.IDLE)

        return StartEndCommand(lambda: self.setState(state), on_end)

    def autoCreateStateCommand(self, state: RobotState):
        return InstantCommand(lambda: self.setState(state))

    def setState(self, newState: RobotState, force: bool = False):
        """
        :param force: Re-enter the state even if it's already active (resets timers).
        """
        if not force and newState == self.robot_state:
            return

        oldState = self.robot_state
        self.robot_state = newState
        self._state_start_time = Timer.getFPGATimestamp()
        self._can_feed_since = None

        log("Superstructure", f"{oldState.name} -> {newState.name}")
        Logger.recordOutput("Superstructure/LastTransition", f"{oldState.name} -> {newState.name}")
        Logger.recordOutput("Superstructure/TransitionTimestamp", self._state_start_time)

    def getState(self) -> RobotState:
        return self.robot_state

    # State handlers

    def _handle_idle(self):
        # Pivot holds its last position; only velocity mechanisms stop
        self._stop_shooter()
        self._stop_feeders()
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

    def _handle_prep_shot(self):
        self._spin_up_shooters()
        self._stop_feeders()
        if self.robot_readiness.canFeed:
            self.setState(NEXT_STATE[self.robot_state])

    def _handle_shooting(self):
        self._spin_up_shooters()
        self._pulse_intake()
        self._feed_shooters()

    def _handle_passing_fuel(self):
        self._spin_up_shooters_dashboard()
        if self.robot_readiness.shooterReady:
            self._feed_shooters()
        else:
            self._stop_feeders()

    def _handle_agitator_reverse(self):
        if self.agitator is not None:
            self.agitator.reverse()

    def _handle_playing_song(self):
        if self.orchestra is not None:
            self.orchestra.play_selected_song()

    def _handle_playing_championship_song(self):
        if self.orchestra is not None:
            self.orchestra.play_championship_song()

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
        t = Timer.getFPGATimestamp() - self._state_start_time
        if (t % 3.0) > 1.5:
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

    # Orchestra

    def _handle_music_cleanup(self):
        if self.orchestra is not None and self.robot_state not in (
            RobotState.PLAYING_SONG, RobotState.PLAYING_CHAMPIONSHIP_SONG
        ):
            self.orchestra.stop()
