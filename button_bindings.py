from wpilib import XboxController
from commands2 import RunCommand
from commands2.button import CommandGenericHID

from commands import ResetSwerveFront, ResetXY
from superstructure import IntakeState, ScoringState

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_container import RobotContainer
    from superstructure import Superstructure


class ButtonBindings:
    """
    Button bindings for the robot. Connects controllers to the commands that
    operate on the subsystems, kept separate from RobotContainer so the mappings
    stay focused.

    This is a single-instance class. Constructing it a second time raises a
    RuntimeError; use ButtonBindings.getInstance() to reach the existing one.
    """
    _instance: "ButtonBindings | None" = None

    def __init__(
            self,
            robot_container: RobotContainer,
            superstructure: Superstructure,
            driver_controller: CommandGenericHID,
            operator_controller: CommandGenericHID
            ):
        if ButtonBindings._instance is not None:
            raise RuntimeError(
                "ButtonBindings is a single-instance class but was constructed twice. "
                "Use ButtonBindings.getInstance() to reuse the existing instance."
            )

        self.robotContainer = robot_container
        self.superstructure = superstructure
        self.driverController = driver_controller
        self.operatorController = operator_controller

        ButtonBindings._instance = self

    @classmethod
    def getInstance(cls) -> "ButtonBindings":
        if cls._instance is None:
            raise RuntimeError("ButtonBindings has not been constructed yet.")
        return cls._instance

    def configureButtonBindings(self):
        self._configureDriverBindings()
        self._configureOperatorBindings()

    def _configureDriverBindings(self):
                # Reset Controls
        # Reset XYZ
        self.driverController.pov(0).onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotContainer.drive_subsystem,

            )
        )

        # Reset Robot Front
        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.robotContainer.drive_subsystem)
        )
        
        # X-Break
        self.driverController.button(
            XboxController.Button.kB
        ).whileTrue(
            RunCommand(self.robotContainer.drive_subsystem.setX, self.robotContainer.drive_subsystem)
        )

        # Right Trigger = Prep Shot, moves to SHOOTING on its own once the shooters are at speed
        self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger, 0.1
        ).whileTrue(
            self.superstructure.createStateCommand(ScoringState.PREP_SHOT)
        )

        # Y = Agitator Opposite
        self.driverController.button(
            XboxController.Button.kY
        ).whileTrue(
            self.superstructure.createStateCommand(ScoringState.AGITATOR_OPPOSITE)
        )

    def _configureOperatorBindings(self):
        # Right Trigger = Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger, 0.05
        ).whileTrue(
            self.superstructure.createStateCommand(IntakeState.INTAKING)
        )

        # Left Trigger = Reverse Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger, 0.05
        ).whileTrue(
            self.superstructure.createStateCommand(IntakeState.REVERSE)
        )

        # Left Bumper = Stow, Right Bumper = Deploy (latched until another state)
        self.operatorController.button(
            XboxController.Button.kLeftBumper
        ).onTrue(
            self.superstructure.autoCreateStateCommand(IntakeState.STOWED)
        )
        self.operatorController.button(
            XboxController.Button.kRightBumper
        ).onTrue(
            self.superstructure.autoCreateStateCommand(IntakeState.DEPLOYED)
        )
