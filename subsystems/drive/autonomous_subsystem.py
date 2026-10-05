from commands2 import Subsystem
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, DriveFeedforwards
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.auto import NamedCommands, EventTrigger
from pathplannerlib.util import FlippingUtil

from wpimath.kinematics import ChassisSpeeds

from constants import AutoConstants

from .drive_subsystem import DriveSubsystem

from utils import log


class AutonomousSubsystem(Subsystem):
    def __init__(
            self,
            drivetrain: DriveSubsystem,
    ):
        """
        Autonomous Subsystem class. Handles all autonomous-related functionality.
        This is a singleton class. Meaning there should only ever be one instance of this class.

        :param drivetrain: The drivetrain subsystem.
        :param robotContainer: The robot container.
        """
        super().__init__()

        self.drivetrain = drivetrain

        AutoBuilder.configure(
            self._getPose,
            self._resetOdometry,
            self._getRobotRelativeSpeeds,
            self._driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.kPController, 0, 0),
                PIDConstants(AutoConstants.kPThetaController, 0, 0),
            ),
            AutoConstants.config,
            self.shouldFlipPath,
            self.drivetrain
        )

    def registerNamedCommands(self, superstructure):
        """Call after the Superstructure exists and before AutoBuilder.buildAutoChooser()."""
        # Imported here: superstructure imports subsystems, so a top-level import would be circular
        from superstructure import IntakeState, ScoringState

        NamedCommands.registerCommand(
            'DEPLOY_INTAKE', superstructure.autoCreateStateCommand(IntakeState.DEPLOYED))
        NamedCommands.registerCommand(
            'POINT_AND_SHOOT', superstructure.createStateCommand(ScoringState.PREP_SHOT).withTimeout(6.0))
        NamedCommands.registerCommand(
            'PREP_SHOT_DS2', superstructure.createStateCommand(ScoringState.PREP_SHOT).withTimeout(8.0))

    def registerEventTriggers(self, superstructure):
        from superstructure import IntakeState

        EventTrigger('DEPLOY_INTAKE').onTrue(superstructure.autoCreateStateCommand(IntakeState.DEPLOYED))
        EventTrigger('INTAKING').whileTrue(superstructure.createStateCommand(IntakeState.INTAKING))
        EventTrigger('INTAKING_DS2_NEUTRAL').whileTrue(superstructure.createStateCommand(IntakeState.INTAKING))

    def _driveRobotRelative(self, speeds, feedforwards):
        self.drivetrain.driveRobotRelativeChassisSpeeds(
            ChassisSpeeds(speeds.vx, speeds.vy, -speeds.omega),
            feedforwards
        )

    def _getRobotRelativeSpeeds(self):
        return self.drivetrain.getRobotRelativeSpeeds()

    def _resetOdometry(self, pose):
        self.drivetrain.resetOdometry(pose)

    def _getPose(self):
        return self.drivetrain.getPose()

    def shouldFlipPath(self):
        return self.drivetrain.getAlliance() == DriverStation.Alliance.kRed

    def drawAuto(self, autoName: str):
        if not autoName:
            return

        try:
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName)

            poses = [pose for path in paths for pose in path.getPathPoses()]

            if self.shouldFlipPath():
                poses = [FlippingUtil.flipFieldPose(pose) for pose in poses]

            self.drivetrain.field.getObject("Auto Path").setPoses(poses)

        except Exception as e:
            log("Autonomous", f"Failed to draw auto '{autoName}': {e}")

    def clearAutoPreview(self):
        self.drivetrain.field.getObject("Auto Path").setPoses([])