"""All subsystems in the robot. This contains all high-level coordination of the robot's subsystems."""

from .drive.drive_subsystem import DriveSubsystem
from .drive.autonomous_subsystem import AutonomousSubsystem
from .orchestra.orchestra_subsystem import OrchestraSubsystem

from .intake.intake_subsystem import IntakeSubsystem

from .shooter.shooter_subsystem import ShooterSubsystem
from .shooter.indexer_subsystem import IndexerSubsystem
from .shooter.agitator_subsystem import AgitatorSubsystem
from .shooter.shot_calculator import ShotCalculator
from .vision.limelight_camera import LimelightCamera
from .vision.limelight_localizer import LimelightLocalizer