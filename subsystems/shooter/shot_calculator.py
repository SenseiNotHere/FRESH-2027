import math
from typing import List

from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Rotation2d

from constants import Hub, getHubPose
from constants import ShooterConstants


class ShotCalculator(Subsystem):
    """
    Shot Calculator Subsystem.

    This subsystem performs all calculations required for distance-based shooting.
    It determines the robot's distance to the target, calculates the required
    shooter wheel speed, and computes the yaw angle needed to aim the robot.

    The subsystem does not directly control any hardware. Instead, it provides
    calculated values that other subsystems (such as the shooter and drivetrain)
    can use for targeting and shooting logic.

    Computed values include:
    - Distance from the robot to the target
    - Required shooter wheel speed (RPS) based on a distance lookup table
    - Effective target pose used for aiming
    - Yaw angle required for the robot to face the target

    Credits to FRC Team 868 - TechHOUNDS for the original shot calculation concepts.

    :param drivetrain: The drivetrain subsystem used to obtain the robot's current pose.
    """

    def __init__(self, drivetrain):
        super().__init__()

        self.drivetrain = drivetrain

        # Computed outputs
        self._target_distance: float = 0.0
        self._target_speed_rps: float = 0.0
        self._effective_target_pose: Pose3d = Hub.CENTER
        self._effective_yaw: float = 0.0

    # Periodic

    def periodic(self):

        drivetrain_pose: Pose2d = self.drivetrain.getPose()
        target_pose = getHubPose()

        # 2D Distance
        DISTANCE_OFFSET = 0.0
        self._target_distance = (
            drivetrain_pose.translation()
            .distance(target_pose.translation())
        ) - DISTANCE_OFFSET

        # Distance -> Speed Lookup
        lookup = ShooterConstants.DISTANCE_TO_RPS
        self._target_speed_rps = lookup.get(self._target_distance)

        # Effective target (future SOTM logic goes here)
        self._effective_target_pose = Pose3d(
            target_pose.x,
            target_pose.y,
            Hub.CENTER.z,
            Hub.CENTER.rotation()
        )

        relative_pose = (
            target_pose
            .relativeTo(drivetrain_pose)
        )

        self._effective_yaw = relative_pose.rotation().radians()
        SmartDashboard.putNumber("ShotCalc/EffectiveYaw", 180 * self._effective_yaw / math.pi)
        SmartDashboard.putNumber("ShotCalc/Distance", self._target_distance)

        if self.drivetrain.field is not None:
            vector_to_goal = target_pose.translation() - drivetrain_pose.translation()
            self.drivetrain.field.getObject("shot-calc-dir").setPoses(draw_arrow(drivetrain_pose.translation(), vector_to_goal))

    # Public API

    def getTargetDistance(self) -> float:
        return self._target_distance

    def getTargetSpeedRPS(self) -> float:
        return self._target_speed_rps

    def getEffectiveTargetPose(self) -> Pose3d:
        return self._effective_target_pose

    def getEffectiveYaw(self) -> float:
        return self._effective_yaw


def draw_arrow(start: Translation2d, directionVector: Translation2d, nPoints=11, size=0.85, tip=0.1) -> List[Pose2d]:
    result = []
    length = directionVector.norm()
    zero = Rotation2d(0)
    if length > 0:
        end = start
        directionVector = directionVector / length
        for i in range(nPoints):
            end = start + directionVector * (size * i / nPoints)
            result.append(Pose2d(end, zero))
        ray1 = directionVector.rotateBy(Rotation2d.fromDegrees(90)) * tip
        ray2 = directionVector * tip
        ray3 = directionVector.rotateBy(Rotation2d.fromDegrees(-90)) * tip
        result.append(Pose2d(end + ray1, zero))
        result.append(Pose2d(end + ray2, zero))
        result.append(Pose2d(end + ray3, zero))
        result.append(Pose2d(end, zero))
    return result