from __future__ import annotations

import math

from commands2 import Subsystem
from wpilib import SmartDashboard, Field2d, DriverStation, RobotBase, Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.filter import SlewRateLimiter

from pykit.logger import Logger

from phoenix6.swerve.requests import ApplyFieldSpeeds, ApplyRobotSpeeds, SwerveDriveBrake

from constants import SwerveConstants, RobotConstants, RobotModes
from .drive_io import DriveIO, DriveIOCTRE


CALIBRATING_DRIVETRAIN = False
class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor):
        Subsystem.__init__(self)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        if self.maxSpeedScaleFactor is not None:
            assert callable(self.maxSpeedScaleFactor)

        # Hardware (replay reads everything back from the log instead)
        self.io = DriveIO() if RobotConstants.kRobotMode == RobotModes.REPLAY else DriveIOCTRE()
        self.inputs = DriveIO.DriveIOInputs()

        # Requests
        self.field_speeds_request = ApplyFieldSpeeds()
        self.robot_speeds_request = ApplyRobotSpeeds()
        self.brake_request = SwerveDriveBrake()

        # Slew Rate Limiters
        self.x_limit = SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate)
        self.y_limit = SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate)
        self.rot_limit = SlewRateLimiter(SwerveConstants.kRotationalSlewRate)

        # Field 2D
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Alliance
        self.alliance = None
        if DriverStation.Alliance is not None:
            self.alliance = DriverStation.getAlliance()

        # Sim stuff
        self.last_speeds = ChassisSpeeds(0, 0, 0)

    # Periodic
    def periodic(self) -> None:
        if self.alliance is None:
            self.getAlliance()

        self.io.updateInputs(self.inputs)
        Logger.processInputs("Drive", self.inputs)

        pose = self.inputs.pose
        self.field.setRobotPose(pose)

        SmartDashboard.putNumber("Drivetrain/X", pose.x)
        SmartDashboard.putNumber("Drivetrain/Y", pose.y)
        SmartDashboard.putNumber("Drivetrain/Heading", pose.rotation().degrees())

        Logger.recordOutput("Drivetrain/Pose", pose)

        for name, position in zip(("FrontLeft", "FrontRight", "BackLeft", "BackRight"), self.inputs.encoderAbsolutePositions):
            SmartDashboard.putNumber(f"Drivetrain/{name}/Position", position)

        # Drivetrain Calibration
        if CALIBRATING_DRIVETRAIN:
            dist_from_origin = math.hypot(pose.x, pose.y) # distance from origin
            SmartDashboard.putNumber("Drivetrain/Calibrating/DistanceFromOrigin", dist_from_origin)
            # Great used for calibrating the gyro

    def resetOdometry(self, pose: Pose2d):
        """
        Resets the odometry of the drivetrain to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        self.io.resetPose(pose)
        
    def add_vision_measurement(self, pose: Pose2d, timestamp: float, stdDevs: tuple[float, float, float]):
        """
        :param timestamp: FPGA time (Timer.getTimestamp()) the measurement was taken.
        """
        self.io.addVisionMeasurement(pose, timestamp, stdDevs)

    def getPose(self) -> Pose2d:
        """
        :return: The current pose of the robot as a Pose2d.
        """
        return self.inputs.pose
    
    def getHeading(self) -> Rotation2d:
        """
        :return: The current heading of the robot as a Rotation2d.
        """
        return self.inputs.pose.rotation()

    def getTurnRate(self) -> float:
        """Degrees per second, counterclockwise positive."""
        return math.degrees(self.inputs.speeds.omega)

    def setX(self):
        """
        Sets the robot into X-Break positon.
        """
        self.io.setControl(self.brake_request)

    def getMotors(self):
        """
        Yields all motors in the drivetrain.
        """
        yield from self.io.getMotors()
            
    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
            square: bool = False
    ):
        if square:
            rot = rot * abs(rot)
            norm = math.hypot(xSpeed, ySpeed)
            xSpeed *= norm
            ySpeed *= norm
            
        scale = self.maxSpeedScaleFactor() if self.maxSpeedScaleFactor is not None else 1.0
        vx = xSpeed * SwerveConstants.kMaxMetersPerSecond * scale
        vy = ySpeed * SwerveConstants.kMaxMetersPerSecond * scale
        omega = rot * SwerveConstants.kMaxAngularSpeed * scale

        if rateLimit:
            vx = self.x_limit.calculate(vx)
            vy = self.y_limit.calculate(vy)
            omega = self.rot_limit.calculate(omega)
            
        speeds = ChassisSpeeds(vx, vy, omega)
        self.last_speeds = speeds
        
        if fieldRelative:
            self.io.setControl(self.field_speeds_request.with_speeds(speeds))
        else:
            self.io.setControl(self.robot_speeds_request.with_speeds(speeds))

    def stop(self):
        self.io.setControl(self.robot_speeds_request.with_speeds(ChassisSpeeds(0, 0, 0)))
        
    # Autonomous support
    def driveRobotRelativeChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards):

        request = self.robot_speeds_request.with_speeds(speeds)

        if feedforwards is not None:
            request = (
                request
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            )

        self.io.setControl(request)
        
    def getRobotRelativeSpeeds(self):
        return self.inputs.speeds

    def getAlliance(self):
        operator_perspective_set = False

        if self.alliance is None:
            self.alliance = DriverStation.getAlliance()
        
        if self.alliance is not None and not operator_perspective_set:
            operator_perspective_set = True
            if self.alliance == DriverStation.Alliance.kRed:
                self.io.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180))
            else:
                self.io.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0))

        return self.alliance