from __future__ import annotations

import math

from commands2 import Subsystem
from wpilib import SmartDashboard, Field2d, DriverStation, RobotBase, Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.filter import SlewRateLimiter

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import MotorOutputConfigs, CurrentLimitsConfigs

from phoenix6.swerve import (
    SwerveDrivetrain,
    SwerveDrivetrainConstants,
    SwerveModuleConstantsFactory,
    ClosedLoopOutputType
)

from phoenix6.swerve.requests import ApplyFieldSpeeds, ApplyRobotSpeeds, SwerveDriveBrake

from constants import SwerveConstants, ModuleConstants


CALIBRATING_DRIVETRAIN = False
class DriveSubsystem(Subsystem, SwerveDrivetrain):
    def __init__(self, maxSpeedScaleFactor):
        Subsystem.__init__(self)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        if self.maxSpeedScaleFactor is not None:
            assert callable(self.maxSpeedScaleFactor)

        # Module factory
        module_factory = (
            SwerveModuleConstantsFactory()
            .with_drive_motor_gear_ratio(ModuleConstants.kDriveGearRatio)
            .with_steer_motor_gear_ratio(ModuleConstants.kTurningGearRatio)
            .with_wheel_radius(ModuleConstants.kWheelRadius)
            .with_slip_current(ModuleConstants.kSlipCurrent)
            .with_drive_motor_gains(ModuleConstants.kDriveGains)
            .with_steer_motor_gains(ModuleConstants.kTurningGains)
            .with_drive_motor_closed_loop_output(ClosedLoopOutputType.TORQUE_CURRENT_FOC)
            .with_speed_at12_volts(ModuleConstants.kSpeedAt12Volts)
            .with_coupling_gear_ratio(ModuleConstants.kSteerDriveCouplingRatio)
        )

        front_left = module_factory.create_module_constants(
            steer_motor_id=SwerveConstants.kFrontLeftTurning,
            drive_motor_id=SwerveConstants.kFrontLeftDriving,
            encoder_id=SwerveConstants.kFrontLeftTurningEncoder,
            encoder_offset=ModuleConstants.kFrontLeftTurningEncoderOffset,
            location_x=SwerveConstants.kFrontLeftX,
            location_y=SwerveConstants.kFrontLeftY,
            drive_motor_inverted=ModuleConstants.kFrontLeftDriveMotorInverted,
            steer_motor_inverted=ModuleConstants.kTurningMotorInverted,
            encoder_inverted=ModuleConstants.kTurningEncoderInverted
        )

        front_right = module_factory.create_module_constants(
            steer_motor_id=SwerveConstants.kFrontRightTurning,
            drive_motor_id=SwerveConstants.kFrontRightDriving,
            encoder_id=SwerveConstants.kFrontRightTurningEncoder,
            encoder_offset=ModuleConstants.kFrontRightTurningEncoderOffset,
            location_x=SwerveConstants.kFrontRightX,
            location_y=SwerveConstants.kFrontRightY,
            drive_motor_inverted=ModuleConstants.kFrontRightDriveMotorInverted,
            steer_motor_inverted=ModuleConstants.kTurningMotorInverted,
            encoder_inverted=ModuleConstants.kTurningEncoderInverted
        )

        back_left = module_factory.create_module_constants(
            steer_motor_id=SwerveConstants.kBackLeftTurning,
            drive_motor_id=SwerveConstants.kBackLeftDriving,
            encoder_id=SwerveConstants.kBackLeftTurningEncoder,
            encoder_offset=ModuleConstants.kBackLeftTurningEncoderOffset,
            location_x=SwerveConstants.kBackLeftX,
            location_y=SwerveConstants.kBackLeftY,
            drive_motor_inverted=ModuleConstants.kBackLeftDriveMotorInverted,
            steer_motor_inverted=ModuleConstants.kTurningMotorInverted,
            encoder_inverted=ModuleConstants.kTurningEncoderInverted
        )

        back_right = module_factory.create_module_constants(
            steer_motor_id=SwerveConstants.kBackRightTurning,
            drive_motor_id=SwerveConstants.kBackRightDriving,
            encoder_id=SwerveConstants.kBackRightTurningEncoder,
            encoder_offset=ModuleConstants.kBackRightTurningEncoderOffset,
            location_x=SwerveConstants.kBackRightX,
            location_y=SwerveConstants.kBackRightY,
            drive_motor_inverted=ModuleConstants.kBackRightDriveMotorInverted,
            steer_motor_inverted=ModuleConstants.kTurningMotorInverted,
            encoder_inverted=ModuleConstants.kTurningEncoderInverted
        )

        drivetrain_constants = (
            SwerveDrivetrainConstants()
            .with_can_bus_name("rio")
            .with_pigeon2_id(SwerveConstants.kPigeonID)
        )

        # Drivetrain Builder
        SwerveDrivetrain.__init__(
            self,
            TalonFX, # Drive motor type
            TalonFX, # Steer motor type
            CANcoder, # Encoder type
            drivetrain_constants, # Drivetrain constants
            SwerveConstants.kOdometryUpdateFrequency, # Odometry update frequency in Hz
            [
                front_left,
                front_right,
                back_left,
                back_right
            ] # Module constants list
        )

        for module in self.modules:
            drive_output_config = MotorOutputConfigs()
            module.drive_motor.configurator.refresh(drive_output_config)
            drive_output_config.neutral_mode = ModuleConstants.kDrivingMotorIdleMode
            module.drive_motor.configurator.apply(drive_output_config)

            steer_output_config = MotorOutputConfigs()
            module.steer_motor.configurator.refresh(steer_output_config)
            steer_output_config.neutral_mode = ModuleConstants.kTurningMotorIdleMode
            module.steer_motor.configurator.apply(steer_output_config)

            drive_current_config = CurrentLimitsConfigs()
            module.drive_motor.configurator.refresh(drive_current_config)
            drive_current_config.supply_current_limit = ModuleConstants.kDrivingMotorCurrentLimit
            drive_current_config.supply_current_limit_enable = True
            drive_current_config.stator_current_limit = ModuleConstants.kDrivingMotorStatorCurrentLimit
            drive_current_config.stator_current_limit_enable = True
            module.drive_motor.configurator.apply(drive_current_config)

            steer_current_config = CurrentLimitsConfigs()
            module.steer_motor.configurator.refresh(steer_current_config)
            steer_current_config.supply_current_limit = ModuleConstants.kTurningMotorCurrentLimit
            steer_current_config.supply_current_limit_enable = True
            steer_current_config.stator_current_limit = ModuleConstants.kTurningStatorCurrentLimit
            steer_current_config.stator_current_limit_enable = True
            module.steer_motor.configurator.apply(steer_current_config)

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

        pose = self.get_state().pose
        self.field.setRobotPose(pose)
        
        SmartDashboard.putNumber("Drivetrain/X", pose.x)
        SmartDashboard.putNumber("Drivetrain/Y", pose.y)
        SmartDashboard.putNumber("Drivetrain/Heading", pose.rotation().degrees())

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
        self.reset_pose(pose)
        
    def getPose(self) -> Pose2d:
        """
        :return: The current pose of the robot as a Pose2d.
        """
        return self.get_state().pose
    
    def getHeading(self) -> Rotation2d:
        """
        :return: The current heading of the robot as a Rotation2d.
        """
        return self.get_state().pose.rotation()

    def setX(self):
        """
        Sets the robot into X-Break positon.
        """
        self.set_control(self.brake_request)

    def getMotors(self):
        """
        Yields all motors in the drivetrain.
        """
        for module in self.modules:
            yield module.drive_motor
            yield module.steer_motor
            
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
            self.set_control(self.field_speeds_request.with_speeds(speeds))
        else:
            self.set_control(self.robot_speeds_request.with_speeds(speeds))

    def stop(self):
        self.set_control(self.robot_speeds_request.with_speeds(ChassisSpeeds(0, 0, 0)))
        
    # Autonomous support
    def driveRobotRelativeChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards):

        request = self.robot_speeds_request.with_speeds(speeds)

        if feedforwards is not None:
            request = (
                request
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            )

        self.set_control(request)
        
    def getRobotRelativeSpeeds(self):
        return self.get_state().speeds

    def getAlliance(self):
        operator_perspective_set = False

        if self.alliance is None:
            self.alliance = DriverStation.getAlliance()
        
        if self.alliance is not None and not operator_perspective_set:
            operator_perspective_set = True
            if self.alliance == DriverStation.Alliance.kRed:
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
            else:
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))

        return self.alliance