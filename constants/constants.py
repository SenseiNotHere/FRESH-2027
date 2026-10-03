from enum import Enum
import math
import os

import wpilib
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.signals import NeutralModeValue
from phoenix6.configs import Slot0Configs

from pathplannerlib.config import RobotConfig

from utils import InterpolatingMap

class SwerveConstants:
    # Robot geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    halfWheelBase = kWheelBase / 2
    halfTrackWidth = kTrackWidth / 2

    kFrontLeftX  = halfWheelBase
    kFrontLeftY  = halfTrackWidth
    kFrontRightX = halfWheelBase
    kFrontRightY = -halfTrackWidth
    kBackLeftX   = -halfWheelBase
    kBackLeftY   = halfTrackWidth
    kBackRightX  = -halfWheelBase
    kBackRightY  = -halfTrackWidth

    kModulePositions = [
        Translation2d(kFrontLeftX,  kFrontLeftY),
        Translation2d(kFrontRightX, kFrontRightY),
        Translation2d(kBackLeftX,   kBackLeftY),
        Translation2d(kBackRightX,  kBackRightY),
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Physical limits
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew rate limiting
    kMagnitudeSlewRate  = 9.8
    kRotationalSlewRate = 12.0

    # CAN IDs: drive motors
    kFrontLeftDriving  = 7
    kFrontRightDriving = 5
    kBackLeftDriving   = 1
    kBackRightDriving  = 3

    # CAN IDs: turning motors
    kFrontLeftTurning  = 8
    kFrontRightTurning = 6
    kBackLeftTurning   = 2
    kBackRightTurning  = 4

    # CAN IDs: absolute encoders
    kFrontLeftTurningEncoder  = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder   = 1
    kBackRightTurningEncoder  = 2

    # CAN IDs: odometry
    kPigeonID = 1

    # Odometry
    kOdometryUpdateFrequency = 250.0  # Hz

    # Lock deadbands
    kLockVxDeadband = 0.05
    kLockVyDeadband = 0.05
    kLockOmegaDeadband = 0.10


class ModuleConstants:
    # Kraken X60 specs
    kMotorFreeSpeedRpm = 5800

    # SDS MK5n L2 Kraken gear ratios
    kDriveGearRatio = 5.36
    kTurningGearRatio = 18.75
    kWheelRadius = 0.0508

    kSlipCurrent = 300.0
    kDrivingMotorFreeSpeedRps = kMotorFreeSpeedRpm / 60.0
    kSpeedAt12Volts = (kDrivingMotorFreeSpeedRps / kDriveGearRatio) * (2 * math.pi * kWheelRadius)

    # Steer/drive coupling (MK5n)
    kSteerDriveCouplingRatio = 3.125

    # Inversions
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute encoder offsets
    kFrontLeftTurningEncoderOffset = 0.264892578125
    kFrontRightTurningEncoderOffset = 0.22265625
    kBackLeftTurningEncoderOffset  = -0.080078125
    kBackRightTurningEncoderOffset = 0.09521484375

    # PID gains
    kDriveGains = (
        Slot0Configs()
        .with_k_p(0.1)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_s(0.1)
    )

    kTurningGains = (
        Slot0Configs()
        .with_k_p(100.0)
        .with_k_i(0.0)
        .with_k_d(0.5)
    )

    # Neutral modes
    kDrivingMotorIdleMode = NeutralModeValue.BRAKE
    kTurningMotorIdleMode = NeutralModeValue.BRAKE

    # Current limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120
    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60


class OIConstants:

    kDriverControllerPort = 0
    kOperatorControllerPort = 1

    kDriveDeadband = 0.05


class AutoConstants:

    config = RobotConfig.fromGUISettings()

    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    kPController = 1.0
    kPThetaController = 0.5

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )

class ShooterConstants:

    kShooterMotorID = 9
    kShooterMotorInverted = True
    kShooterMotor2ID = 10
    kShooterMotor2Inverted = False

    kMinRPM = 600
    kMaxRPM = 4500

    kFF = 0.121
    kP = 0.4
    kI = 0.0001
    kD = 0.0

    kShooterSupplyLimit = 40
    kShooterStatorLimit = 80

    # (Meters, RPS)
    DISTANCE_TO_RPS = InterpolatingMap()
    DISTANCE_TO_RPS.insert(1.0, 48.75) # minimum distance
    DISTANCE_TO_RPS.insert(2.0, 51.75)
    DISTANCE_TO_RPS.insert(2.5, 54.75)
    DISTANCE_TO_RPS.insert(3.0, 56.75)
    DISTANCE_TO_RPS.insert(3.5, 58.0)
    DISTANCE_TO_RPS.insert(4.0, 60.0)
    DISTANCE_TO_RPS.insert(4.5, 63.0)
    DISTANCE_TO_RPS.insert(5.0, 67.5)
    DISTANCE_TO_RPS.insert(5.5, 70.5) # maximum distance

class IntakeConstants:
    # CAN IDs
    kDeployMotorID = 2
    kRollerMotorID = 11

    # Motor Inversions
    kDeployMotorInverted = False
    kRollerMotorInverted = True

    # Deploy (Spark MAX)
    kDeployP = 0.05
    kDeployI = 0.0
    kDeployD = 0.0
    kDeployFF = 0.0

    kDeployMinOutput = -1.0
    kDeployMaxOutput = 1.0

    # Positions (rotations)
    kDeployPosition = 18.4
    kStowPosition = 1.0

    # Homing
    kHomeSpeed = 0.1

    # Anti-jam (position wiggle)
    kPulsePosition = 9.0

    # Rollers (TalonFX)
    
    kIntakeP = 2.6 
    kIntakeI = 0.0
    kIntakeD = 0.0
    kIntakeFF = 0.112

    kIntakeSpeed = 100  # rps at 100% on the chooser
    kIntakePulseSpeed = 30
class IndexerConstants:

    kIndexerMotorID = 1
    kIndexerMotorInverted = True

    kMaxRPS = 40.0
    kFeedRPS = 18.0

    kP = 0.0
    kI = 0.0
    kD = 0.0
    kFF = 0.12

class AgitatorConstants:

    kMotorCANID = 3
    kMotorInverted = True


class RobotModes(Enum):
    REAL = "real"
    SIM = "sim"
    REPLAY = "replay"


class RobotConstants:
    kPDHCanID = 1

    kLogPDH = False

    # Set LOG_PATH to a .wpilog to run sim as a replay of that log.
    kRobotMode: RobotModes = (
        RobotModes.REAL if wpilib.RobotBase.isReal()
        else RobotModes.REPLAY if os.environ.get("LOG_PATH")
        else RobotModes.SIM
    )
