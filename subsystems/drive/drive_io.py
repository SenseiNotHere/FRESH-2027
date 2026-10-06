from dataclasses import dataclass, field

from pykit.autolog import autolog
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState

from phoenix6 import utils
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration
from phoenix6.swerve import (
    SwerveDrivetrain,
    SwerveDrivetrainConstants,
    SwerveModuleConstantsFactory,
    ClosedLoopOutputType
)

from constants import SwerveConstants, ModuleConstants


class DriveIO:
    """Drivetrain hardware boundary. This base class does nothing, which is what replay uses."""

    # ponytail: pose comes from CTRE's own estimator (odometry + vision), so replay plays the
    # logged pose back instead of recomputing it. To replay vision/odometry changes, log
    # modulePositions + rawHeading and run a SwerveDrive4PoseEstimator in DriveSubsystem.
    @autolog
    @dataclass
    class DriveIOInputs:
        pose: Pose2d = field(default_factory=Pose2d)
        speeds: ChassisSpeeds = field(default_factory=ChassisSpeeds)
        moduleStates: list[SwerveModuleState] = field(default_factory=list)
        moduleTargets: list[SwerveModuleState] = field(default_factory=list)
        modulePositions: list[SwerveModulePosition] = field(default_factory=list)
        rawHeading: Rotation2d = field(default_factory=Rotation2d)
        odometryPeriod: float = 0.0
        encoderAbsolutePositions: list[float] = field(default_factory=list)

    def updateInputs(self, inputs: DriveIOInputs) -> None:
        pass

    def setControl(self, request) -> None:
        pass

    def resetPose(self, pose: Pose2d) -> None:
        pass

    def addVisionMeasurement(self, pose: Pose2d, fpgaTimestamp: float, stdDevs: tuple[float, float, float]) -> None:
        pass

    def setOperatorPerspectiveForward(self, direction: Rotation2d) -> None:
        pass

    def getMotors(self):
        yield from ()


class DriveIOCTRE(DriveIO, SwerveDrivetrain[TalonFX, TalonFX, CANcoder]):
    """Real swerve (and Phoenix sim) built on CTRE's SwerveDrivetrain."""

    def __init__(self):
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
            self._configureMotor(
                module.drive_motor,
                ModuleConstants.kDrivingMotorIdleMode,
                ModuleConstants.kDrivingMotorCurrentLimit,
                ModuleConstants.kDrivingMotorStatorCurrentLimit
            )
            self._configureMotor(
                module.steer_motor,
                ModuleConstants.kTurningMotorIdleMode,
                ModuleConstants.kTurningMotorCurrentLimit,
                ModuleConstants.kTurningStatorCurrentLimit
            )

    @staticmethod
    def _configureMotor(motor: TalonFX, neutral_mode, supply_limit: float, stator_limit: float):
        config = TalonFXConfiguration()
        motor.configurator.refresh(config)

        config.motor_output.neutral_mode = neutral_mode
        config.current_limits.supply_current_limit = supply_limit
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.stator_current_limit = stator_limit
        config.current_limits.stator_current_limit_enable = True

        motor.configurator.apply(config)

    def updateInputs(self, inputs: DriveIO.DriveIOInputs) -> None:
        state = self.get_state()
        inputs.pose = state.pose
        inputs.speeds = state.speeds
        inputs.moduleStates = list(state.module_states or [])
        inputs.moduleTargets = list(state.module_targets or [])
        inputs.modulePositions = list(state.module_positions or [])
        inputs.rawHeading = state.raw_heading
        inputs.odometryPeriod = float(state.odometry_period)
        inputs.encoderAbsolutePositions = [
            float(module.encoder.get_absolute_position().value) for module in self.modules
        ]

    def setControl(self, request) -> None:
        self.set_control(request)

    def resetPose(self, pose: Pose2d) -> None:
        self.reset_pose(pose)

    def addVisionMeasurement(self, pose: Pose2d, fpgaTimestamp: float, stdDevs: tuple[float, float, float]) -> None:
        self.add_vision_measurement(pose, utils.fpga_to_current_time(fpgaTimestamp), stdDevs)

    def setOperatorPerspectiveForward(self, direction: Rotation2d) -> None:
        self.set_operator_perspective_forward(direction)

    def getMotors(self):
        for module in self.modules:
            yield module.drive_motor
            yield module.steer_motor
