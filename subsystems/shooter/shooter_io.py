from dataclasses import dataclass

from pykit.autolog import autolog

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, NeutralOut
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    CurrentLimitsConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.sim import ChassisReference

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode
)

from constants import ShooterConstants, IndexerConstants

# Base classes do nothing, which is what replay uses.


class ShooterIO:
    @autolog
    @dataclass
    class ShooterIOInputs:
        velocityRPS: float = 0.0
        appliedVolts: float = 0.0
        supplyCurrentAmps: float = 0.0

    def updateInputs(self, inputs: ShooterIOInputs) -> None:
        pass

    def setVelocity(self, rps: float) -> None:
        pass

    def stop(self) -> None:
        pass

    def getMotors(self):
        yield from ()


class ShooterIOTalonFX(ShooterIO):
    def __init__(self, motorCANID: int, motorInverted: bool):
        self.motor = TalonFX(motorCANID)

        motorConfig = TalonFXConfiguration()
        motorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        motorConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        self.motor.configurator.apply(motorConfig)
        # Sim reports velocity in the same direction the real motor is configured for
        self.motor.sim_state.orientation = (
            ChassisReference.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else ChassisReference.CLOCKWISE_POSITIVE
        )

        slot0 = Slot0Configs()
        (
            slot0
            .with_k_p(ShooterConstants.kP)
            .with_k_i(ShooterConstants.kI)
            .with_k_d(ShooterConstants.kD)
            .with_k_v(ShooterConstants.kFF)
        )
        self.motor.configurator.apply(slot0)

        currentLimits = CurrentLimitsConfigs()
        (
            currentLimits
            .with_supply_current_limit(ShooterConstants.kShooterSupplyLimit)
            .with_stator_current_limit(ShooterConstants.kShooterStatorLimit)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(True)
        )
        self.motor.configurator.apply(currentLimits)

        self.velocityRequest = VelocityVoltage(0.0).with_slot(0)
        self.neutralRequest = NeutralOut()

    def updateInputs(self, inputs: ShooterIO.ShooterIOInputs) -> None:
        inputs.velocityRPS = float(self.motor.get_velocity().value)
        inputs.appliedVolts = float(self.motor.get_motor_voltage().value)
        inputs.supplyCurrentAmps = float(self.motor.get_supply_current().value)

    def setVelocity(self, rps: float) -> None:
        self.motor.set_control(self.velocityRequest.with_velocity(rps))

    def stop(self) -> None:
        self.motor.set_control(self.neutralRequest)

    def getMotors(self):
        yield self.motor


class IndexerIO:
    @autolog
    @dataclass
    class IndexerIOInputs:
        velocityRPM: float = 0.0
        appliedOutput: float = 0.0
        currentAmps: float = 0.0

    def updateInputs(self, inputs: IndexerIOInputs) -> None:
        pass

    def setVelocity(self, rpm: float) -> None:
        pass

    def stop(self) -> None:
        pass


class IndexerIOSparkMax(IndexerIO):
    def __init__(self, motorCANID: int, motorInverted: bool):
        self.motor = SparkMax(
            motorCANID,
            SparkLowLevel.MotorType.kBrushless
        )

        config = SparkMaxConfig()
        config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        config.inverted(motorInverted)

        config.closedLoop.P(IndexerConstants.kP)
        config.closedLoop.I(IndexerConstants.kI)
        config.closedLoop.D(IndexerConstants.kD)
        config.closedLoop.velocityFF(IndexerConstants.kFF)
        config.closedLoop.outputRange(-1.0, 1.0)

        self.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.encoder = self.motor.getEncoder()
        self.pid = self.motor.getClosedLoopController()

    def updateInputs(self, inputs: IndexerIO.IndexerIOInputs) -> None:
        inputs.velocityRPM = float(self.encoder.getVelocity())
        inputs.appliedOutput = float(self.motor.getAppliedOutput())
        inputs.currentAmps = float(self.motor.getOutputCurrent())

    def setVelocity(self, rpm: float) -> None:
        self.pid.setReference(rpm, SparkBase.ControlType.kVelocity)

    def stop(self) -> None:
        self.motor.set(0.0)


class AgitatorIO:
    @autolog
    @dataclass
    class AgitatorIOInputs:
        velocityRPM: float = 0.0
        appliedOutput: float = 0.0
        currentAmps: float = 0.0

    def updateInputs(self, inputs: AgitatorIOInputs) -> None:
        pass

    def setSpeed(self, speed: float) -> None:
        pass


class AgitatorIOSparkMax(AgitatorIO):
    def __init__(self, motorCANID: int, motorInverted: bool):
        self.motor = SparkMax(motorCANID, SparkLowLevel.MotorType.kBrushless)

        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        motorConfig.inverted(motorInverted)

        self.motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.encoder = self.motor.getEncoder()

    def updateInputs(self, inputs: AgitatorIO.AgitatorIOInputs) -> None:
        inputs.velocityRPM = float(self.encoder.getVelocity())
        inputs.appliedOutput = float(self.motor.getAppliedOutput())
        inputs.currentAmps = float(self.motor.getOutputCurrent())

    def setSpeed(self, speed: float) -> None:
        self.motor.set(speed)
