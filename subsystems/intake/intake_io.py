from dataclasses import dataclass

from pykit.autolog import autolog

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityTorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration, Slot0Configs, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkBaseConfig,
    SparkBase,
    LimitSwitchConfig,
    ClosedLoopSlot,
    PersistMode,
    ResetMode,
    FeedbackSensor
)

from constants import IntakeConstants


class IntakeIO:
    """Intake hardware boundary. This base class does nothing, which is what replay uses."""

    @autolog
    @dataclass
    class IntakeIOInputs:
        deployPosition: float = 0.0
        deployAppliedOutput: float = 0.0
        deployCurrentAmps: float = 0.0
        forwardLimit: bool = False
        reverseLimit: bool = False
        rollerVelocity: float = 0.0
        rollerSupplyCurrentAmps: float = 0.0

    def updateInputs(self, inputs: IntakeIOInputs) -> None:
        pass

    def setDeploySpeed(self, speed: float) -> None:
        pass

    def stopDeploy(self) -> None:
        pass

    def setDeployPosition(self, position: float) -> None:
        pass

    def resetDeployPosition(self, position: float) -> None:
        pass

    def setRollerVelocity(self, velocity: float) -> None:
        pass

    def getMotors(self):
        yield from ()


class IntakeIOReal(IntakeIO):
    """
    - Spark MAX: Controls the deploy position of the intake using built-in limit switches.
    - TalonFX (Kraken X60): Drives the intake roller.
    """

    def __init__(
            self,
            deployMotorCANID: int,
            deployMotorInverted: bool,
            intakeMotorCANID: int,
            intakeMotorInverted: bool
    ):
        # Deploy motor
        self.deployMotor = SparkMax(deployMotorCANID, SparkMax.MotorType.kBrushless)

        deployConfig = SparkMaxConfig()
        deployConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        deployConfig.inverted(deployMotorInverted)

        deployConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        deployConfig.limitSwitch.reverseLimitSwitchEnabled(True)
        deployConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        deployConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)

        deployConfig.closedLoop.pid(
            IntakeConstants.kDeployP,
            IntakeConstants.kDeployI,
            IntakeConstants.kDeployD,
        )
        deployConfig.closedLoop.outputRange(
            IntakeConstants.kDeployMinOutput,
            IntakeConstants.kDeployMaxOutput,
            ClosedLoopSlot.kSlot0
        )
        deployConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)

        self.deployMotor.configure(
            deployConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        self.deployMotor.clearFaults()

        self.deployEncoder = self.deployMotor.getEncoder()
        self.deployController = self.deployMotor.getClosedLoopController()

        self.forwardLimit = self.deployMotor.getForwardLimitSwitch()
        self.reverseLimit = self.deployMotor.getReverseLimitSwitch()

        # Intake Motor (Talon FX)
        self.intakeMotor = TalonFX(intakeMotorCANID)

        intakeConfig = TalonFXConfiguration()
        intakeConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        intakeConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if intakeMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.intakeMotor.configurator.apply(intakeConfig)

        slot0Intake = Slot0Configs()
        (
            slot0Intake
            .with_k_p(IntakeConstants.kIntakeP)
            .with_k_i(IntakeConstants.kIntakeI)
            .with_k_d(IntakeConstants.kIntakeD)
            .with_k_v(IntakeConstants.kIntakeFF)
        )
        self.intakeMotor.configurator.apply(slot0Intake)

        currentConfig = CurrentLimitsConfigs()
        (
            currentConfig
            .with_supply_current_limit(20)
            .with_stator_current_limit(20)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(True)
        )
        self.intakeMotor.configurator.apply(currentConfig)

        self.intakeRequest = VelocityTorqueCurrentFOC(0)

    def updateInputs(self, inputs: IntakeIO.IntakeIOInputs) -> None:
        inputs.deployPosition = float(self.deployEncoder.getPosition())
        inputs.deployAppliedOutput = float(self.deployMotor.getAppliedOutput())
        inputs.deployCurrentAmps = float(self.deployMotor.getOutputCurrent())
        inputs.forwardLimit = bool(self.forwardLimit.get())
        inputs.reverseLimit = bool(self.reverseLimit.get())
        inputs.rollerVelocity = float(self.intakeMotor.get_velocity().value)
        inputs.rollerSupplyCurrentAmps = float(self.intakeMotor.get_supply_current().value)

    def setDeploySpeed(self, speed: float) -> None:
        self.deployMotor.set(speed)

    def stopDeploy(self) -> None:
        self.deployMotor.stopMotor()

    def setDeployPosition(self, position: float) -> None:
        self.deployController.setReference(
            position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )

    def resetDeployPosition(self, position: float) -> None:
        self.deployEncoder.setPosition(position)

    def setRollerVelocity(self, velocity: float) -> None:
        self.intakeMotor.set_control(self.intakeRequest.with_velocity(velocity))

    def getMotors(self):
        yield self.intakeMotor
