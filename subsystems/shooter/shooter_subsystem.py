from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, NeutralOut
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    CurrentLimitsConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue

from wpilib import SmartDashboard, SendableChooser

from constants import ShooterConstants


class ShooterSubsystem(Subsystem):
    def __init__(self, motorCANID: int, motorInverted: bool):
        """
        Shooter Subsystem.

        This subsystem controls the robot's shooter mechanism using a single Kraken X60
        (TalonFX) motor. The motor is controlled in closed-loop velocity mode to achieve
        a target shooter speed.

        This subsystem can be instantiated multiple times if needed.

        Hardware:
        - TalonFX (Kraken X60) used to spin the shooter wheel.

        :param motorCANID: CAN ID of the TalonFX controlling the shooter motor.
        :param motorInverted: Whether the shooter motor is inverted.
        """
        super().__init__()

        # Motor setup

        self.motor = TalonFX(motorCANID)

        motorConfig = TalonFXConfiguration()
        motorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        motorConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        self.motor.configurator.apply(motorConfig)

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

        # State

        self._targetRPS: float | None = None

        # Dashboard (Manual Testing Only)
        SmartDashboard.putNumber("Shooter/Shooter Percent Input", 25)
        self.kMaxRPM = ShooterConstants.kMaxRPM

    # Periodic

    def periodic(self):

        if self._targetRPS is None:
            self.motor.set_control(self.neutralRequest)
        else:
            self.motor.set_control(
                self.velocityRequest.with_velocity(self._targetRPS)
            )

        # Telemetry
        SmartDashboard.putNumber(
            "Shooter/Target RPM",
            (self._targetRPS or 0.0) * 60.0
        )
        SmartDashboard.putNumber(
            "Shooter/Current RPM",
            self.motor.get_velocity().value * 60.0
        )

    # High-Level API

    def setTargetRPS(self, target_rps: float):
        self._targetRPS = max(target_rps, 0.0)

    def setPercent(self, percent: float):
        percent = max(min(percent, 1.0), 0.0)
        target_rpm = percent * self.kMaxRPM
        self._targetRPS = target_rpm / 60.0

    def useDashboardPercent(self):
        percentInput = SmartDashboard.getNumber("Shooter/Shooter Percent Input", 75)
        percent = percentInput / 100
        self.setPercent(percent)

    def stop(self):
        self._targetRPS = None

    def atSpeed(self, tolerance_rpm: float) -> bool:
        if self._targetRPS is None:
            return False

        target_rpm = self._targetRPS * 60.0
        current_rpm = self.motor.get_velocity().value * 60.0

        return abs(current_rpm - target_rpm) <= tolerance_rpm

    def getTargetRPS(self) -> float:
        return self._targetRPS or 0.0

    def getCurrentRPS(self) -> float:
        return self.motor.get_velocity().value

    def isSpinning(self) -> bool:
        return self._targetRPS is not None

    # Motors
    def getMotors(self):
        yield self.motor