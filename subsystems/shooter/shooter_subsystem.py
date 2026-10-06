from commands2 import Subsystem

from pykit.logger import Logger
from pykit.networktables.loggednetworknumber import LoggedNetworkNumber

from wpilib import SmartDashboard, SendableChooser

from constants import ShooterConstants, RobotConstants, RobotModes
from .shooter_io import ShooterIO, ShooterIOTalonFX


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

        # Hardware (replay reads everything back from the log instead)
        self.io = ShooterIO() if RobotConstants.kRobotMode == RobotModes.REPLAY else ShooterIOTalonFX(motorCANID, motorInverted)
        self.inputs = ShooterIO.ShooterIOInputs()
        self._logKey = f"Shooter/{motorCANID}"

        # State

        self._targetRPS: float | None = None

        # Dashboard (Manual Testing Only)
        self.percentInput = LoggedNetworkNumber("/SmartDashboard/Shooter/Shooter Percent Input", 25)
        self.kMaxRPM = ShooterConstants.kMaxRPM

    # Periodic

    def periodic(self):
        self.io.updateInputs(self.inputs)
        Logger.processInputs(self._logKey, self.inputs)

        if self._targetRPS is None:
            self.io.stop()
        else:
            self.io.setVelocity(self._targetRPS)

        # Telemetry
        SmartDashboard.putNumber(
            "Shooter/Target RPM",
            (self._targetRPS or 0.0) * 60.0
        )
        SmartDashboard.putNumber(
            "Shooter/Current RPM",
            self.inputs.velocityRPS * 60.0
        )

    # High-Level API

    def setTargetRPS(self, target_rps: float):
        self._targetRPS = max(target_rps, 0.0)

    def setPercent(self, percent: float):
        percent = max(min(percent, 1.0), 0.0)
        target_rpm = percent * self.kMaxRPM
        self._targetRPS = target_rpm / 60.0

    def useDashboardPercent(self):
        percentInput = self.percentInput.value
        percent = percentInput / 100
        self.setPercent(percent)

    def stop(self):
        self._targetRPS = None

    def atSpeed(self, tolerance_rpm: float) -> bool:
        if self._targetRPS is None:
            return False

        target_rpm = self._targetRPS * 60.0
        current_rpm = self.inputs.velocityRPS * 60.0

        return abs(current_rpm - target_rpm) <= tolerance_rpm

    def getTargetRPS(self) -> float:
        return self._targetRPS or 0.0

    def getCurrentRPS(self) -> float:
        return self.inputs.velocityRPS

    def isSpinning(self) -> bool:
        return self._targetRPS is not None

    # Motors
    def getMotors(self):
        yield from self.io.getMotors()