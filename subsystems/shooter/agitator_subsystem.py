from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, Timer

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    ResetMode,
    PersistMode
)

class AgitatorSubsystem(Subsystem):
    def __init__(
            self,
            motorCANID: int,
            motorInverted: bool,
    ):
        """
        Agitator Subsystem.

        This subsystem controls the agitator mechanism using one or two motors.
        Each motor has its own speed chooser on the dashboard.

        Hardware:
        - Spark MAX(s) controlling brushless motor(s) used to drive the agitator.

        :param motorCANID: CAN ID of the Spark MAX controlling the first agitator motor.
        :param motorInverted: Whether the first agitator motor is inverted.
        """
        super().__init__()

        # Motor Setup
        self.motor = SparkMax(motorCANID, SparkLowLevel.MotorType.kBrushless)

        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        motorConfig.inverted(motorInverted)

        self.motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("25%", 0.25)
        self.speedChooser.addOption("5%", 0.05)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("100%", 1.0)
        SmartDashboard.putData("Agitator/Motor 1 Speed Chooser", self.speedChooser)

        # Oscillation logic
        self._oscillateEnabled = False
        self._forwardPeriod = 2.0
        self._backwardPeriod = 0.5
        self._lastToggleTime = 0.0
        self._forward = True

        # Track last commanded speed to avoid re-commanding
        self._lastCommandedSpeed: float | None = None

    def periodic(self):
        if self._oscillateEnabled:
            now = Timer.getFPGATimestamp()
            elapsed = now - self._lastToggleTime
            period = self._forwardPeriod if self._forward else self._backwardPeriod

            if elapsed >= period:
                self._lastToggleTime = now
                self._forward = not self._forward
                self._applyOscillateOutput()  # only command on direction flip

        SmartDashboard.putNumber("Agitator/Motor Speed", self.motor.get())
        SmartDashboard.putBoolean("Agitator/Agitator Running", self.isRunning())
        SmartDashboard.putBoolean("Agitator/Agitator Oscillating", self._oscillateEnabled)
        SmartDashboard.putBoolean("Agitator/Agitator Forward", self._forward)

    def _setSpeed(self, speed: float) -> None:
        """Only sends command to motor if speed has changed."""
        if speed != self._lastCommandedSpeed:
            self.motor.set(speed)
            self._lastCommandedSpeed = speed

    def feed(self) -> None:
        self._oscillateEnabled = False
        self._setSpeed(self.speedChooser.getSelected())

    def reverse(self) -> None:
        self._oscillateEnabled = False
        self._setSpeed(-self.speedChooser.getSelected())

    def stop(self) -> None:
        self._oscillateEnabled = False
        self._forward = True
        self._lastToggleTime = 0.0
        self._setSpeed(0.0)

    def isRunning(self) -> bool:
        return abs(self.motor.get()) > 0.01

    def startOscillate(self, forwardSeconds: float = 2.0, backwardSeconds: float = 0.5) -> None:
        self._forwardPeriod = max(0.05, float(forwardSeconds))
        self._backwardPeriod = max(0.05, float(backwardSeconds))
        if not self._oscillateEnabled:
            self._lastToggleTime = Timer.getFPGATimestamp()
            self._forward = True
        self._oscillateEnabled = True
        self._applyOscillateOutput()  # command immediately on start

    def _applyOscillateOutput(self) -> None:
        speed = self.speedChooser.getSelected()
        self._setSpeed(speed if self._forward else -speed)