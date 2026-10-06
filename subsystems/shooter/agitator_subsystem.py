from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, Timer

from pykit.logger import Logger

from utils import LoggedChooser
from constants import RobotConstants, RobotModes
from .shooter_io import AgitatorIO, AgitatorIOSparkMax

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

        # Hardware (replay reads everything back from the log instead)
        self.io = AgitatorIO() if RobotConstants.kRobotMode == RobotModes.REPLAY else AgitatorIOSparkMax(motorCANID, motorInverted)
        self.inputs = AgitatorIO.AgitatorIOInputs()

        self.speedChooser = LoggedChooser("Agitator/Motor 1 Speed Chooser")
        self.speedChooser.setDefaultOption("25%", 0.25)
        self.speedChooser.addOption("5%", 0.05)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("100%", 1.0)

        # Oscillation logic
        self._oscillateEnabled = False
        self._forwardPeriod = 2.0
        self._backwardPeriod = 0.5
        self._lastToggleTime = 0.0
        self._forward = True

        # Track last commanded speed to avoid re-commanding
        self._lastCommandedSpeed: float | None = None

    def periodic(self):
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Agitator", self.inputs)

        if self._oscillateEnabled:
            now = Timer.getTimestamp()
            elapsed = now - self._lastToggleTime
            period = self._forwardPeriod if self._forward else self._backwardPeriod

            if elapsed >= period:
                self._lastToggleTime = now
                self._forward = not self._forward
                self._applyOscillateOutput()  # only command on direction flip

        SmartDashboard.putNumber("Agitator/Motor Speed", self._lastCommandedSpeed or 0.0)
        SmartDashboard.putBoolean("Agitator/Agitator Running", self.isRunning())
        SmartDashboard.putBoolean("Agitator/Agitator Oscillating", self._oscillateEnabled)
        SmartDashboard.putBoolean("Agitator/Agitator Forward", self._forward)

    def _setSpeed(self, speed: float) -> None:
        """Only sends command to motor if speed has changed."""
        if speed != self._lastCommandedSpeed:
            self.io.setSpeed(speed)
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
        return abs(self._lastCommandedSpeed or 0.0) > 0.01

    def startOscillate(self, forwardSeconds: float = 2.0, backwardSeconds: float = 0.5) -> None:
        self._forwardPeriod = max(0.05, float(forwardSeconds))
        self._backwardPeriod = max(0.05, float(backwardSeconds))
        if not self._oscillateEnabled:
            self._lastToggleTime = Timer.getTimestamp()
            self._forward = True
        self._oscillateEnabled = True
        self._applyOscillateOutput()  # command immediately on start

    def _applyOscillateOutput(self) -> None:
        speed = self.speedChooser.getSelected()
        self._setSpeed(speed if self._forward else -speed)