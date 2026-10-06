from commands2 import Subsystem
from pykit.logger import Logger
from wpilib import SmartDashboard, SendableChooser

from constants import IndexerConstants, RobotConstants, RobotModes
from .shooter_io import IndexerIO, IndexerIOSparkMax


class IndexerSubsystem(Subsystem):
    def __init__(self, motorCANID: int, motorInverted: bool):
        """
        Indexer Subsystem.

        This subsystem controls the robot's indexer mechanism using a single motor.
        It can be instantiated multiple times if needed.

        Hardware:
        - Spark MAX controlling a brushless motor used to move game pieces through the indexer.

        :param motorCANID: CAN ID of the Spark MAX controlling the indexer motor.
        :param motorInverted: Whether the indexer motor is inverted.
        """
        super().__init__()

        # Hardware (replay reads everything back from the log instead)
        self.io = IndexerIO() if RobotConstants.kRobotMode == RobotModes.REPLAY else IndexerIOSparkMax(motorCANID, motorInverted)
        self.inputs = IndexerIO.IndexerIOInputs()

        # Internal state
        self._targetRPM: float | None = None
        self._lastCommandedRPM: float | None = None  # track last sent command

        # Optional speed chooser (disabled by default)
        speedChooserEnabled = False

        if speedChooserEnabled:
            self.speedChooser = SendableChooser()
            self.speedChooser.setDefaultOption("100%", 1.0)
            self.speedChooser.addOption("75%", 0.75)
            self.speedChooser.addOption("50%", 0.5)
            self.speedChooser.addOption("25%", 0.25)
            self.speedChooser.addOption("0%", 0.0)

            SmartDashboard.putData("Indexer Speed", self.speedChooser)

    # Periodic

    def periodic(self):
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Indexer", self.inputs)

        if self._targetRPM is None:
            self.io.stop()
        else:
            self.io.setVelocity(self._targetRPM)
        self._lastCommandedRPM = self._targetRPM

        SmartDashboard.putNumber(
            "Indexer/Target RPM",
            self._targetRPM if self._targetRPM else 0.0
        )

    # High-Level API

    def feed(self):
        scale = self.speedChooser.getSelected() if hasattr(self, "speedChooser") else 0.5
        self._targetRPM = IndexerConstants.kFeedRPS * 60.0 * scale

    def reverse(self):
        scale = self.speedChooser.getSelected() if hasattr(self, "speedChooser") else 0.5
        self._targetRPM = -IndexerConstants.kFeedRPS * 60.0 * scale

    def stop(self):
        self._targetRPM = None

    # Optional Helper

    def isRunning(self) -> bool:
        return self._targetRPM is not None