from commands2 import Subsystem
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode
)
from wpilib import SmartDashboard, SendableChooser

from constants import IndexerConstants


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

        # Motor Setup
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

        self.pid = self.motor.getClosedLoopController()

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
        if self._targetRPM is None:
            self.motor.set(0.0)
        else:
            self.pid.setReference(
                self._targetRPM,
                SparkBase.ControlType.kVelocity
            )
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