from __future__ import annotations

import typing
import wpilib
from commands2 import InstantCommand, Command
from commands2.button import CommandGenericHID
from wpilib import XboxController, SendableChooser, SmartDashboard
from wpimath.geometry import Rotation2d, Translation3d
from pathplannerlib.auto import AutoBuilder

from pykit.logger import Logger
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution

from commands import HolonomicDrive

from constants import *

from superstructure import Superstructure
from subsystems import (
    DriveSubsystem,
    AutonomousSubsystem,
    OrchestraSubsystem,
    IntakeSubsystem,
    ShooterSubsystem,
    IndexerSubsystem,
    AgitatorSubsystem,
    ShotCalculator,
    LimelightCamera,
    LimelightLocalizer,
)

from button_bindings import ButtonBindings

from utils import log, print_banner


class _RobotPowerDistribution(LoggedPowerDistribution):
    """
    pykit's Logger logs the PDH every loop via LoggedPowerDistribution.getInstance(),
    which otherwise hardcodes a REV module at CAN ID 1 and ignores our config. We seed
    the singleton with this subclass so the logger uses our CAN ID/type, skip the
    hardware entirely in simulation, and log defensively so an unresponsive device or a
    missing per-channel current frame can't stop logging or crash the loop.

    Note: WPILib reports CAN read failures as printed HAL warnings, not Python
    exceptions, so try/except can't silence them. To stop persistent per-channel
    "CAN: Message not Found" spam, set RobotConstants.kLogPDHChannels = False.
    """

    def __init__(self, moduleId: int, moduleType: wpilib.PowerDistribution.ModuleType):
        self._available = wpilib.RobotBase.isReal() and RobotConstants.kEnablePDHLogging
        if self._available:
            super().__init__(moduleId, moduleType)

    def saveToTable(self, table):
        if not self._available:
            return

        try:
            table.put("Voltage", self.distribution.getVoltage())
            table.put("TotalCurrent", self.distribution.getTotalCurrent())
            table.put("TotalPower", self.distribution.getTotalPower())
            table.put("TotalEnergy", self.distribution.getTotalEnergy())
            table.put("Temperature", self.distribution.getTemperature())
        except Exception:
            self._available = False
            return

        if not RobotConstants.kLogPDHChannels:
            return

        channelCurrents = []
        for channel in range(self.distribution.getNumChannels()):
            try:
                channelCurrents.append(self.distribution.getCurrent(channel))
            except Exception:
                channelCurrents.append(0.0)

        table.put("ChannelCurrentsList", channelCurrents)
        table.put("ChannelCurrentsTotal", sum(channelCurrents))


class RobotContainer:
    def __init__(self):
        print_banner("INITIALIZING ROBOT CONTAINER")

        # Controller
        self.driver_controller = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operator_controller = CommandGenericHID(OIConstants.kOperatorControllerPort)

        LoggedPowerDistribution.instance = _RobotPowerDistribution(
            RobotConstants.kPDHCanID, wpilib.PowerDistribution.ModuleType.kRev
        )

        # Subsystems
        def slowdown_when():
            return 0.5 if self.driver_controller.getRawAxis(XboxController.Axis.kLeftTrigger) < 0.5 else 1.0

        self.drive_subsystem = DriveSubsystem(maxSpeedScaleFactor=slowdown_when)
        self.autonomous_subsystem = AutonomousSubsystem(self.drive_subsystem)

        self.intake_subsystem = IntakeSubsystem(
            deployMotorCANID=IntakeConstants.kDeployMotorID,
            deployMotorInverted=IntakeConstants.kDeployMotorInverted,
            intakeMotorCANID=IntakeConstants.kRollerMotorID,
            intakeMotorInverted=IntakeConstants.kRollerMotorInverted,
        )

        self.shooter_subsystem = ShooterSubsystem(
            ShooterConstants.kShooterMotorID,
            ShooterConstants.kShooterMotorInverted,
        )
        self.shooter2_subsystem = ShooterSubsystem(
            ShooterConstants.kShooterMotor2ID,
            ShooterConstants.kShooterMotor2Inverted,
        )

        self.agitator_subsystem = AgitatorSubsystem(
            AgitatorConstants.kMotorCANID,
            AgitatorConstants.kMotorInverted,
        )

        self.indexer_subsystem = IndexerSubsystem(
            IndexerConstants.kIndexerMotorID,
            IndexerConstants.kIndexerMotorInverted,
        )

        self.shot_calculator = ShotCalculator(self.drive_subsystem)

        # Vision
        self.front_limelight = LimelightCamera("limelight-front")
        self.back_limelight = LimelightCamera("limelight-back")

        self.localizer = LimelightLocalizer(drivetrain=self.drive_subsystem, flipIfRed=True)
        self.localizer.addCamera(
            camera=self.front_limelight,
            cameraPoseOnRobot=Translation3d(-0.051, -0.241, 0.533),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )
        self.localizer.addCamera(
            camera=self.back_limelight,
            cameraPoseOnRobot=Translation3d(0.305, 0.025, 0.459),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        self.orchestra = OrchestraSubsystem(
            self.drive_subsystem,
            self.shooter_subsystem,
            self.intake_subsystem,
        )

        # Superstructure - MUST BE LAST TO INITIALIZE
        self.superstructure = Superstructure(
            drivetrain=self.drive_subsystem,
            intake=self.intake_subsystem,
            shooter=self.shooter_subsystem,
            shooter2=self.shooter2_subsystem,
            indexer=self.indexer_subsystem,
            agitator=self.agitator_subsystem,
            shotCalculator=self.shot_calculator,
            vision=self.front_limelight,
            orchestra=self.orchestra,
            driverController=self.driver_controller,
            operatorController=self.operator_controller,
        )

        # Button bindings
        self.button_bindings = ButtonBindings(self, self.superstructure, self.driver_controller, self.operator_controller)
        self.button_bindings.configureButtonBindings()

        self.drive_subsystem.setDefaultCommand(
            HolonomicDrive(
                drivetrain=self.drive_subsystem,
                forwardSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: self.driver_controller.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: self.driver_controller.getRawAxis(XboxController.Axis.kRightX),
                fieldRelative=True,
                rateLimit=True,
                square=True
            )
        )

        # Auto and Test Choosers
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)
        self._lastPreviewedAuto = None
        self.test_chooser = SendableChooser()

        print_banner("ROBOT CONTAINER INITIALIZATION COMPLETE")

    def update(self):
        """Called every loop in robotPeriodic. Logs high-level robot state."""
        Logger.recordOutput("Robot/MatchTime", wpilib.Timer.getMatchTime())
        Logger.recordOutput("Robot/BatteryVoltage", wpilib.RobotController.getBatteryVoltage())
        Logger.recordOutput("Robot/RobotMode", RobotConstants.kRobotMode.value)
        Logger.recordOutput("Robot/Enabled", wpilib.DriverStation.isEnabled())
        Logger.recordOutput("Robot/Autonomous", wpilib.DriverStation.isAutonomous())
        Logger.recordOutput("Robot/Teleop", wpilib.DriverStation.isTeleop())
        Logger.recordOutput("Robot/ActiveAuto", str(self._lastPreviewedAuto))

    def updateAutoPreview(self):
        selected = self.auto_chooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.autonomous_subsystem.drawAuto(selected.getName() if selected is not None else "")
            self._lastPreviewedAuto = selected
    
    # Autonomous and Test Command Getters
    def getAutonomousCommand(self) -> typing.Optional[Command]:
        selected_auto = self.auto_chooser.getSelected()
        if selected_auto is not None:
            log("Robot Container", f"Selected autonomous: {selected_auto.getName()}")
            return selected_auto
        else:
            log("Robot Container", "No autonomous selected")
            return None
        
    def getTestCommand(self) -> typing.Optional[Command]:
        return self.test_chooser.getSelected()