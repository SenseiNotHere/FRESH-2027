from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation, Timer, SendableChooser

from pykit.logger import Logger

from utils import LoggedChooser
from constants import IntakeConstants, RobotConstants, RobotModes
from .intake_io import IntakeIO, IntakeIOReal


class IntakeSubsystem(Subsystem):
    def __init__(
            self,
            deployMotorCANID: int,
            deployMotorInverted: bool,
            intakeMotorCANID: int,
            intakeMotorInverted: bool
    ):
        """
        Intake Subsystem.

        This subsystem controls the robot's intake mechanism using two motors and can be
        instantiated multiple times if needed.

        Hardware:
        - Spark MAX: Controls the deploy position of the intake using built-in limit switches.
        - TalonFX (Kraken X60): Drives the intake roller, providing the high torque required
          to pull game pieces into the robot.

        :param deployMotorCANID: CAN ID of the Spark MAX controlling the intake deploy mechanism.
        :param deployMotorInverted: Whether the deploy motor (Spark MAX) is inverted.
        :param intakeMotorCANID: CAN ID of the TalonFX controlling the intake roller.
        :param intakeMotorInverted: Whether the intake roller motor (TalonFX) is inverted.
        """
        super().__init__()

        # Hardware (replay reads everything back from the log instead)
        self.io = (
            IntakeIO() if RobotConstants.kRobotMode == RobotModes.REPLAY
            else IntakeIOReal(deployMotorCANID, deployMotorInverted, intakeMotorCANID, intakeMotorInverted)
        )
        self.inputs = IntakeIO.IntakeIOInputs()

        # State
        self._homed = False
        self._isDeployed = False

        # Speed Chooser (percent of kIntakeSpeed)
        self.speedChooser = LoggedChooser("Intake Speed")
        self.speedChooser.addOption("5%", 5)
        self.speedChooser.addOption("1%", 1)
        self.speedChooser.addOption("10%", 10)
        self.speedChooser.addOption("15%", 15)
        self.speedChooser.addOption("20%", 20)
        self.speedChooser.addOption("25%", 25)
        self.speedChooser.addOption("30%", 30)
        self.speedChooser.addOption("35%", 35)
        self.speedChooser.addOption("40%", 40)
        self.speedChooser.addOption("45%", 45)
        self.speedChooser.addOption("50%", 50)
        self.speedChooser.addOption("55%", 55)
        self.speedChooser.addOption("60%", 60)
        self.speedChooser.addOption("65%", 65)
        self.speedChooser.setDefaultOption("70%", 70)
        self.speedChooser.addOption("75%", 75)
        self.speedChooser.addOption("80%", 80)
        self.speedChooser.addOption("85%", 85)
        self.speedChooser.addOption("90%", 90)
        self.speedChooser.addOption("95%", 95)
        self.speedChooser.addOption("100%", 100)
        self.velocity = 0

        # Limit Helpers

    def forward_limit_pressed(self):
        return self.inputs.forwardLimit

    def reverse_limit_pressed(self):
        return self.inputs.reverseLimit

    # Limit Switch Sync

    def _sync_encoders(self, target_position: float):
        self.io.resetDeployPosition(target_position)

    def _sync_to_stow(self):
        if not self._homed:
            self._sync_encoders(IntakeConstants.kStowPosition)
        self._homed = True
        self._isDeployed = False

        # Periodic

    def periodic(self):
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)

        SmartDashboard.putBoolean("Intake/Intake Homed", self._homed)
        SmartDashboard.putBoolean("Intake/Intake Deployed", self._isDeployed)
        SmartDashboard.putBoolean("Intake/Forward Limit", self.forward_limit_pressed())
        SmartDashboard.putBoolean("Intake/Reverse Limit", self.reverse_limit_pressed())
        SmartDashboard.putNumber("Intake/Intake Actual Speed", self.inputs.rollerVelocity)
        SmartDashboard.putNumber("Intake/Intake Motor Supply Current", self.inputs.rollerSupplyCurrentAmps)
        SmartDashboard.putNumber("Pivot Motor/Pivot Motor Positon", self.inputs.deployPosition)
        SmartDashboard.putNumber("Pivot Motor/Current", self.inputs.deployCurrentAmps)

        # # Always sync position if a limit switch is hit (homed or not)
        # if self.forward_limit_pressed():
        #     self._isDeployed = True
        #     return
        if self.reverse_limit_pressed():
            self._sync_to_stow()
            return

        if not self._homed:
            self._run_homing()
            return

        # Homing

    def _run_homing(self):
        speed = IntakeConstants.kHomeSpeed
        # if DriverStation.isAutonomous():
        #    speed = -speed
        self.io.setDeploySpeed(-speed)

        # Deploy Control

    def driveDeployMotor(self, speed: float):
        self.io.setDeploySpeed(speed)

    def stopDeployMotor(self):
        self.io.stopDeploy()

    def deploy(self):
        if not self._homed:
            return
        self.io.setDeployPosition(IntakeConstants.kDeployPosition)
        self._isDeployed = True

    def stow(self):
        if not self._homed:
            return
        self.io.setDeployPosition(IntakeConstants.kStowPosition)
        self._isDeployed = False

    def go_to_pulse_position(self):
        if not self._homed:
            return
        self.io.setDeployPosition(IntakeConstants.kPulsePosition)
        self._isDeployed = False

    def toggle_position(self):
        if self._isDeployed:
            self.stow()
        else:
            self.deploy()

    def stop_deploy(self):
        self.io.setDeploySpeed(0)

        # Intake Rollers

    def intake(self):
        self.velocity = self.speedChooser.getSelected() / 100 * IntakeConstants.kIntakeSpeed
        self.io.setRollerVelocity(self.velocity)
        SmartDashboard.putNumber("Intake/Intake Velocity", self.velocity)

    def intake_reverse(self):
        self.io.setRollerVelocity(-IntakeConstants.kIntakeSpeed)

    def stop_intake(self):
        self.io.setRollerVelocity(0)
        SmartDashboard.putNumber("Intake/Intake Velocity", 0)

    def stop(self):
        SmartDashboard.putNumber("Intake/Intake Velocity", 0)
        self.stop_intake()
        self.stop_deploy()

        # State Helpers

    def is_homed(self):
        return self._homed

    def is_deployed(self):
        return self._isDeployed

    def getMotors(self):
        yield from self.io.getMotors()