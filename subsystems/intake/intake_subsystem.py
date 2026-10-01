from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation, Timer, SendableChooser

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
            .with_k_v(IntakeConstants.kIntakeFF
                      )
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

        # State
        self._homed = False
        self._isDeployed = False

        # Speed Chooser (percent of kIntakeSpeed)
        self.speedChooser = SendableChooser()
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
        SmartDashboard.putData("Intake Speed", self.speedChooser)
        self.velocity = 0

        # Limit Helpers

    def forward_limit_pressed(self):
        return self.forwardLimit.get()

    def reverse_limit_pressed(self):
        return self.reverseLimit.get()

    # Limit Switch Sync

    def _sync_encoders(self, target_position: float):
        self.deployEncoder.setPosition(target_position)

    def _sync_to_stow(self):
        if not self._homed:
            self._sync_encoders(IntakeConstants.kStowPosition)
        self._homed = True
        self._isDeployed = False

        # Periodic

    def periodic(self):
        SmartDashboard.putBoolean("Intake/Intake Homed", self._homed)
        SmartDashboard.putBoolean("Intake/Intake Deployed", self._isDeployed)
        SmartDashboard.putBoolean("Intake/Forward Limit", self.forward_limit_pressed())
        SmartDashboard.putBoolean("Intake/Reverse Limit", self.reverse_limit_pressed())
        SmartDashboard.putNumber("Intake/Intake Actual Speed", self.intakeMotor.get_velocity().value)
        SmartDashboard.putNumber("Intake/Intake Motor Supply Current", self.intakeMotor.get_supply_current().value)
        SmartDashboard.putNumber("Pivot Motor/Pivot Motor Positon", self.deployMotor.getEncoder().getPosition())
        SmartDashboard.putNumber("Pivot Motor/Current", self.deployMotor.getOutputCurrent())

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
        self.deployMotor.set(-speed)

        # Deploy Control

    def driveDeployMotor(self, speed: float):
        self.deployMotor.set(speed)

    def stopDeployMotor(self):
        self.deployMotor.stopMotor()

    def deploy(self):
        if not self._homed:
            return
        self.deployController.setReference(
            IntakeConstants.kDeployPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )
        self._isDeployed = True

    def stow(self):
        if not self._homed:
            return
        self.deployController.setReference(
            IntakeConstants.kStowPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )
        self._isDeployed = False

    def go_to_pulse_position(self):
        if not self._homed:
            return
        self.deployController.setReference(
            IntakeConstants.kPulsePosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )
        self._isDeployed = False

    def toggle_position(self):
        if self._isDeployed:
            self.stow()
        else:
            self.deploy()

    def stop_deploy(self):
        self.deployMotor.set(0)

        # Intake Rollers

    def intake(self):
        self.velocity = self.speedChooser.getSelected() / 100 * IntakeConstants.kIntakeSpeed
        self.intakeMotor.set_control(self.intakeRequest.with_velocity(self.velocity))
        SmartDashboard.putNumber("Intake/Intake Velocity", self.velocity)

    def intake_reverse(self):
        self.intakeMotor.set_control(
            self.intakeRequest.with_velocity(-IntakeConstants.kIntakeSpeed)
        )

    def stop_intake(self):
        self.intakeMotor.set_control(self.intakeRequest.with_velocity(0))
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
        yield self.intakeMotor