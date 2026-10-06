from wpimath.geometry import Pose2d, Rotation2d

from constants import RobotConstants, RobotModes


class PhysicsEngine:
    def __init__(self, physics_controller, robot):
        self.robot = robot
        self.x = 0.0
        self.y = 0.0
        self.heading = Rotation2d()

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Replay has no hardware to simulate, every input comes from the log
        if RobotConstants.kRobotMode == RobotModes.REPLAY:
            return

        container = self.robot.robot_container

        # No flywheel model: shooters instantly read back the speed they're asked for
        for shooter in (container.shooter_subsystem, container.shooter2_subsystem):
            target = shooter.getTargetRPS() if self.robot.isEnabled() else 0.0
            shooter.io.motor.sim_state.set_rotor_velocity(target)

        if not self.robot.isEnabled():
            return

        drivetrain = container.drive_subsystem
        speeds = drivetrain.last_speeds

        self.heading = self.heading + Rotation2d(speeds.omega * tm_diff)
        self.x += speeds.vx * tm_diff
        self.y += speeds.vy * tm_diff

        drivetrain.resetOdometry(Pose2d(self.x, self.y, self.heading))
