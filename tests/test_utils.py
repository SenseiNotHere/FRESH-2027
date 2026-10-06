import pytest

from utils.interpolating_map import InterpolatingMap


@pytest.fixture
def table():
    m = InterpolatingMap()
    m.insert(1.0, 10.0)
    m.insert(3.0, 30.0)
    return m


def test_interpolates_between_points(table):
    assert table.get(2.0) == pytest.approx(20.0)


def test_clamps_outside_range(table):
    assert table.get(0.0) == 10.0
    assert table.get(99.0) == 30.0


def test_empty_map_returns_zero():
    assert InterpolatingMap().get(5.0) == 0.0


def test_logged_choosers_keep_their_own_options():
    from utils import LoggedChooser
    intake = LoggedChooser("TestIntakeSpeed")
    intake.setDefaultOption("50%", 50)
    agitator = LoggedChooser("TestAgitatorSpeed")
    agitator.setDefaultOption("50%", 0.5)
    agitator.setDefaultOption("25%", 0.25)  # last default wins, like SendableChooser
    assert intake.getSelected() == 50
    assert agitator.getSelected() == 0.25


def test_drive_inputs_survive_a_log_round_trip():
    from pykit.logtable import LogTable
    from wpimath.geometry import Pose2d, Rotation2d
    from wpimath.kinematics import SwerveModuleState
    from subsystems.drive.drive_io import DriveIO

    logged = DriveIO.DriveIOInputs()
    logged.pose = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30))
    logged.moduleStates = [SwerveModuleState(1.5, Rotation2d.fromDegrees(10))] * 4
    logged.encoderAbsolutePositions = [0.1, 0.2, 0.3, 0.4]
    table = LogTable(0)
    logged.toLog(table, "Drive")

    replayed = DriveIO.DriveIOInputs()
    replayed.fromLog(table, "Drive")
    assert replayed.pose == logged.pose
    assert replayed.moduleStates == logged.moduleStates
    assert replayed.encoderAbsolutePositions == logged.encoderAbsolutePositions
