from types import SimpleNamespace

import pytest
from wpimath.geometry import Rotation2d, Translation3d

from subsystems import LimelightCamera, LimelightLocalizer
from subsystems.vision import limelight_camera, limelight_localizer


class FakeDrivetrain:
    def __init__(self, turnRate=0.0):
        self.turnRate = turnRate
        self.measurements = []

    def getHeading(self):
        return Rotation2d.fromDegrees(30)

    def getTurnRate(self):
        return self.turnRate

    def add_vision_measurement(self, pose, timestamp, stdDevs):
        self.measurements.append((pose, timestamp, stdDevs))


@pytest.fixture
def localizer_with_camera(monkeypatch):
    monkeypatch.setattr(limelight_localizer.utils, "get_current_time_seconds", lambda: 100.0)
    LimelightLocalizer._instance = None

    def make(botpose, turnRate=0.0):
        drivetrain = FakeDrivetrain(turnRate)
        localizer = LimelightLocalizer(drivetrain)
        localizer.enabled = SimpleNamespace(getSelected=lambda: (True, False))

        camera = LimelightCamera("limelight-test")
        localizer.addCamera(camera, Translation3d(0.3, 0.0, 0.2), Rotation2d())
        camera.ticked = True
        camera.botPose.set(botpose)

        localizer.periodic()
        return drivetrain.measurements

    yield make
    LimelightLocalizer._instance = None


# x, y, z, roll, pitch, yaw, latency ms, tag count, span, distance, percent of frame
def botpose(percent=0.7, latency=50.0, count=1):
    return [3.0, 4.0, 0.0, 0.0, 0.0, 0.0, latency, count, 0.0, 2.0, percent]


def test_vision_measurement_is_latency_corrected(localizer_with_camera):
    [(pose, timestamp, stdDevs)] = localizer_with_camera(botpose())
    assert (pose.x, pose.y) == (3.0, 4.0)
    assert timestamp == pytest.approx(99.95)
    assert stdDevs[0] == pytest.approx(limelight_localizer.XY_STD_DEV)
    assert stdDevs[2] == limelight_localizer.HEADING_STD_DEV


def test_bigger_tags_are_trusted_more(localizer_with_camera):
    [(_, _, stdDevs)] = localizer_with_camera(botpose(percent=1.4))
    assert stdDevs[0] == pytest.approx(limelight_localizer.XY_STD_DEV / 2)


@pytest.mark.parametrize("pose, turnRate", [
    (botpose(), 200.0),           # spinning faster than maxRotationSpeed
    (botpose(percent=0.01), 0.0), # tag too small
    (botpose(count=0), 0.0),      # no tags
])
def test_bad_readings_are_skipped(localizer_with_camera, pose, turnRate):
    assert localizer_with_camera(pose, turnRate) == []


def test_usb_camera_feed_points_at_roborio(monkeypatch):
    monkeypatch.setattr(limelight_camera.RobotController, "getTeamNumber", lambda: 1234)
    monkeypatch.setattr(limelight_camera, "PortForwarder", SimpleNamespace(
        getInstance=lambda: SimpleNamespace(add=lambda *args: None)
    ))
    camera = LimelightCamera("limelight-usb", isUsb0=True)
    assert camera.ntSourceValue == "ip:http://10.12.34.2:5800"

