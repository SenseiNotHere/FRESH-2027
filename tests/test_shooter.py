from types import SimpleNamespace

import pytest
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants import Hub, IndexerConstants, ShooterConstants
from subsystems import AgitatorSubsystem, IndexerSubsystem, ShooterSubsystem, ShotCalculator
from subsystems.shooter import agitator_subsystem
from subsystems.shooter.shot_calculator import draw_arrow


def test_shooter_percent_is_clamped(can_id):
    shooter = ShooterSubsystem(can_id(), False)
    shooter.setPercent(2.0)
    assert shooter._targetRPS == pytest.approx(ShooterConstants.kMaxRPM / 60)
    shooter.setPercent(-1.0)
    assert shooter._targetRPS == 0.0


def test_shooter_never_spins_backwards(can_id):
    shooter = ShooterSubsystem(can_id(), False)
    shooter.setTargetRPS(-10)
    assert shooter._targetRPS == 0.0


def test_shooter_stop(can_id):
    shooter = ShooterSubsystem(can_id(), False)
    shooter.setTargetRPS(50)
    assert shooter.isSpinning()
    shooter.stop()
    assert not shooter.isSpinning()
    assert not shooter.atSpeed(tolerance_rpm=1e9)


def test_shooter_not_at_speed_while_spinning_up(can_id):
    shooter = ShooterSubsystem(can_id(), False)
    shooter.setTargetRPS(50)
    assert not shooter.atSpeed(tolerance_rpm=100)


def test_indexer_feed_reverse_stop(can_id):
    indexer = IndexerSubsystem(can_id(), False)
    feedRPM = IndexerConstants.kFeedRPS * 60 * 0.5

    indexer.feed()
    assert indexer._targetRPM == pytest.approx(feedRPM)
    indexer.reverse()
    assert indexer._targetRPM == pytest.approx(-feedRPM)
    indexer.stop()
    assert not indexer.isRunning()
    indexer.periodic()


class FakeClock:
    now = 0.0

    @classmethod
    def getFPGATimestamp(cls):
        return cls.now


@pytest.fixture
def agitator(can_id, monkeypatch):
    FakeClock.now = 0.0
    monkeypatch.setattr(agitator_subsystem, "Timer", FakeClock)
    return AgitatorSubsystem(can_id(), False)


def test_agitator_feed_reverse_stop(agitator):
    agitator.feed()
    assert agitator.motor.get() == pytest.approx(0.25)
    agitator.reverse()
    assert agitator.motor.get() == pytest.approx(-0.25)
    agitator.stop()
    assert agitator.motor.get() == 0.0


def test_agitator_oscillates(agitator):
    agitator.startOscillate(forwardSeconds=2.0, backwardSeconds=0.5)
    assert agitator.motor.get() > 0

    FakeClock.now = 1.9
    agitator.periodic()
    assert agitator.motor.get() > 0

    FakeClock.now = 2.1
    agitator.periodic()
    assert agitator.motor.get() < 0

    FakeClock.now = 2.7
    agitator.periodic()
    assert agitator.motor.get() > 0


def test_feed_cancels_oscillation(agitator):
    agitator.startOscillate()
    agitator.feed()
    FakeClock.now = 10.0
    agitator.periodic()
    assert agitator.motor.get() > 0


def shot_from(pose: Pose2d) -> ShotCalculator:
    calc = ShotCalculator(SimpleNamespace(getPose=lambda: pose, field=None))
    calc.periodic()
    return calc


def test_shot_distance_and_speed():
    hub = Hub.BLUE_HUB
    calc = shot_from(Pose2d(hub.x - 3.0, hub.y, Rotation2d()))
    assert calc.getTargetDistance() == pytest.approx(3.0)
    assert calc.getTargetSpeedRPS() == pytest.approx(ShooterConstants.DISTANCE_TO_RPS.get(3.0))


@pytest.mark.xfail(strict=True, reason="yaw compares headings, not the direction to the hub")
def test_shot_yaw_points_at_hub():
    hub = Hub.BLUE_HUB
    calc = shot_from(Pose2d(hub.x, hub.y - 2.0, Rotation2d()))  # directly below the hub, facing +x
    assert calc.getEffectiveYaw() == pytest.approx(Rotation2d.fromDegrees(90).radians())


def test_draw_arrow():
    assert draw_arrow(Translation2d(), Translation2d()) == []
    assert len(draw_arrow(Translation2d(), Translation2d(1, 0), nPoints=11)) == 15
