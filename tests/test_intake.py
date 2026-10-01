import pytest

from constants import IntakeConstants
from subsystems import IntakeSubsystem


@pytest.fixture
def intake(can_id):
    return IntakeSubsystem(can_id(), False, can_id(), False)


def home(intake, monkeypatch):
    monkeypatch.setattr(intake, "reverse_limit_pressed", lambda: True)
    intake.periodic()
    monkeypatch.setattr(intake, "reverse_limit_pressed", lambda: False)


def test_homes_by_driving_down(intake, monkeypatch):
    monkeypatch.setattr(intake, "reverse_limit_pressed", lambda: False)
    intake.periodic()
    assert not intake.is_homed()
    assert intake.deployMotor.get() == pytest.approx(-IntakeConstants.kHomeSpeed)


def test_reverse_limit_homes_at_stow(intake, monkeypatch):
    home(intake, monkeypatch)
    assert intake.is_homed()
    assert intake.deployEncoder.getPosition() == pytest.approx(IntakeConstants.kStowPosition)


def test_wont_deploy_before_homing(intake):
    intake.deploy()
    assert not intake.is_deployed()


def test_toggle_after_homing(intake, monkeypatch):
    home(intake, monkeypatch)
    intake.toggle_position()
    assert intake.is_deployed()
    intake.toggle_position()
    assert not intake.is_deployed()


def test_intake_speed_is_percent_of_max(intake):
    intake.intake()
    assert intake.velocity == pytest.approx(0.70 * IntakeConstants.kIntakeSpeed)


def test_reverse_and_stop(intake):
    intake.intake_reverse()
    assert intake.intakeRequest.velocity == pytest.approx(-IntakeConstants.kIntakeSpeed)
    intake.stop_intake()
    assert intake.intakeRequest.velocity == 0
