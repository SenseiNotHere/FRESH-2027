import pytest
from wpilib.simulation import DriverStationSim, pauseTiming, resumeTiming, stepTiming

from superstructure import Superstructure, MusicState, IntakeState, ScoringState


class FakeShooter:
    at_speed = False

    def atSpeed(self, tolerance_rpm):
        return self.at_speed

    def getCurrentRPS(self):
        return 0.0

    def getTargetRPS(self):
        return 0.0

    def setTargetRPS(self, rps):
        pass

    def useDashboardPercent(self):
        pass

    def stop(self):
        pass


class FakeIndexer:
    feeding = False

    def feed(self):
        self.feeding = True

    def stop(self):
        self.feeding = False


def set_enabled(enabled: bool):
    DriverStationSim.setEnabled(enabled)
    DriverStationSim.notifyNewData()


@pytest.fixture
def superstructure():
    pauseTiming()
    Superstructure._instance = None
    s = Superstructure(shooter=FakeShooter(), indexer=FakeIndexer())
    set_enabled(True)
    yield s
    set_enabled(False)
    Superstructure._instance = None
    resumeTiming()


def test_releasing_shoot_keeps_operator_intaking(superstructure):
    intake = superstructure.createStateCommand(IntakeState.INTAKING)
    shoot = superstructure.createStateCommand(ScoringState.PREP_SHOT)

    intake.initialize()
    shoot.initialize()
    superstructure.setState(ScoringState.SHOOTING)  # as if PREP_SHOT had advanced
    shoot.end(False)

    assert superstructure.states[IntakeState] == IntakeState.INTAKING
    assert superstructure.states[ScoringState] == ScoringState.IDLE


def test_prep_shot_waits_for_speed_then_keeps_shooting(superstructure):
    shooter, indexer = superstructure.shooter, superstructure.indexer
    superstructure.setState(ScoringState.PREP_SHOT)
    superstructure.update()

    def step(seconds, at_speed):
        shooter.at_speed = at_speed
        stepTiming(seconds)
        superstructure.update()

    step(0.02, True)
    assert superstructure.states[ScoringState] == ScoringState.PREP_SHOT  # one at-speed sample isn't enough
    assert not indexer.feeding

    step(0.1, True)
    assert superstructure.states[ScoringState] == ScoringState.SHOOTING
    step(0.02, True)
    assert indexer.feeding

    step(0.5, False)
    assert superstructure.states[ScoringState] == ScoringState.SHOOTING  # no fallback once shooting
    assert indexer.feeding


def test_disable_resets_game_states_but_not_music(superstructure):
    superstructure.setState(IntakeState.DEPLOYED)
    superstructure.setState(ScoringState.SHOOTING)
    superstructure.setState(MusicState.PLAYING_SONG)

    set_enabled(False)
    superstructure.update()

    assert superstructure.states[IntakeState] == IntakeState.IDLE
    assert superstructure.states[ScoringState] == ScoringState.IDLE
    assert superstructure.states[MusicState] == MusicState.PLAYING_SONG
