from enum import Enum

class MusicState(Enum):
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2


class IntakeState(Enum):
    IDLE = 0
    DEPLOYED = 1
    STOWED = 2
    INTAKING = 3
    REVERSE = 4


class ScoringState(Enum):
    IDLE = 0
    PREP_SHOT = 1
    SHOOTING = 2
    PASSING_FUEL = 3
    AGITATOR_OPPOSITE = 4
