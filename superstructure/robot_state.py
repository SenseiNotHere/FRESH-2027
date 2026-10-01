from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    # General
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2

    # Intake
    INTAKE_STOWED = 10
    INTAKE_DEPLOYED = 11
    INTAKING = 12
    INTAKE_REVERSE = 13

    # Shooter
    PREP_SHOT = 20
    SHOOTING = 21

    # Jam handling
    AGITATOR_OPPOSITE = 40

    # Extra gameplay
    PASSING_FUEL = 50

    # Autonomous
    PREP_SHOT_AUTONOMOUS = 100
    INTAKING_AUTONOMOUS = 101
    SHOOTING_AUTONOMOUS = 102


@dataclass
class RobotReadiness:
    shooterReady: bool = False
    intakeDeployed: bool = False
    canFeed: bool = False
