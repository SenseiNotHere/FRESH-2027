from wpilib import DriverStation, Timer, XboxController
from pykit.logger import Logger
from utils import log

# Teleop (seconds since enable) at which each period starts: 10s transition, four 25s shifts, endgame
SHIFT_STARTS = (
    (0, "TRANSITION"),
    (10, "SHIFT 1"),
    (35, "SHIFT 2"),
    (60, "SHIFT 3"),
    (85, "SHIFT 4"),
    (110, "ENDGAME"),
)
WARNING_LEAD = 5.0


class AuxiliaryActions:
    def __init__(self, driverController=None):
        self.shiftNotifier = ShiftNotifier(driverController)

    def update(self):
        self.shiftNotifier.update()


class ShiftNotifier:
    """Rumbles the driver controller WARNING_LEAD seconds before each teleop shift change."""

    def __init__(self, driverController=None):
        self.driverController = driverController
        self.matchStartTime = None
        self._alerted = set()
        self._rumble_end_time = None

    def update(self):
        if DriverStation.isDisabled() and not DriverStation.isFMSAttached():
            self.matchStartTime = None
            self._alerted.clear()
            return

        if not DriverStation.isTeleopEnabled():
            return

        if self.matchStartTime is None:
            self.matchStartTime = Timer.getFPGATimestamp()

        elapsed = Timer.getFPGATimestamp() - self.matchStartTime

        currentShift = SHIFT_STARTS[0][1]
        for start, name in SHIFT_STARTS[1:]:
            if elapsed >= start - WARNING_LEAD and name not in self._alerted:
                self._alerted.add(name)
                self._notify(name)
            if elapsed >= start:
                currentShift = name

        # Logger publishes to NT4, which Elastic reads
        Logger.recordOutput("Match/CurrentShift", currentShift)
        Logger.recordOutput("Match/ElapsedTime", elapsed)

        self._handle_rumble_timeout()

    def _notify(self, text: str):
        log("Aux", text)
        Logger.recordOutput("Match/ShiftAlert", text)
        self._set_rumble(0.6)
        self._rumble_end_time = Timer.getFPGATimestamp() + 0.3

    def _handle_rumble_timeout(self):
        if self._rumble_end_time is not None and Timer.getFPGATimestamp() >= self._rumble_end_time:
            self._set_rumble(0)
            self._rumble_end_time = None

    def _set_rumble(self, value: float):
        if self.driverController is None:
            return
        try:
            self.driverController.getHID().setRumble(XboxController.RumbleType.kBothRumble, value)
        except Exception as e:
            log("Aux", f"Failed to set rumble: {e}")
