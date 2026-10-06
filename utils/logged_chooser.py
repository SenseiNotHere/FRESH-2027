from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser


class LoggedChooser(LoggedDashboardChooser):
    """
    Dashboard chooser whose selection is logged, so replay picks the same option the robot did.

    Fixes two pykit 1.0.5 quirks:
    - `options` is a class-level dict shared by every chooser ("50%" on two choosers would collide).
    - Until pykit's first loop, nothing is selected; fall back to the default option instead of None.
    """

    def __init__(self, key: str) -> None:
        self.options = {}
        self._defaultKey = ""
        super().__init__(key)

    def setDefaultOption(self, key: str, value) -> None:
        super().setDefaultOption(key, value)
        # Last default wins, same as SendableChooser
        if self.selectedValue in ("", self._defaultKey):
            self.selectedValue = key
        self._defaultKey = key
