def test_disabled(robot, control):
    with control.run_robot():
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)


def test_autonomous(robot, control):
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=True, enabled=True)


def test_teleop(robot, control):
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)


def test_full_match(robot, control):
    with control.run_robot():
        control.step_timing(seconds=1.0, autonomous=False, enabled=False)
        control.step_timing(seconds=15.0, autonomous=True, enabled=True)
        control.step_timing(seconds=1.0, autonomous=False, enabled=False)
        control.step_timing(seconds=135.0, autonomous=False, enabled=True)
