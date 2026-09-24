from functools import wraps
from wpilib import DriverStation, Timer, reportWarning
from pykit.logger import Logger


def teleop_only(func):
    """Prevents a function from being called outside of Teleop mode.

    Warnings are rate-limited to one per second so a blocked
    call inside a periodic loop doesn't flood the Driver Station log.
    """
    key = f"Decorators/TeleopOnly/{func.__qualname__}"
    blocked_count = 0
    last_warn_time = float("-inf")

    @wraps(func)
    def wrapper(*args, **kwargs):
        nonlocal blocked_count, last_warn_time
        if not DriverStation.isTeleopEnabled():
            blocked_count += 1
            Logger.recordOutput(f"{key}/BlockedCount", blocked_count)
            now = Timer.getFPGATimestamp()
            if now - last_warn_time >= 1.0:
                last_warn_time = now
                reportWarning(
                    f"Blocked {func.__qualname__} - cannot run outside of Teleop", False
                )
            return None
        return func(*args, **kwargs)
    return wrapper

def throttle(cooldown_seconds):
    """Prevents a function from being called more than once every cooldown_seconds."""
    def decorator(func):
        key = f"Decorators/Throttle/{func.__qualname__}"
        last_called = float("-inf")
        throttled_count = 0

        @wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal last_called, throttled_count
            current_time = Timer.getFPGATimestamp()
            if current_time - last_called < cooldown_seconds:
                throttled_count += 1
                Logger.recordOutput(f"{key}/ThrottledCount", throttled_count)
                return None
            last_called = current_time
            return func(*args, **kwargs)
        return wrapper
    return decorator

def fail_safe(fallback_value=None, warn_interval_seconds=1.0):
    """Catches exceptions so a bad sensor/logic check doesn't kill the robot loop.

    Warnings are rate-limited to one every ``warn_interval_seconds`` so a method
    that throws every loop doesn't flood the Driver Station log. The error count
    and health are still logged to pyKit every call.
    """
    def decorator(func):
        key = f"Decorators/FailSafe/{func.__qualname__}"
        error_count = 0
        last_warn_time = float("-inf")

        @wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal error_count, last_warn_time
            try:
                result = func(*args, **kwargs)
                Logger.recordOutput(f"{key}/Healthy", True)
                return result
            except Exception as e:
                error_count += 1
                Logger.recordOutput(f"{key}/Healthy", False)
                Logger.recordOutput(f"{key}/ErrorCount", error_count)
                Logger.recordOutput(f"{key}/LastError", str(e))

                now = Timer.getFPGATimestamp()
                if now - last_warn_time >= warn_interval_seconds:
                    last_warn_time = now
                    reportWarning(
                        f"[CRITICAL] {func.__qualname__} crashed! Error: {e}", True
                    )
                return fallback_value
        return wrapper
    return decorator

def experimental(func):
    """Flags a method as risky/untested so the drive team knows what's up."""
    key = f"Decorators/Experimental/{func.__qualname__}"
    warned = False

    @wraps(func)
    def wrapper(*args, **kwargs):
        nonlocal warned
        Logger.recordOutput(f"{key}/Active", True)
        if not warned:
            warned = True
            reportWarning(
                f"Running EXPERIMENTAL method '{func.__qualname__}' - use with caution", False
            )
        return func(*args, **kwargs)
    return wrapper
