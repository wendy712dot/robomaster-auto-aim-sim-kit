"""Minimal camera-error adapter for this kit's VisionSendData interface (degrees)."""
import math


def absolute_command(yaw, pitch, yaw_error, pitch_error, max_step=2.0, deadband=0.15):
    """Input: right-positive yaw error, up-positive pitch error; output RM angles.

    This is a small-error, aligned-camera example, not a general TF/ballistics solver.
    """
    if not all(math.isfinite(x) for x in (yaw, pitch, yaw_error, pitch_error, max_step, deadband)):
        raise ValueError('Non-finite angle or control parameter')
    if max_step <= 0 or deadband < 0:
        raise ValueError('max_step must be positive; deadband must be nonnegative')

    def correction(x):
        return 0.0 if abs(x) < deadband else max(-max_step, min(max_step, x))

    return ((yaw - correction(yaw_error) + 180.0) % 360.0 - 180.0,
            pitch - correction(pitch_error))
