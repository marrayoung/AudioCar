"""
Motor feedback tuning script (PID-ish) with encoder RPM measurement and CSV logging.

Purpose:
    Runs two motors at a target RPM using feedback control, while logging time-series
    data to CSV for tuning gains.

Usage:
    python tune_motors.py <goal_rpm> <pk1> <pk2> <ik1> <ik2> <dk1> <dk2>

Args:
    goal_rpm : int    Target RPM for both motors
    pk1      : float  Proportional gain for motor 1
    pk2      : float  Proportional gain for motor 2
    ik1      : float  Integral gain for motor 1
    ik2      : float  Integral gain for motor 2
    dk1      : float  Derivative gain for motor 1
    dk2      : float  Derivative gain for motor 2

Notes:
    - This script assumes optical encoders with TICKS_PER_REV ticks per revolution.
    - Motors are driven forward only in this tuning program.
    - Data is written on Ctrl+C to motorData1.csv and motorData2.csv
"""

import math
import sys
import time
import csv

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


# ----------------------------
# Configuration
# ----------------------------

# Encoder inputs (BCM)
OPTICAL_1 = 25
OPTICAL_2 = 16

# Motor driver pins (BCM)
# Motor 1 forward = IN1, Motor 1 backward = IN2
# Motor 2 forward = IN3, Motor 2 backward = IN4
IN1 = 12
IN2 = 13
IN3 = 19
IN4 = 18

# PWM frequencies used in the original script
PWM_FWD_FREQ_HZ = 2000
PWM_REV_FREQ_HZ = 1000  # not used for tuning (forward-only), but initialized safely

TICKS_PER_REV = 40
MEASUREMENT_INTERVAL_S = 0.15


# ----------------------------
# Global tick counters (updated in callbacks)
# ----------------------------

tickCount1 = 0
tickCount2 = 0


def on_tick_motor1(_channel: int) -> None:
    """Encoder tick callback for motor 1."""
    global tickCount1
    tickCount1 += 1


def on_tick_motor2(_channel: int) -> None:
    """Encoder tick callback for motor 2."""
    global tickCount2
    tickCount2 += 1


# ----------------------------
# Helpers
# ----------------------------

def clamp_duty_cycle(dc: float) -> float:
    """Clamp PWM duty cycle to valid range [0, 100]."""
    if dc > 100.0:
        return 100.0
    if dc < 0.0:
        return 0.0
    return dc


def rpm_from_ticks(ticks: int, dt: float) -> float:
    """Convert encoder ticks over dt seconds into RPM."""
    if dt <= 0.0:
        return 0.0
    revs = ticks / TICKS_PER_REV
    return (revs / dt) * 60.0


def parse_args() -> tuple[int, float, float, float, float, float, float]:
    """Parse CLI args and return gains."""
    if len(sys.argv) < 8:
        raise SystemExit(
            "Usage: python tune_motors.py <goal_rpm> <pk1> <pk2> <ik1> <ik2> <dk1> <dk2>"
        )

    goal_rpm = int(sys.argv[1])
    pk1 = float(sys.argv[2])
    pk2 = float(sys.argv[3])
    ik1 = float(sys.argv[4])
    ik2 = float(sys.argv[5])
    dk1 = float(sys.argv[6])
    dk2 = float(sys.argv[7])

    return goal_rpm, pk1, pk2, ik1, ik2, dk1, dk2


def setup_gpio() -> tuple[GPIO.PWM, GPIO.PWM, GPIO.PWM, GPIO.PWM]:
    """
    Configure GPIO pins and return PWM objects.

    Returns:
        pwm1_fwd, pwm1_rev, pwm2_fwd, pwm2_rev
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Encoder inputs
    GPIO.setup(OPTICAL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(OPTICAL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Motor outputs
    for pin in (IN1, IN2, IN3, IN4):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # PWM setup
    pwm1_fwd = GPIO.PWM(IN1, PWM_FWD_FREQ_HZ)
    pwm1_rev = GPIO.PWM(IN2, PWM_REV_FREQ_HZ)
    pwm2_fwd = GPIO.PWM(IN3, PWM_FWD_FREQ_HZ)
    pwm2_rev = GPIO.PWM(IN4, PWM_REV_FREQ_HZ)

    # Encoder edge detection
    GPIO.add_event_detect(OPTICAL_1, GPIO.BOTH, callback=on_tick_motor1, bouncetime=20)
    GPIO.add_event_detect(OPTICAL_2, GPIO.BOTH, callback=on_tick_motor2, bouncetime=20)

    return pwm1_fwd, pwm1_rev, pwm2_fwd, pwm2_rev


def write_csv(path: str, rows: list[list[float]]) -> None:
    """Write tuning data to CSV with a consistent header."""
    header = ["t", "RPM", "Duty Cycle (%)", "Error", "Prop.", "Integ.", "Deriv."]
    with open(path, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


# ----------------------------
# Main
# ----------------------------

def main() -> None:
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    goal_rpm, pk1, pk2, ik1, ik2, dk1, dk2 = parse_args()

    pwm1_fwd, pwm1_rev, pwm2_fwd, pwm2_rev = setup_gpio()

    # Ensure reverse channels are inactive for forward-only tuning
    pwm1_rev.start(0.0)
    pwm2_rev.start(0.0)

    # Initial duty cycles
    dc1 = 0.0
    dc2 = 0.0

    # Integral and derivative state
    int_err1 = 0.0
    int_err2 = 0.0
    prev_err1 = 0.0
    prev_err2 = 0.0

    # Logging buffers
    motorData1: list[list[float]] = []
    motorData2: list[list[float]] = []

    # Timing state
    start_time = time.time()
    prev_time = start_time

    # Start motors forward
    pwm1_fwd.start(dc1)
    pwm2_fwd.start(dc2)

    try:
        while True:
            now = time.time()
            dt = now - prev_time
            if dt < MEASUREMENT_INTERVAL_S:
                continue

            # Snapshot tick counts and reset quickly to reduce race window
            global tickCount1, tickCount2
            ticks1 = tickCount1
            ticks2 = tickCount2
            tickCount1 = 0
            tickCount2 = 0

            t = now - start_time

            rpm1 = rpm_from_ticks(ticks1, dt)
            rpm2 = rpm_from_ticks(ticks2, dt)

            err1 = goal_rpm - rpm1
            err2 = goal_rpm - rpm2

            # PID components (using continuous-time style with dt)
            prop1 = pk1 * err1
            prop2 = pk2 * err2

            int_err1 += err1 * dt
            int_err2 += err2 * dt
            integ1 = ik1 * int_err1
            integ2 = ik2 * int_err2

            deriv1 = 0.0
            deriv2 = 0.0
            if dt > 0.0:
                deriv1 = dk1 * ((err1 - prev_err1) / dt)
                deriv2 = dk2 * ((err2 - prev_err2) / dt)

            # Log before updating duty cycle so logs reflect the state used this cycle
            motorData1.append([t, rpm1, dc1, err1, prop1, integ1, deriv1])
            motorData2.append([t, rpm2, dc2, err2, prop2, integ2, deriv2])

            print(
                f"t: {t:.3f} | "
                f"DC1: {dc1:.2f}, err1: {err1:.2f} | "
                f"DC2: {dc2:.2f}, err2: {err2:.2f}"
            )

            # Update duty cycles
            dc1 = clamp_duty_cycle(dc1 + prop1 + integ1 + deriv1)
            dc2 = clamp_duty_cycle(dc2 + prop2 + integ2 + deriv2)

            pwm1_fwd.ChangeDutyCycle(dc1)
            pwm2_fwd.ChangeDutyCycle(dc2)

            prev_err1 = err1
            prev_err2 = err2
            prev_time = now

    except KeyboardInterrupt:
        pass
    finally:
        # Stop PWM safely
        try:
            pwm1_fwd.stop()
            pwm2_fwd.stop()
            pwm1_rev.stop()
            pwm2_rev.stop()
        except Exception:
            pass

        # Write logs
        if motorData1:
            write_csv("motorData1.csv", motorData1)
            print("Saved data to motorData1.csv")

        if motorData2:
            write_csv("motorData2.csv", motorData2)
            print("Saved data to motorData2.csv")

        GPIO.cleanup()


if __name__ == "__main__":
    main()
