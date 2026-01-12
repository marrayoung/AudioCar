"""
Robot car motion control with optical encoder feedback and dip-switch command selection.

Usage:
    python car_control.py <goal_rpm> <k1> <k2> <i1> <i2>

Args:
    goal_rpm : int    Target RPM for straight-line motion
    k1       : float  Proportional gain for motor 1 (driver side)
    k2       : float  Proportional gain for motor 2 (passenger side)
    i1       : float  Integral gain for motor 1
    i2       : float  Integral gain for motor 2
"""

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

import math
import sys
import time


# ----------------------------
# Constants / Configuration
# ----------------------------

WHEEL_DIAMETER_IN = 2.5
TICKS_PER_REV = 40  # encoder tick count per wheel revolution
PWM_FREQ_HZ = 1000
MEASUREMENT_INTERVAL_S = 0.20

DEBOUNCE_DELAY_S = 0.05
BIT3_CONFIRM_DELAY_S = 3.0

# Optical encoder input pins (BCM numbering)
OPTICAL_1 = 25  # driver side encoder input
OPTICAL_2 = 16  # passenger side encoder input

# Motor control pins (BCM numbering)
MOTOR1_F = 12  # driver forward
MOTOR1_B = 13  # driver backward
MOTOR2_F = 19  # passenger forward
MOTOR2_B = 18  # passenger backward

# Dip switch pins (BCM numbering)
SWITCH_3 = 17  # bit 3
SWITCH_2 = 27  # bit 2
SWITCH_1 = 22  # bit 1
SWITCH_0 = 23  # bit 0


# ----------------------------
# Global state (encoder ticks)
# ----------------------------

tickCount1 = 0
tickCount2 = 0


def _on_tick_motor1(_channel: int) -> None:
    """Encoder tick callback for motor 1 (driver side)."""
    global tickCount1
    tickCount1 += 1


def _on_tick_motor2(_channel: int) -> None:
    """Encoder tick callback for motor 2 (passenger side)."""
    global tickCount2
    tickCount2 += 1


# ----------------------------
# Utility functions
# ----------------------------

def clamp_duty_cycle(dc: float) -> float:
    """Clamp PWM duty cycle to valid range [0, 100]."""
    if dc > 100.0:
        return 100.0
    if dc < 0.0:
        return 0.0
    return dc


def rpm_from_ticks(ticks: int, dt: float) -> float:
    """Convert encoder ticks over a time interval into RPM."""
    if dt <= 0:
        return 0.0
    revs = ticks / TICKS_PER_REV
    return (revs / dt) * 60.0


def runtime_for_distance_inches(distance_in: float, goal_rpm: float) -> float:
    """
    Estimate runtime to travel a given distance at goal_rpm.
    Uses wheel circumference and assumes wheel speed ≈ goal_rpm.
    """
    if goal_rpm <= 0:
        return 0.0
    wheel_circumference = math.pi * WHEEL_DIAMETER_IN
    inches_per_minute = wheel_circumference * goal_rpm
    minutes_needed = distance_in / inches_per_minute
    return minutes_needed * 60.0


def read_cli_args() -> tuple[int, float, float, float, float]:
    """Parse and validate command line arguments."""
    if len(sys.argv) < 6:
        raise SystemExit(
            "Usage: python car_control.py <goal_rpm> <k1> <k2> <i1> <i2>"
        )

    goal_rpm = int(sys.argv[1])
    k1 = float(sys.argv[2])
    k2 = float(sys.argv[3])
    i1 = float(sys.argv[4])
    i2 = float(sys.argv[5])

    return goal_rpm, k1, k2, i1, i2


class DebouncedInput:
    """
    Simple debounced input reader for GPIO digital inputs.

    Call update() repeatedly. It returns the debounced (stable) state.
    """
    def __init__(self, pin: int, delay_s: float):
        self.pin = pin
        self.delay_s = delay_s
        self._stable = GPIO.LOW
        self._last_reading = GPIO.LOW
        self._last_change_time = 0.0

    @property
    def stable(self) -> int:
        return self._stable

    def update(self, now: float) -> int:
        reading = GPIO.input(self.pin)
        if reading != self._last_reading:
            self._last_change_time = now

        if (now - self._last_change_time) > self.delay_s:
            if reading != self._stable:
                self._stable = reading

        self._last_reading = reading
        return self._stable


# ----------------------------
# Main motion controller
# ----------------------------

class RobotCar:
    def __init__(self, goal_rpm: int, k1: float, k2: float, i1: float, i2: float):
        self.goal_rpm = goal_rpm
        self.k1 = k1
        self.k2 = k2
        self.i1 = i1
        self.i2 = i2

        # PWM objects
        self.pwm1_f = GPIO.PWM(MOTOR1_F, PWM_FREQ_HZ)
        self.pwm1_b = GPIO.PWM(MOTOR1_B, PWM_FREQ_HZ)
        self.pwm2_f = GPIO.PWM(MOTOR2_F, PWM_FREQ_HZ)
        self.pwm2_b = GPIO.PWM(MOTOR2_B, PWM_FREQ_HZ)

        # Ensure motors start disabled
        GPIO.output(MOTOR1_F, GPIO.LOW)
        GPIO.output(MOTOR1_B, GPIO.LOW)
        GPIO.output(MOTOR2_F, GPIO.LOW)
        GPIO.output(MOTOR2_B, GPIO.LOW)

    def stop_all(self) -> None:
        """Stop all PWM outputs."""
        for pwm in (self.pwm1_f, self.pwm1_b, self.pwm2_f, self.pwm2_b):
            try:
                pwm.stop()
            except Exception:
                pass

        GPIO.output(MOTOR1_F, GPIO.LOW)
        GPIO.output(MOTOR1_B, GPIO.LOW)
        GPIO.output(MOTOR2_F, GPIO.LOW)
        GPIO.output(MOTOR2_B, GPIO.LOW)

    def left_turn_90(self) -> None:
        """
        Turn approximately 90 degrees left.
        This is an open-loop timed turn using one wheel forward and one wheel backward.
        """
        self.pwm1_f.start(clamp_duty_cycle(self.duty_cycle_motor1(100)))
        self.pwm2_b.start(clamp_duty_cycle(self.duty_cycle_motor2(100)))
        time.sleep(0.50)
        self.stop_all()
        time.sleep(1.0)

    @staticmethod
    def duty_cycle_motor1(rpm: float) -> float:
        """
        Empirical mapping: target RPM -> duty cycle for motor 1.
        """
        dc = 1.20452 * rpm - 37.4778
        return clamp_duty_cycle(dc)

    @staticmethod
    def duty_cycle_motor2(rpm: float) -> float:
        """
        Empirical mapping: target RPM -> duty cycle for motor 2.
        """
        dc = 0.92522 * rpm - 5.63558
        return clamp_duty_cycle(dc)

    def curved_line(self, runtime_s: float, goal_rpm1: float, goal_rpm2: float) -> None:
        """
        Drive a curved line for a fixed time using proportional feedback on each motor.

        runtime_s : total runtime of the motion
        goal_rpm1 : target RPM for motor 1 (driver side)
        goal_rpm2 : target RPM for motor 2 (passenger side)
        """
        global tickCount1, tickCount2

        time.sleep(1.0)

        tickCount1 = 0
        tickCount2 = 0

        dc1 = 0.0
        dc2 = 0.0

        prev = time.time()
        elapsed = 0.0

        # Start both motors forward
        self.pwm1_f.start(dc1)
        self.pwm2_f.start(dc2)

        # Ignore the first measurement window (often noisy)
        ignore_first = True

        try:
            while elapsed < runtime_s:
                now = time.time()
                dt = now - prev
                if dt < MEASUREMENT_INTERVAL_S:
                    continue

                if ignore_first:
                    ignore_first = False
                    tickCount1 = 0
                    tickCount2 = 0
                    prev = now
                    continue

                rpm1 = rpm_from_ticks(tickCount1, dt)
                rpm2 = rpm_from_ticks(tickCount2, dt)

                err1 = goal_rpm1 - rpm1
                err2 = goal_rpm2 - rpm2

                dc1 = clamp_duty_cycle(dc1 + self.k1 * err1)
                dc2 = clamp_duty_cycle(dc2 + self.k2 * err2)

                self.pwm1_f.ChangeDutyCycle(dc1)
                self.pwm2_f.ChangeDutyCycle(dc2)

                elapsed += dt
                prev = now
                tickCount1 = 0
                tickCount2 = 0
        finally:
            self.stop_all()

    def straight_line(self, distance_in: float) -> None:
        """
        Drive straight for a desired distance (in inches) using PI feedback.

        distance_in : distance to travel in inches
        """
        global tickCount1, tickCount2

        time.sleep(1.0)

        tickCount1 = 0
        tickCount2 = 0

        dc1 = 0.0
        dc2 = 0.0

        prev = time.time()
        elapsed = 0.0
        runtime_s = runtime_for_distance_inches(distance_in, self.goal_rpm)

        # Ignore the first measurement window (often noisy)
        ignore_first = True

        # Integral accumulators (time-weighted error)
        int_err1 = 0.0
        int_err2 = 0.0

        self.pwm1_f.start(dc1)
        self.pwm2_f.start(dc2)

        try:
            while elapsed < runtime_s:
                now = time.time()
                dt = now - prev
                if dt < MEASUREMENT_INTERVAL_S:
                    continue

                if ignore_first:
                    ignore_first = False
                    tickCount1 = 0
                    tickCount2 = 0
                    prev = now
                    continue

                rpm1 = rpm_from_ticks(tickCount1, dt)
                rpm2 = rpm_from_ticks(tickCount2, dt)

                err1 = self.goal_rpm - rpm1
                err2 = self.goal_rpm - rpm2

                int_err1 += err1 * dt
                int_err2 += err2 * dt

                dc1 = clamp_duty_cycle(dc1 + self.k1 * err1 + self.i1 * int_err1)
                dc2 = clamp_duty_cycle(dc2 + self.k2 * err2 + self.i2 * int_err2)

                self.pwm1_f.ChangeDutyCycle(dc1)
                self.pwm2_f.ChangeDutyCycle(dc2)

                elapsed += dt
                prev = now
                tickCount1 = 0
                tickCount2 = 0
        finally:
            self.stop_all()

    def figure_eight(self) -> None:
        """
        Execute a figure-eight pattern using timed curved segments and straight segments.
        Stops between movements to allow RPM to settle for improved measurement stability.
        """
        self.curved_line(6.3, 50, 35)
        time.sleep(0.5)
        self.straight_line(36)
        time.sleep(0.5)

        self.curved_line(6.7, 35, 50)
        time.sleep(0.5)
        self.straight_line(36)
        time.sleep(0.5)

        self.curved_line(6.4, 50, 35)
        time.sleep(0.5)
        self.straight_line(36)
        time.sleep(0.5)

        self.curved_line(6.6, 35, 50)
        time.sleep(0.5)
        self.straight_line(36)

    def squares(self) -> None:
        """
        Drive two 3-foot squares (called twice), using straight segments and left turns.
        """
        for _ in range(4):
            time.sleep(1.0)
            self.straight_line(36)
            time.sleep(1.0)
            self.left_turn_90()


# ----------------------------
# GPIO setup / main loop
# ----------------------------

def setup_gpio() -> None:
    """Initialize GPIO pin modes and event detection."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Encoder inputs
    GPIO.setup(OPTICAL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(OPTICAL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Dip switch inputs
    for pin in (SWITCH_3, SWITCH_2, SWITCH_1, SWITCH_0):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Motor outputs
    for pin in (MOTOR1_F, MOTOR1_B, MOTOR2_F, MOTOR2_B):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # Encoder event detection
    GPIO.add_event_detect(OPTICAL_1, GPIO.BOTH, callback=_on_tick_motor1, bouncetime=25)
    GPIO.add_event_detect(OPTICAL_2, GPIO.BOTH, callback=_on_tick_motor2, bouncetime=25)


def main() -> None:
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    goal_rpm, k1, k2, i1, i2 = read_cli_args()

    setup_gpio()

    car = RobotCar(goal_rpm=goal_rpm, k1=k1, k2=k2, i1=i1, i2=i2)

    # Debounced dip switch readers
    sw3 = DebouncedInput(SWITCH_3, DEBOUNCE_DELAY_S)
    sw2 = DebouncedInput(SWITCH_2, DEBOUNCE_DELAY_S)
    sw1 = DebouncedInput(SWITCH_1, DEBOUNCE_DELAY_S)
    sw0 = DebouncedInput(SWITCH_0, DEBOUNCE_DELAY_S)

    bit3_confirmed = False

    # These flags ensure each command triggers once while the dip pattern remains set
    ran_straight = False
    ran_figure8 = False
    ran_square = False

    try:
        while True:
            now = time.time()

            b3 = sw3.update(now)
            b2 = sw2.update(now)
            b1 = sw1.update(now)
            b0 = sw0.update(now)

            # Only accept commands when bit 3 is enabled
            if b3 == GPIO.HIGH:
                if not bit3_confirmed:
                    time.sleep(BIT3_CONFIRM_DELAY_S)
                    bit3_confirmed = True

                # Straight (3 ft)
                if b2 == 1 and b1 == 0 and b0 == 0:
                    if not ran_straight:
                        car.straight_line(36)
                        ran_straight = True
                else:
                    ran_straight = False

                # Figure Eight
                if b2 == 1 and b1 == 0 and b0 == 1:
                    if not ran_figure8:
                        car.figure_eight()
                        ran_figure8 = True
                else:
                    ran_figure8 = False

                # Square (two squares)
                if b2 == 1 and b1 == 1 and b0 == 0:
                    if not ran_square:
                        car.squares()
                        car.squares()
                        ran_square = True
                else:
                    ran_square = False

            else:
                # Reset all command flags when bit 3 is disabled
                bit3_confirmed = False
                ran_straight = False
                ran_figure8 = False
                ran_square = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.stop_all()
        finally:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
