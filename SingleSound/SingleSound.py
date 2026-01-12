"""
Robot car control with:
  - Dual motor RPM feedback using optical encoders
  - Dip-switch mode selection (straight / figure eight / square / sound seek / debug)
  - Bump switch safety stop
  - Microphone amplitude read via external ADC module (adc_reader.read_adc)

Notes:
  - This is a single-file "integration" script. For long-term maintainability,
    consider splitting into modules: motor_control.py, inputs.py, behaviors.py, etc.
  - Gains/goal RPM are currently hard-coded below, but the CLI parsing function
    is included and can be re-enabled easily.
"""

import math
import time

try:
    import RPi.GPIO as GPIO
    from adc_reader import read_adc
except ImportError:
    GPIO = None
    read_adc = None


# ----------------------------
# Configuration
# ----------------------------

# Encoder inputs (BCM)
OPTICAL_1 = 25  # driver motor encoder
OPTICAL_2 = 16  # passenger motor encoder

# Motor driver pins (BCM)
MOTOR1_F = 12
MOTOR1_B = 13
MOTOR2_F = 19
MOTOR2_B = 18

# Dip switches (BCM)
SWITCH_3 = 17  # enable bit (bit 3)
SWITCH_2 = 27
SWITCH_1 = 22
SWITCH_0 = 23

# Bump switch (BCM)
BUMPSWITCH = 4

# Encoder properties
TICKS_PER_REV = 40

# Wheel size (inches)
WHEEL_DIAMETER_IN = 2.5

# Switch debounce and confirmation delay
DEBOUNCE_DELAY_S = 0.05
BIT3_CONFIRM_DELAY_S = 3.0

# Feedback loop timing
MEASUREMENT_INTERVAL_S = 0.20

# Motion timing
TURN_90_LEFT_S = 0.52


# ----------------------------
# Gains / Control targets
# ----------------------------

# Re-enable CLI parsing if desired (left in for convenience)
# cmdArgGoalRpm = int(sys.argv[1])
# cmdArgk1 = float(sys.argv[2])
# cmdArgk2 = float(sys.argv[3])
# cmdArgi1 = float(sys.argv[4])
# cmdArgi2 = float(sys.argv[5])

cmdArgGoalRpm = 45
cmdArgk1 = 0.5
cmdArgk2 = 0.5
cmdArgi1 = 0.3
cmdArgi2 = 0.3


# ----------------------------
# Global state (encoder ticks, safety)
# ----------------------------

tickCount1 = 0
tickCount2 = 0
tickAccumulator1 = 0
tickAccumulator2 = 0

isBumped = False


# ----------------------------
# GPIO helpers
# ----------------------------

class DebouncedInput:
    """
    Debounced GPIO input reader.

    Call update(now) repeatedly; read stable state using .stable.
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


def clamp_duty_cycle(dc: float) -> float:
    """Clamp duty cycle to [0, 100]."""
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


def calculateDutyCycle1(rpm: float) -> float:
    """Empirical mapping RPM -> duty cycle for motor 1."""
    dc = 1.20452 * rpm - 37.4778
    return clamp_duty_cycle(dc)


def calculateDutyCycle2(rpm: float) -> float:
    """Empirical mapping RPM -> duty cycle for motor 2."""
    dc = 0.92522 * rpm - 5.63558
    return clamp_duty_cycle(dc)


# ----------------------------
# Encoder callbacks
# ----------------------------

def counting1(_channel: int) -> None:
    """Encoder tick callback for motor 1."""
    global tickCount1, tickAccumulator1
    tickCount1 += 1
    tickAccumulator1 += 1


def counting2(_channel: int) -> None:
    """Encoder tick callback for motor 2."""
    global tickCount2, tickAccumulator2
    tickCount2 += 1
    tickAccumulator2 += 1


# ----------------------------
# Setup
# ----------------------------

def setup_gpio() -> None:
    """Configure GPIO pins and event callbacks."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Encoders
    GPIO.setup(OPTICAL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(OPTICAL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Dip switches
    for pin in (SWITCH_3, SWITCH_2, SWITCH_1, SWITCH_0):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Bump switch
    GPIO.setup(BUMPSWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Motor outputs
    for pin in (MOTOR1_F, MOTOR1_B, MOTOR2_F, MOTOR2_B):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # Encoder events
    GPIO.add_event_detect(OPTICAL_1, GPIO.BOTH, callback=counting1, bouncetime=25)
    GPIO.add_event_detect(OPTICAL_2, GPIO.BOTH, callback=counting2, bouncetime=25)


# ----------------------------
# Motor control wrapper
# ----------------------------

class MotorOutputs:
    """Holds PWM objects for forward/back on both motors and provides safe stop()."""
    def __init__(self):
        self.pwm1_f = GPIO.PWM(MOTOR1_F, 1000)
        self.pwm1_b = GPIO.PWM(MOTOR1_B, 1000)
        self.pwm2_f = GPIO.PWM(MOTOR2_F, 1000)
        self.pwm2_b = GPIO.PWM(MOTOR2_B, 1000)

        # Ensure outputs start inactive
        GPIO.output(MOTOR1_F, GPIO.LOW)
        GPIO.output(MOTOR1_B, GPIO.LOW)
        GPIO.output(MOTOR2_F, GPIO.LOW)
        GPIO.output(MOTOR2_B, GPIO.LOW)

    def stop(self) -> None:
        """Stop all PWM outputs and drive pins low."""
        for pwm in (self.pwm1_f, self.pwm1_b, self.pwm2_f, self.pwm2_b):
            try:
                pwm.stop()
            except Exception:
                pass

        GPIO.output(MOTOR1_F, GPIO.LOW)
        GPIO.output(MOTOR1_B, GPIO.LOW)
        GPIO.output(MOTOR2_F, GPIO.LOW)
        GPIO.output(MOTOR2_B, GPIO.LOW)


# ----------------------------
# Safety (bump switch)
# ----------------------------

def make_bump_callback(motors: MotorOutputs):
    """Factory to create a callback that can stop PWM safely."""
    def _bumped(_channel: int) -> None:
        global isBumped
        isBumped = True
        motors.stop()
        print("Bumped!")
    return _bumped


# ----------------------------
# Behaviors
# ----------------------------

def left_turn_90(motors: MotorOutputs) -> None:
    """Open-loop timed 90-degree left turn."""
    motors.pwm1_f.start(calculateDutyCycle1(100))
    motors.pwm2_b.start(calculateDutyCycle2(100))
    time.sleep(TURN_90_LEFT_S)
    motors.stop()
    time.sleep(1.0)


def increment_left(motors: MotorOutputs, amount_s: float) -> None:
    """Small left rotation for sound seeking."""
    motors.pwm1_f.start(calculateDutyCycle1(100))
    motors.pwm2_b.start(calculateDutyCycle2(100))
    time.sleep(amount_s)
    motors.stop()
    time.sleep(0.5)


def increment_right(motors: MotorOutputs, amount_s: float) -> None:
    """Small right rotation for sound seeking."""
    motors.pwm2_f.start(calculateDutyCycle1(100))
    motors.pwm1_b.start(calculateDutyCycle2(100))
    time.sleep(amount_s)
    motors.stop()
    time.sleep(0.5)


def line(motors: MotorOutputs, runT: float, goalRpm1: float, goalRpm2: float) -> None:
    """
    Drive forward for runT seconds while applying PI control to match motor RPM targets.
    """
    global tickCount1, tickCount2

    time.sleep(1.0)

    tickCount1 = 0
    tickCount2 = 0

    # Gains and integrator state
    k1 = cmdArgk1
    k2 = cmdArgk2
    ik1 = cmdArgi1
    ik2 = cmdArgi2

    dc1 = 0.0
    dc2 = 0.0

    prev = time.time()
    elapsed = 0.0

    # Ignore the first measurement window (often noisy)
    ignore_first = True

    int_err1 = 0.0
    int_err2 = 0.0

    motors.pwm1_f.start(dc1)
    motors.pwm2_f.start(dc2)

    try:
        while elapsed < runT:
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

            err1 = goalRpm1 - rpm1
            err2 = goalRpm2 - rpm2

            int_err1 += err1 * dt
            int_err2 += err2 * dt

            dc1 = clamp_duty_cycle(dc1 + k1 * err1 + ik1 * int_err1)
            dc2 = clamp_duty_cycle(dc2 + k2 * err2 + ik2 * int_err2)

            motors.pwm1_f.ChangeDutyCycle(dc1)
            motors.pwm2_f.ChangeDutyCycle(dc2)

            elapsed += dt
            prev = now
            tickCount1 = 0
            tickCount2 = 0

    finally:
        motors.stop()


def straight_line(motors: MotorOutputs, left_rpm: float, right_rpm: float) -> None:
    """
    Drive forward using line() for a fixed runtime, and print encoder totals.
    """
    global tickAccumulator1, tickAccumulator2

    runT = 3.25
    line(motors, runT, left_rpm, right_rpm)

    print("Count 1:", tickAccumulator1)
    print("Count 2:", tickAccumulator2)

    tickAccumulator1 = 0
    tickAccumulator2 = 0


def figure_eight(motors: MotorOutputs) -> None:
    """Perform figure-eight using timed arcs and straight segments."""
    line(motors, 5.2, 50, 30)
    time.sleep(0.5)
    straight_line(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 30, 50)
    time.sleep(0.5)
    straight_line(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 50, 30)
    time.sleep(0.5)
    straight_line(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 30, 50)
    time.sleep(0.5)
    straight_line(motors, 45, 45)


def squares(motors: MotorOutputs) -> None:
    """Drive two squares using straight segments and timed left turns."""
    print("calling squares()")

    for _ in range(2):  # two squares
        for _ in range(4):
            time.sleep(1.0)
            straight_line(motors, cmdArgGoalRpm, cmdArgGoalRpm)
            time.sleep(1.0)
            left_turn_90(motors)


def single_sound(motors: MotorOutputs) -> None:
    """
    Rotate in increments to find a direction with higher mic amplitude, then drive forward.
    Uses read_adc() from adc_reader.
    """
    global isBumped

    if read_adc is None:
        print("read_adc not available (missing adc_reader).")
        return

    value = 0.0
    prev_value = 0.0

    count_3v = 0
    threshold = 3.0
    turns = 0

    # Rotate until we detect strong signal or a clear peak.
    while value < 3.3:
        increment_left(motors, 0.25)

        turns += 1
        if turns == 10:
            threshold = 2.3

        value = read_adc()
        print(f"{value} V from ADC")

        # Peak detection case (source farther away)
        if (prev_value > value) and (prev_value > threshold):
            while value < 3.3:
                increment_right(motors, 0.1)
                prev_value = value
                value = read_adc()
                print(f"{value} V from ADC")

                if prev_value > value:
                    increment_left(motors, 0.5)
                    break
            break

        # Saturation case (source close). Re-center within saturated region.
        if value == 3.3:
            while value == 3.3:
                increment_left(motors, 0.08)
                value = read_adc()
                print(f"{value} V from ADC")
                count_3v += 1

            for _ in range(count_3v // 2):
                increment_right(motors, 0.07)
                time.sleep(0.1)
            break

        prev_value = value

    # Drive forward until bumped; if signal drops, re-acquire direction.
    while not isBumped:
        time.sleep(1.0)
        line(motors, 5.5, cmdArgGoalRpm, cmdArgGoalRpm)

        if read_adc() < 3.3:
            single_sound(motors)
            return
        else:
            line(motors, 5.0, cmdArgGoalRpm, cmdArgGoalRpm)


def debug_adc_loop() -> None:
    """Print ADC readings once per second until bumped."""
    if read_adc is None:
        print("read_adc not available (missing adc_reader).")
        return

    while not isBumped:
        print(read_adc())
        time.sleep(1.0)


# ----------------------------
# Main loop
# ----------------------------

def main() -> None:
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    setup_gpio()
    motors = MotorOutputs()

    # Register bump callback after motors exist, so we can stop them in the callback
    GPIO.add_event_detect(BUMPSWITCH, GPIO.RISING, callback=make_bump_callback(motors), bouncetime=25)

    # Debounced dip switch readers
    sw3 = DebouncedInput(SWITCH_3, DEBOUNCE_DELAY_S)
    sw2 = DebouncedInput(SWITCH_2, DEBOUNCE_DELAY_S)
    sw1 = DebouncedInput(SWITCH_1, DEBOUNCE_DELAY_S)
    sw0 = DebouncedInput(SWITCH_0, DEBOUNCE_DELAY_S)

    bit3_confirmed = False

    ran_straight = False
    ran_figure8 = False
    ran_square = False
    ran_single = False
    ran_debug = False

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

                # Straight: 100
                if b2 == 1 and b1 == 0 and b0 == 0:
                    if not ran_straight:
                        straight_line(motors, 45, 45)
                        ran_straight = True
                else:
                    ran_straight = False

                # Figure eight: 101
                if b2 == 1 and b1 == 0 and b0 == 1:
                    if not ran_figure8:
                        print("Figure Eight")
                        figure_eight(motors)
                        ran_figure8 = True
                else:
                    ran_figure8 = False

                # Square: 110
                if b2 == 1 and b1 == 1 and b0 == 0:
                    if not ran_square:
                        print("Square")
                        squares(motors)
                        ran_square = True
                else:
                    ran_square = False

                # Single sound detect: 000
                if b2 == 0 and b1 == 0 and b0 == 0:
                    if not ran_single:
                        print("Single Sound")
                        ran_single = True
                        single_sound(motors)
                else:
                    ran_single = False

                # Debug ADC loop: 111
                if b2 == 1 and b1 == 1 and b0 == 1:
                    if not ran_debug:
                        print("Debugging")
                        ran_debug = True
                        debug_adc_loop()
                else:
                    ran_debug = False

            else:
                bit3_confirmed = False
                ran_straight = False
                ran_figure8 = False
                ran_square = False
                ran_single = False
                ran_debug = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
