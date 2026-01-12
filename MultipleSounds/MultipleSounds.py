"""
Robot car control with:
  - Dual motor RPM feedback using optical encoders
  - Dip-switch mode selection (straight / figure eight / squares / single sound / multiple sounds / debug)
  - Bump switch safety stop
  - Two ADC readers (read_adc1 and read_adc2) for two frequency channels

Notes:
  - Gains/goal RPM are currently hard-coded (see "Control parameters").
  - ADC readers are imported from adc_reader1 / adc_reader2 (package-relative imports).
  - This script is structured as a single file, but the functions are organized and
    commented for readability.
"""

import time

try:
    import RPi.GPIO as GPIO
    from .adc_reader1 import read_adc1
    from .adc_reader2 import read_adc2
except ImportError:
    GPIO = None
    read_adc1 = None
    read_adc2 = None


# ----------------------------
# Pin / hardware configuration
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

# ADC wiring (BCM) - configured here for completeness; adc_reader modules handle PWM/comp
PWM_PIN1 = 5
COMPARATOR1 = 21
PWM_PIN2 = 26
COMPARATOR2 = 24

# Encoder properties
TICKS_PER_REV = 40

# Switch debounce and confirmation delay
DEBOUNCE_DELAY_S = 0.05
BIT3_CONFIRM_DELAY_S = 3.0

# Feedback loop timing
MEASUREMENT_INTERVAL_S = 0.20

# Motion timing
TURN_90_LEFT_S = 0.52

# Sampling rate is kept for context (ADC readers may depend on it)
SAMPLE_RATE = 40000


# ----------------------------
# Control parameters
# ----------------------------

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
# Utilities
# ----------------------------

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

    # ADC pins (the reader modules likely initialize their own PWM, but set safe modes here)
    GPIO.setup(COMPARATOR1, GPIO.IN)
    GPIO.setup(PWM_PIN1, GPIO.OUT)
    GPIO.setup(COMPARATOR2, GPIO.IN)
    GPIO.setup(PWM_PIN2, GPIO.OUT)


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
# Motor PWM wrapper
# ----------------------------

class MotorOutputs:
    """Holds PWM objects for both motors and provides a safe stop()."""
    def __init__(self):
        self.pwm1_f = GPIO.PWM(MOTOR1_F, 1000)
        self.pwm1_b = GPIO.PWM(MOTOR1_B, 1000)
        self.pwm2_f = GPIO.PWM(MOTOR2_F, 1000)
        self.pwm2_b = GPIO.PWM(MOTOR2_B, 1000)

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
# Safety callback (bump switch)
# ----------------------------

def make_bump_callback(motors: MotorOutputs):
    """Factory to create bump callback that can stop PWM safely."""
    def _bumped(_channel: int) -> None:
        global isBumped
        isBumped = True
        motors.stop()
        print("Bumped!")
        time.sleep(2.0)
    return _bumped


# ----------------------------
# Movement primitives
# ----------------------------

def left_turn_90(motors: MotorOutputs) -> None:
    """Open-loop timed 90-degree left turn."""
    motors.pwm1_f.start(calculateDutyCycle1(100))
    motors.pwm2_b.start(calculateDutyCycle2(100))
    time.sleep(TURN_90_LEFT_S)
    motors.stop()
    time.sleep(1.0)


def increment_left(motors: MotorOutputs, amount_s: float) -> None:
    """Small left rotation (used during sound seeking)."""
    motors.pwm1_f.start(calculateDutyCycle1(100))
    motors.pwm2_b.start(calculateDutyCycle2(100))
    time.sleep(amount_s)
    motors.stop()
    time.sleep(0.5)


def increment_right(motors: MotorOutputs, amount_s: float) -> None:
    """Small right rotation (used during sound seeking)."""
    motors.pwm2_f.start(calculateDutyCycle1(100))
    motors.pwm1_b.start(calculateDutyCycle2(100))
    time.sleep(amount_s)
    motors.stop()
    time.sleep(0.5)


def line(motors: MotorOutputs, runT: float, goalRpm1: float, goalRpm2: float) -> None:
    """
    Drive forward for runT seconds while applying PI control to match motor RPM targets.

    goalRpm1: target RPM for motor 1
    goalRpm2: target RPM for motor 2
    """
    global tickCount1, tickCount2

    time.sleep(1.0)
    print("Driving a straight line")

    tickCount1 = 0
    tickCount2 = 0

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

    # Integrators (time-weighted error)
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


def straightLine(motors: MotorOutputs, left_rpm: float, right_rpm: float) -> None:
    """Drive forward for a fixed runtime and print encoder totals."""
    global tickAccumulator1, tickAccumulator2

    runT = 3.25
    line(motors, runT, left_rpm, right_rpm)

    print("Count 1:", tickAccumulator1)
    print("Count 2:", tickAccumulator2)

    tickAccumulator1 = 0
    tickAccumulator2 = 0


# ----------------------------
# Complex behaviors
# ----------------------------

def figureEight(motors: MotorOutputs) -> None:
    """Perform figure-eight pattern."""
    line(motors, 5.2, 50, 30)
    time.sleep(0.5)
    straightLine(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 30, 50)
    time.sleep(0.5)
    straightLine(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 50, 30)
    time.sleep(0.5)
    straightLine(motors, 45, 45)
    time.sleep(0.5)

    line(motors, 4.6, 30, 50)
    time.sleep(0.5)
    straightLine(motors, 45, 45)


def squares(motors: MotorOutputs) -> None:
    """Drive two squares."""
    print("calling squares()")

    for _ in range(2):
        for _ in range(4):
            time.sleep(1.0)
            straightLine(motors, cmdArgGoalRpm, cmdArgGoalRpm)
            time.sleep(1.0)
            left_turn_90(motors)


def averaged_adc(read_fn, samples: int = 5) -> float:
    """Read ADC multiple times and return the average."""
    total = 0.0
    for _ in range(samples):
        total += read_fn()
    return total / float(samples)


def singleSound(motors: MotorOutputs) -> None:
    """
    Seek the first target frequency using ADC1 readings, then drive forward until bumped.
    """
    if read_adc1 is None:
        print("read_adc1 not available.")
        return

    print("Single Sound is Running.")

    global isBumped
    value = 0.0
    prevValue = 0.0
    count_3v = 0
    threshold = 3.0
    turns = 0

    while value < 3.3:
        increment_left(motors, 0.25)

        turns += 1
        if turns == 10:
            threshold = 2.3

        value = averaged_adc(read_adc1, samples=5)
        print(f"{value}V from ADC")

        # Peak detection (may not saturate at 3.3)
        if (prevValue > value) and (prevValue > threshold):
            while value < 3.3:
                increment_right(motors, 0.1)
                prevValue = value
                value = averaged_adc(read_adc1, samples=5)
                print(f"{value}V from ADC")
                if prevValue > value:
                    increment_left(motors, 0.11)
                    break
            break

        # Saturation case
        if value == 3.3:
            while value == 3.3:
                increment_left(motors, 0.08)
                value = read_adc1()
                print(f"{value}V from ADC")
                count_3v += 1

            for _ in range(count_3v // 2):
                increment_right(motors, 0.08)
                time.sleep(0.1)
            break

        prevValue = value

    # Drive straight until bumped
    while not isBumped:
        time.sleep(1.0)
        line(motors, 3.0, cmdArgGoalRpm, cmdArgGoalRpm)

    isBumped = False


def secondSound(motors: MotorOutputs) -> None:
    """
    Seek the second target frequency using ADC2 readings, reverse to re-center,
    then drive forward until bumped.
    """
    if read_adc2 is None:
        print("read_adc2 not available.")
        return

    print("Second sound is performing.")

    global isBumped
    isBumped = False

    value = 0.0
    prevValue = 0.0
    count_3v = 0
    threshold = 3.0
    turns = 0

    # Reverse back to center
    print("REVERSING...")
    motors.pwm1_b.start(75)
    motors.pwm2_b.start(75)
    time.sleep(1.0)
    time.sleep(1.0)
    time.sleep(1.0)
    time.sleep(1.5)
    motors.stop()
    print("...DONE REVERSING")
    time.sleep(2.0)

    # Rotate until we detect strong signal or a clear peak
    while value < 3.3:
        increment_left(motors, 0.25)

        turns += 1
        if turns == 10:
            threshold = 2.0

        value = averaged_adc(read_adc2, samples=5)
        print(f"{value}V from ADC")

        if (prevValue > value) and (prevValue > threshold):
            while value < 3.3:
                increment_right(motors, 0.1)
                prevValue = value
                value = averaged_adc(read_adc2, samples=5)
                print(f"{value}V from ADC")
                if prevValue > value:
                    increment_left(motors, 0.11)
                    break
            break

        if value == 3.3:
            while value == 3.3:
                increment_left(motors, 0.08)
                value = read_adc2()
                print(f"{value}V from ADC")
                count_3v += 1

            for _ in range(count_3v // 2 + 2):
                increment_right(motors, 0.08)
                time.sleep(0.1)
            break

        prevValue = value

    # Turn ~180 degrees to face the sound
    for _ in range(9):
        increment_right(motors, 0.2)

    # Drive straight until bumped
    isBumped = False
    while not isBumped:
        print("Driving Straight")
        time.sleep(1.0)
        line(motors, 5.5, cmdArgGoalRpm, cmdArgGoalRpm)

    print("Done going straight")


def debug_compare_adcs(motors: MotorOutputs) -> None:
    """Rotate and compare ADC readings side-by-side until bumped."""
    if read_adc1 is None or read_adc2 is None:
        print("ADC readers not available.")
        return

    while not isBumped:
        print("________________")
        increment_right(motors, 0.2)

        value1 = averaged_adc(read_adc1, samples=5)
        value2 = averaged_adc(read_adc2, samples=5)

        print(f"Old ADC: {value1}  New ADC: {value2}")
        time.sleep(0.05)


# ----------------------------
# Main loop
# ----------------------------

def main() -> None:
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    setup_gpio()
    motors = MotorOutputs()

    # Register events after motors exist
    GPIO.add_event_detect(OPTICAL_1, GPIO.BOTH, callback=counting1, bouncetime=25)
    GPIO.add_event_detect(OPTICAL_2, GPIO.BOTH, callback=counting2, bouncetime=25)
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
    ran_double = False
    ran_debug = False

    try:
        while True:
            now = time.time()

            b3 = sw3.update(now)
            b2 = sw2.update(now)
            b1 = sw1.update(now)
            b0 = sw0.update(now)

            # Optional visibility while developing; comment out if noisy
            # print(f"Switch values: s3={b3} s2={b2} s1={b1} s0={b0}")

            if b3 == GPIO.HIGH:
                if not bit3_confirmed:
                    time.sleep(BIT3_CONFIRM_DELAY_S)
                    bit3_confirmed = True

                # Straight: 100
                if b2 == 1 and b1 == 0 and b0 == 0:
                    if not ran_straight:
                        straightLine(motors, 45, 45)
                        ran_straight = True
                else:
                    ran_straight = False

                # Figure eight: 101
                if b2 == 1 and b1 == 0 and b0 == 1:
                    if not ran_figure8:
                        print("Figure Eight")
                        figureEight(motors)
                        ran_figure8 = True
                else:
                    ran_figure8 = False

                # Squares: 110
                if b2 == 1 and b1 == 1 and b0 == 0:
                    if not ran_square:
                        print("Square")
                        squares(motors)
                        ran_square = True
                else:
                    ran_square = False

                # Single sound: 000
                if b2 == 0 and b1 == 0 and b0 == 0:
                    if not ran_single:
                        print("Single Sound")
                        ran_single = True
                        singleSound(motors)
                else:
                    ran_single = False

                # Multiple sounds: 001
                if b2 == 0 and b1 == 0 and b0 == 1:
                    if not ran_double:
                        print("Multiple Sounds")
                        ran_double = True
                        singleSound(motors)
                        secondSound(motors)
                else:
                    ran_double = False

                # Debug mode: 111
                if b2 == 1 and b1 == 1 and b0 == 1:
                    if not ran_debug:
                        print("Debugging")
                        ran_debug = True
                        debug_compare_adcs(motors)
                else:
                    ran_debug = False

            else:
                bit3_confirmed = False
                ran_straight = False
                ran_figure8 = False
                ran_square = False
                ran_single = False
                ran_double = False
                ran_debug = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
