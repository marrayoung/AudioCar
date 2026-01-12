"""
4-bit successive-approximation ADC using:
  - PWM output as a simple DAC (via RC filter)
  - Comparator digital output for threshold decisions

This script provides a `read_adc()` function that runs a 4-step SAR routine and
returns an estimated input voltage (assuming a 3.3V reference).

Notes:
  - The PWM "DAC" here is a rough approximation and depends heavily on your RC filter.
  - Timing/settling delays are conservative by default; tune for your hardware.
"""

import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


# ----------------------------
# Configuration
# ----------------------------

PWM_PIN = 6
COMPARATOR_PIN = 21

PWM_FREQ_HZ = 1000

# Reference voltage used for converting 4-bit code to volts
VREF = 3.3

# Settling delays (tune for your RC + comparator behavior)
DAC_SETTLE_S = 0.01
INITIAL_STABILIZE_S = 0.05


# ----------------------------
# Global SAR state (4 bits)
# ----------------------------

bit3, bit2, bit1, bit0 = 1, 0, 0, 0


# ----------------------------
# Helpers
# ----------------------------

def bits_to_code(b3: int, b2: int, b1: int, b0: int) -> int:
    """Convert 4 individual bits into a 0–15 integer."""
    return (b3 << 3) | (b2 << 2) | (b1 << 1) | b0


def code_to_voltage(code: int, vref: float = VREF) -> float:
    """Convert a 0–15 code into a voltage using the provided reference."""
    code = max(0, min(15, int(code)))
    return (code / 15.0) * vref


def code_to_duty_cycle(code: int) -> float:
    """
    Convert a 4-bit code (0–15) to a PWM duty cycle.

    If your PWM+RC DAC is calibrated differently, adjust this mapping.
    """
    code = max(0, min(15, int(code)))
    return (code / 15.0) * 100.0


# ----------------------------
# GPIO / PWM setup
# ----------------------------

def setup_gpio() -> "GPIO.PWM":
    """Initialize GPIO and return a configured PWM instance."""
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(COMPARATOR_PIN, GPIO.IN)
    GPIO.setup(PWM_PIN, GPIO.OUT)

    pwm = GPIO.PWM(PWM_PIN, PWM_FREQ_HZ)
    return pwm


# ----------------------------
# DAC + SAR logic
# ----------------------------

def set_dac_code(pwm: "GPIO.PWM", code: int) -> None:
    """
    Output a 4-bit code through the PWM-based DAC.

    The settle delay allows the RC filtered voltage to approach its new level.
    """
    duty = code_to_duty_cycle(code)

    # Optional offset compensation (kept from your original intent).
    # If you don't need this, remove it or calibrate it.
    if duty <= 95.0:
        duty += 5.0

    pwm.ChangeDutyCycle(max(0.0, min(100.0, duty)))
    time.sleep(DAC_SETTLE_S)


def comparator_is_high() -> bool:
    """
    Read the comparator output.

    Convention:
      - True  = comparator output is HIGH
      - False = comparator output is LOW
    """
    return bool(GPIO.input(COMPARATOR_PIN))


def sar_step(pwm: "GPIO.PWM", current_code: int, bit_mask: int) -> int:
    """
    One SAR step:
      - Tentatively set the target bit (current_code | bit_mask)
      - Read comparator
      - Keep or clear the bit based on comparator result

    Comparator convention matters here. This implementation assumes:
      - comparator HIGH means "input >= DAC"
      - comparator LOW  means "input < DAC"

    If your comparator behaves the opposite, flip the decision logic.
    """
    trial_code = current_code | bit_mask
    set_dac_code(pwm, trial_code)

    if comparator_is_high():
        # Input is above (or equal to) DAC threshold: keep the bit.
        return trial_code
    else:
        # Input is below DAC threshold: clear the bit.
        return current_code


def read_adc() -> float:
    """
    Run a full 4-bit SAR conversion and return the estimated input voltage.

    Returns:
        float: Estimated voltage at the ADC input (0 to VREF).
    """
    global bit3, bit2, bit1, bit0

    pwm = setup_gpio()

    # Start PWM at mid-scale to avoid huge initial jumps (optional).
    pwm.start(50.0)
    time.sleep(INITIAL_STABILIZE_S)

    # SAR starts at 0 and tests bits from MSB -> LSB
    code = 0

    try:
        for bit_mask in (0b1000, 0b0100, 0b0010, 0b0001):
            code = sar_step(pwm, code, bit_mask)

        # Update global bits (kept for compatibility with your original structure)
        bit3 = 1 if (code & 0b1000) else 0
        bit2 = 1 if (code & 0b0100) else 0
        bit1 = 1 if (code & 0b0010) else 0
        bit0 = 1 if (code & 0b0001) else 0

        return code_to_voltage(code)

    finally:
        try:
            pwm.stop()
        finally:
            GPIO.cleanup()
