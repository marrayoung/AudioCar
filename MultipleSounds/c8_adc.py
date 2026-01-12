"""
4-bit successive-approximation ADC reader (C8 ADC - original).

Implements a simple SAR ADC using:
  - PWM output as a DAC (through an RC filter)
  - Comparator digital output as the decision bit

Primary entry point:
    read_adc1() -> float  (estimated voltage, assuming 3.3V reference)

Notes:
  - Timing/settling delays are hardware-dependent; tune DAC_SETTLE_S and INITIAL_SETTLE_S.
  - The PWM->DAC mapping includes an offset for low duty cycles (kept from original behavior).
"""

import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


# ----------------------------
# Configuration
# ----------------------------

SAMPLE_RATE = 40000  # kept for context; not directly used in this routine

PWM_PIN1 = 5
COMPARATOR1 = 21

PWM_FREQ_HZ = 1500

VREF = 3.3

# Settling delays (tune for your RC/comparator)
DAC_SETTLE_S = 0.05
INITIAL_SETTLE_S = 0.10
FIRST_CYCLE_EXTRA_SETTLE_S = 0.10

# Small delay between SAR "clocks" (kept from original)
SAR_STEP_DELAY_S = 3.125e-6


# ----------------------------
# Global SAR state (bits)
# ----------------------------

bit3_1, bit2_1, bit1_1, bit0_1 = 1, 0, 0, 0


# ----------------------------
# Setup
# ----------------------------

def setup_gpio_and_pwm() -> "GPIO.PWM":
    """Initialize GPIO and return a PWM instance for the DAC output."""
    if GPIO is None:
        raise SystemExit("RPi.GPIO not available. Run this on a Raspberry Pi.")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(COMPARATOR1, GPIO.IN)
    GPIO.setup(PWM_PIN1, GPIO.OUT)

    pwm = GPIO.PWM(PWM_PIN1, PWM_FREQ_HZ)
    return pwm


# ----------------------------
# DAC + ADC routines
# ----------------------------

def dac_from_bits(pwm: "GPIO.PWM", b3: int, b2: int, b1: int, b0: int) -> None:
    """
    Output the given 4-bit value on the PWM-based DAC.

    Uses weighted bits to compute duty cycle and includes a small offset
    for low values (retained from original code).
    """
    # Equivalent to (b3*8 + b2*4 + b1*2 + b0*1) / 16, expressed as duty cycle.
    duty_cycle = 100.0 * ((b3 / 2) + (b2 / 4) + (b1 / 8) + (b0 / 16))

    if duty_cycle <= 95.0:
        duty_cycle += 5.0

    pwm.ChangeDutyCycle(max(0.0, min(100.0, duty_cycle)))
    time.sleep(DAC_SETTLE_S)


def adc_step(pwm: "GPIO.PWM", clk: int) -> None:
    """
    Perform one SAR step based on the current clock value.

    This preserves the original step ordering and logic:
      clk == 2 : decide bit2 (and possibly bit3)
      clk == 4 : decide bit1 (and possibly bit2)
      clk == 6 : decide bit0 (and possibly bit1)
      clk == 8 : finalize bit0

    Comparator convention:
      - If GPIO.input(COMPARATOR1) is HIGH, original code tended to set the current bit.
      - If LOW, it sometimes cleared the previous bit and set the next lower bit.
    """
    global bit3_1, bit2_1, bit1_1, bit0_1

    # Local copies to modify
    v3, v2, v1, v0 = bit3_1, bit2_1, bit1_1, bit0_1
    count = 0

    if clk == 2:
        time.sleep(FIRST_CYCLE_EXTRA_SETTLE_S)

    c1 = GPIO.input(COMPARATOR1)

    if clk == 2:
        if c1 > 0:
            v2 = 1
            count = 1
        elif c1 == 0 and bit2_1 == 0 and count == 0:
            v3 = 0
            v2 = 1

    elif clk == 4:
        if c1 > 0:
            v1 = 1
            count = 1
        elif c1 == 0 and bit1_1 == 0 and count == 0:
            v2 = 0
            v1 = 1

    elif clk == 6:
        if c1 > 0:
            v0 = 1
            count = 1
        elif c1 == 0 and bit0_1 == 0 and count == 0:
            v1 = 0
            v0 = 1

    elif clk == 8:
        if c1 == 0:
            v0 = 0

    bit3_1, bit2_1, bit1_1, bit0_1 = v3, v2, v1, v0
    dac_from_bits(pwm, bit3_1, bit2_1, bit1_1, bit0_1)


def read_adc1() -> float:
    """
    Run the full 4-cycle SAR routine and return the estimated input voltage.

    Returns:
        float: estimated voltage (0 to VREF)
    """
    global bit3_1, bit2_1, bit1_1, bit0_1

    pwm = setup_gpio_and_pwm()

    # Reset bits to known starting state
    bit3_1, bit2_1, bit1_1, bit0_1 = 1, 0, 0, 0

    pwm.start(50.0)
    time.sleep(INITIAL_SETTLE_S)

    try:
        clk = 2
        for _ in range(4):
            adc_step(pwm, clk)
            clk += 2
            time.sleep(SAR_STEP_DELAY_S)

        digital_value = (bit3_1 * 8) + (bit2_1 * 4) + (bit1_1 * 2) + bit0_1
        magnitude = (digital_value / 15.0) * VREF
        return magnitude

    finally:
        try:
            pwm.stop()
        finally:
            GPIO.cleanup()


# Optional direct test
# if __name__ == "__main__":
#     try:
#         print("Measured voltage:", read_adc1())
#     except KeyboardInterrupt:
#         GPIO.cleanup()
