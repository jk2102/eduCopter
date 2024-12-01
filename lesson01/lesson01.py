from machine import Pin, PWM
import time

# Define motor pins and corresponding PWM objects
motor_pins = {
    "front_right": 20,  # Motor 3
    "rear_right": 16,   # Motor 1
    "front_left": 11,   # Motor 2
    "rear_left": 15     # Motor 4
}

# Constants
MAX_MOTOR_DUTY_CYCLE = 0.2  # Maximum duty cycle (10%)
TEST_DURATION = 5           # Duration of each motor test in seconds
STEP_SIZE = 0.002            # Increment/decrement step size for duty cycle
STEP_DELAY = TEST_DURATION / (2.0*MAX_MOTOR_DUTY_CYCLE/STEP_SIZE)          # Time delay between duty cycle increments

# Initialize PWM for each motor
motors = {name: PWM(Pin(pin)) for name, pin in motor_pins.items()}
for pwm in motors.values():
    pwm.freq(10000)  # Standard ESC frequency (50Hz)

def set_motor_duty_cycle(pwm, duty_cycle):
    """Convert a duty cycle (0.0 to 1.0) into a 16-bit value and set it."""
    duty_u16 = int(duty_cycle * 65535)
    pwm.duty_u16(duty_u16)

def test_motor(name, pwm):
    """Test a single motor by ramping up and down the duty cycle."""
    print(f"Testing motor: {name}")
    # Ramp up
    duty_cycle = 0.0
    while duty_cycle <= MAX_MOTOR_DUTY_CYCLE:
        set_motor_duty_cycle(pwm, duty_cycle)
        duty_cycle += STEP_SIZE
        time.sleep(STEP_DELAY)
    # Ramp down
    while duty_cycle >= 0.0:
        set_motor_duty_cycle(pwm, duty_cycle)
        duty_cycle -= STEP_SIZE
        time.sleep(STEP_DELAY)
    # Stop motor
    set_motor_duty_cycle(pwm, 0.0)
    print(f"Completed testing motor: {name}")

def main():
    print("Starting motor test sequence...")
    for name, pwm in motors.items():
        test_motor(name, pwm)
        time.sleep(0.5)  # Small delay between motor tests
    print("Motor test sequence completed.")

    # Cleanup
    for pwm in motors.values():
        set_motor_duty_cycle(pwm, 0.0)

if __name__ == "__main__":
    main()
