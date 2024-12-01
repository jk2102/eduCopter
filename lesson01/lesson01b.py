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
MAX_MOTOR_DUTY_CYCLE = 0.5  # Maximum duty cycle (10%)
PHASE_DURATION = 5       # Duration for each phase (ramp up, hold, ramp down) in seconds
STEP_DELAY = 0.05           # Time delay between duty cycle increments
STEP_SIZE = STEP_DELAY / PHASE_DURATION * MAX_MOTOR_DUTY_CYCLE  # Step size based on phase duration

# Initialize PWM for each motor
motors = {name: PWM(Pin(pin)) for name, pin in motor_pins.items()}
for pwm in motors.values():
    pwm.freq(50)  # Standard ESC frequency (50Hz)

def set_motor_duty_cycle(pwm, duty_cycle):
    """Convert a duty cycle (0.0 to 1.0) into a 16-bit value and set it."""
    duty_u16 = int(duty_cycle * 65535)
    pwm.duty_u16(duty_u16)

def ramp_motors(duty_cycle_target, duration, ramp_down=False):
    """Ramp all motors to or from a target duty cycle over a specified duration."""
    current_duty = MAX_MOTOR_DUTY_CYCLE if ramp_down else 0.0
    while (current_duty <= duty_cycle_target and not ramp_down) or (current_duty >= 0.0 and ramp_down):
        for pwm in motors.values():
            set_motor_duty_cycle(pwm, current_duty)
        current_duty += -STEP_SIZE if ramp_down else STEP_SIZE
        time.sleep(STEP_DELAY)

def hold_motors(duty_cycle, duration):
    """Hold all motors at a specific duty cycle for a specified duration."""
    for pwm in motors.values():
        set_motor_duty_cycle(pwm, duty_cycle)
    time.sleep(duration)

def stop_motors():
    """Stop all motors by setting their duty cycles to 0."""
    for pwm in motors.values():
        set_motor_duty_cycle(pwm, 0.0)

def main():
    print("Init pause!")
    led = Pin(25, Pin.OUT)
    led.on()
    time.sleep(10)
    print("Starting hover test firmware...")
    try:
        print("Ramping up motors...")
        ramp_motors(MAX_MOTOR_DUTY_CYCLE, PHASE_DURATION)

        print("Holding motors at max duty cycle...")
        hold_motors(MAX_MOTOR_DUTY_CYCLE, PHASE_DURATION)

        print("Ramping down motors...")
        ramp_motors(MAX_MOTOR_DUTY_CYCLE, PHASE_DURATION, ramp_down=True)

        print("Hover test complete.")
    except KeyboardInterrupt:
        print("Interrupted! Stopping motors...")
    finally:
        stop_motors()

if __name__ == "__main__":
    main()
