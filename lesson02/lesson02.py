from machine import Pin, PWM, I2C
import time
import math
from time import ticks_ms, sleep_ms


# Constants
MAX_MOTOR_DUTY_CYCLE = 0.10  # Maximum duty cycle (10%)
BASE_MOTOR_DUTY_CYCLE = 0.05  # Base throttle (5%)
PWM_FREQUENCY = 50  # PWM frequency in Hz
UPDATE_RATE = 0.02  # Control loop period (50 Hz)
PID_GAIN = {"kp": 1.0, "ki": 0.01, "kd": 0.1}

# Motor pin mapping
motor_pins = {
    "front_right": 20,  # Motor 3
    "rear_right": 16,   # Motor 1
    "front_left": 11,   # Motor 2
    "rear_left": 15     # Motor 4
}

# Initialize motors
motors = {name: PWM(Pin(pin)) for name, pin in motor_pins.items()}
for motor in motors.values():
    motor.freq(PWM_FREQUENCY)
    motor.duty_u16(0)

I2C_SDA_PIN = 4  # GPIO pin for SDA
I2C_SCL_PIN = 5  # GPIO pin for SCL

# Power pins for the IMU
I2C_VDD_PIN = 3
I2C_GND_PIN = 2

OUTX_L_XL = 0x28  # Accel X-axis data register (Low byte)
LSM6DS3_ADDR = 0x6B  # LSM6DS3 I2C address


# Initialize power pins
i2c_vdd_pin = Pin(I2C_VDD_PIN, Pin.OUT)
i2c_gnd_pin = Pin(I2C_GND_PIN, Pin.OUT)

# Power on the IMU
i2c_gnd_pin.value(0)
i2c_vdd_pin.value(1)

time.sleep(1)

# IMU setup (I2C for LSM6DS3 or similar sensor)
i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))  # Adjust pins for your setup
imu_address = 0x6B  # Default I2C address for LSM6DS3


# PID controller class
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# IMU functions
# Convert two's complement to integer
def twos_complement(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val


# Read from a register
def read_register(register, length=1):
    return i2c.readfrom_mem(LSM6DS3_ADDR, register, length)

# Read accelerometer and gyroscope data, and calculate roll and pitch
def read_imu():
    """Reads roll and pitch angles from the IMU, with data conversion and error handling."""
    try:
        # Read accelerometer data
        accel_data = read_register(OUTX_L_XL, 6)  # Read 6 bytes from accelerometer
        accel_x = twos_complement(accel_data[1] << 8 | accel_data[0], 16)
        accel_y = twos_complement(accel_data[3] << 8 | accel_data[2], 16)
        accel_z = twos_complement(accel_data[5] << 8 | accel_data[4], 16)
        accel_sensitivity = 0.000061  # Sensitivity for ±2g full scale
        accel_x_g = accel_x * accel_sensitivity
        accel_y_g = accel_y * accel_sensitivity
        accel_z_g = accel_z * accel_sensitivity

        # Calculate roll and pitch angles
        roll = math.atan2(accel_y_g, accel_z_g) * 180 / math.pi
        pitch = math.atan2(-accel_x_g, math.sqrt(accel_y_g**2 + accel_z_g**2)) * 180 / math.pi
        print (accel_x_g,accel_y_g,accel_z_g)
        return roll, pitch
    except OSError as e:
        print(f"[{ticks_ms()}] IMU read error: {e}")
        return 0.0, 0.0



# Motor control
def set_motor_duty_cycle(motor, duty_cycle):
    """Set motor duty cycle."""
    duty = max(0, min(int(duty_cycle * 65535), 65535))  # Clamp to [0, 1]
    motor.duty_u16(duty)

def update_motors(throttle, roll_correction, pitch_correction):
    """Update motor speeds based on throttle and corrections."""
    motor_speeds = {
        "front_right": throttle - roll_correction + pitch_correction,
        "rear_right": throttle - roll_correction - pitch_correction,
        "front_left": throttle + roll_correction + pitch_correction,
        "rear_left": throttle + roll_correction - pitch_correction
    }

    for name, speed in motor_speeds.items():
        set_motor_duty_cycle(motors[name], speed)

# Main hover test
def hover_test():
    print("Starting hover test with IMU and PID controllers...")

    # Initialize PID controllers for roll and pitch
    pid_roll = PID(PID_GAIN["kp"], PID_GAIN["ki"], PID_GAIN["kd"])
    pid_pitch = PID(PID_GAIN["kp"], PID_GAIN["ki"], PID_GAIN["kd"])

    # Base throttle to maintain hover
    throttle = BASE_MOTOR_DUTY_CYCLE

    try:
        while True:
            # Read IMU for roll and pitch angles
            roll, pitch = read_imu()

            # Compute corrections using PID controllers
            roll_correction = pid_roll.compute(0, roll, UPDATE_RATE)
            pitch_correction = pid_pitch.compute(0, pitch, UPDATE_RATE)

            # Clamp corrections to prevent exceeding motor range
            roll_correction = max(-MAX_MOTOR_DUTY_CYCLE, min(roll_correction, MAX_MOTOR_DUTY_CYCLE))
            pitch_correction = max(-MAX_MOTOR_DUTY_CYCLE, min(pitch_correction, MAX_MOTOR_DUTY_CYCLE))

            # Update motor speeds
            update_motors(throttle, roll_correction, pitch_correction)

            # Debug print in one line
#             print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, ",
#                   f"Roll Correction: {roll_correction:.4f}, Pitch Correction: {pitch_correction:.4f}, ",
#                   f"Throttle: {throttle:.4f}, ",
#                   f"FR: {throttle - roll_correction + pitch_correction:.4f}, ",
#                   f"RR: {throttle - roll_correction - pitch_correction:.4f}, ",
#                   f"FL: {throttle + roll_correction + pitch_correction:.4f}, ",
#                   f"RL: {throttle + roll_correction - pitch_correction:.4f}")

            # Wait for next control loop iteration
            time.sleep(UPDATE_RATE)

    except KeyboardInterrupt:
        print("Test interrupted. Stopping motors.")
    finally:
        # Stop all motors
        for motor in motors.values():
            set_motor_duty_cycle(motor, 0.0)
        print("Hover test completed. Motors stopped.")

# Run the hover test
hover_test()
