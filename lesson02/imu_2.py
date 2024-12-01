from machine import I2C, Pin, PWM
import math
from time import ticks_ms, sleep_ms

# Configuration constants
I2C_SDA_PIN = 4  # GPIO pin for SDA
I2C_SCL_PIN = 5  # GPIO pin for SCL
I2C_FREQ = 400000  # Frequency for I2C communication
LSM6DS3_ADDR = 0x6B  # LSM6DS3 I2C address
WHO_AM_I_REG = 0x0F
CTRL1_XL = 0x10
CTRL2_G = 0x11
OUTX_L_G = 0x22  # Gyro X-axis data register (Low byte)
OUTX_L_XL = 0x28  # Accel X-axis data register (Low byte)

# Power pins for the IMU
I2C_VDD_PIN = 3
I2C_GND_PIN = 2

# Initialize power pins
i2c_vdd_pin = Pin(I2C_VDD_PIN, Pin.OUT)
i2c_gnd_pin = Pin(I2C_GND_PIN, Pin.OUT)

# Power on the IMU
i2c_gnd_pin.value(0)
i2c_vdd_pin.value(1)

# Initialize I2C
i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

# Complementary filter parameters
COMPLEMENTARY_FILTER_ALPHA = 0.98  # Weight for gyro data
UPDATE_RATE = 0.02  # Update interval in seconds

# Initialize complementary filter state
roll = 0.0
pitch = 0.0
yaw = 0.0
height = 0.0
vertical_velocity = 0.0

# Write to a register
def write_register(register, value):
    i2c.writeto_mem(LSM6DS3_ADDR, register, bytes([value]))

# Read from a register
def read_register(register, length=1):
    return i2c.readfrom_mem(LSM6DS3_ADDR, register, length)

# Initialize the IMU
def init_IMU():
    who_am_i = read_register(WHO_AM_I_REG)[0]
    if who_am_i != 0x69:
        raise RuntimeError("LSM6DS3 not detected")
    write_register(CTRL1_XL, 0x60)  # Accelerometer: 1.66 kHz, ±2g full scale
    write_register(CTRL2_G, 0x60)  # Gyroscope: 1.66 kHz, 245 dps full scale

# Convert two's complement to integer
def twos_complement(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

# Read IMU data
def read_IMU():
    try:
        # Read accelerometer data
        accel_data = read_register(OUTX_L_XL, 6)
        accel_x = twos_complement(accel_data[1] << 8 | accel_data[0], 16)
        accel_y = twos_complement(accel_data[3] << 8 | accel_data[2], 16)
        accel_z = twos_complement(accel_data[5] << 8 | accel_data[4], 16)
        accel_sensitivity = 0.000061  # Sensitivity for ±2g full scale
        accel_x_g = accel_x * accel_sensitivity
        accel_y_g = accel_y * accel_sensitivity
        accel_z_g = accel_z * accel_sensitivity

        # Remap accelerometer axes for 90° CCW rotation
        accel_x_g, accel_y_g = accel_y_g, -accel_x_g

        # Read gyroscope data
        gyro_data = read_register(OUTX_L_G, 6)
        gyro_x = twos_complement(gyro_data[1] << 8 | gyro_data[0], 16)
        gyro_y = twos_complement(gyro_data[3] << 8 | gyro_data[2], 16)
        gyro_z = twos_complement(gyro_data[5] << 8 | gyro_data[4], 16)
        gyro_sensitivity = 0.00875  # Sensitivity for ±245 dps full scale
        gyro_x_dps = gyro_x * gyro_sensitivity
        gyro_y_dps = gyro_y * gyro_sensitivity
        gyro_z_dps = gyro_z * gyro_sensitivity

        # Remap gyroscope axes for 90° CCW rotation
        gyro_x_dps, gyro_y_dps = gyro_y_dps, -gyro_x_dps

        return accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps
    except Exception as e:
        print(f"[{ticks_ms()}] read_IMU error: {e}")
        return None


# Complementary filter for roll, pitch, yaw, and height
def update_orientation(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt):
    global roll, pitch, yaw, height, vertical_velocity

    # Roll and pitch estimation
    accel_roll = math.atan2(accel_y, accel_z) * 180 / math.pi
    accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi

    roll = COMPLEMENTARY_FILTER_ALPHA * (roll + gyro_x * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_roll
    pitch = COMPLEMENTARY_FILTER_ALPHA * (pitch + gyro_y * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch
    yaw += gyro_z * dt  # Yaw is integrated directly from gyro Z

    # Height estimation using vertical acceleration
#     accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)  # Total acceleration magnitude
#     print(accel_magnitude, accel_z)
#     accel_z_corrected = accel_z - (accel_magnitude - 1.014)  # Subtract 1g (gravity component in g)
#     accel_z_m_s2 = accel_z_corrected * 9.81  # Convert corrected acceleration to m/s^2
# 
#     vertical_velocity += accel_z_m_s2 * dt  # Integrate acceleration to velocity
#     height += vertical_velocity * dt  # Integrate velocity to height


    return roll, pitch, yaw, height

# Main script
def main():
    try:
        init_IMU()
        print("IMU initialized successfully.")
    except RuntimeError as e:
        print(f"IMU initialization failed: {e}")
        return
    
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
        motor.freq(15000)
        motor.duty_u16(int(0.1 * 65535))

    # Continuously read and display IMU data
    last_update = ticks_ms()
    while True:
        sensor_data = read_IMU()
        if sensor_data:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = sensor_data
            current_time = ticks_ms()
            dt = (current_time - last_update) / 1000.0  # Convert ms to seconds
            last_update = current_time

            roll, pitch, yaw, height = update_orientation(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt)

            print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            print(f"Height: {height:.2f} m")

        else:
            print("Failed to read IMU data.")

        sleep_ms(20)  # Adjust delay as needed

# Run the script
if __name__ == "__main__":
    main()
