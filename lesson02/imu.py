from machine import I2C, Pin
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

        # Read gyroscope data
        gyro_data = read_register(OUTX_L_G, 6)
        gyro_x = twos_complement(gyro_data[1] << 8 | gyro_data[0], 16)
        gyro_y = twos_complement(gyro_data[3] << 8 | gyro_data[2], 16)
        gyro_z = twos_complement(gyro_data[5] << 8 | gyro_data[4], 16)
        gyro_sensitivity = 0.00875  # Sensitivity for ±245 dps full scale
        gyro_x_dps = gyro_x * gyro_sensitivity
        gyro_y_dps = gyro_y * gyro_sensitivity
        gyro_z_dps = gyro_z * gyro_sensitivity

        return accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps
    except Exception as e:
        print(f"[{ticks_ms()}] read_IMU error: {e}")
        return None

# Main script
def main():
    try:
        init_IMU()
        print("IMU initialized successfully.")
    except RuntimeError as e:
        print(f"IMU initialization failed: {e}")
        return

    # Continuously read and display IMU data
    while True:
        sensor_data = read_IMU()
        if sensor_data:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = sensor_data
            print(f"Accel (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
            print(f"Gyro (dps): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
        else:
            print("Failed to read IMU data.")

        sleep_ms(100)  # Adjust delay as needed

# Run the script
if __name__ == "__main__":
    main()
