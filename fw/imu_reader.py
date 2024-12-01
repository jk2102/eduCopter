import uasyncio as asyncio
from machine import I2C, Pin
import math
from time import ticks_ms, ticks_diff, sleep
from quadcopter_model import QuadcopterModel

class IMUReader:
    def __init__(self, model):
        """
        Initialize the IMUReader module.

        :param model: QuadcopterModel instance containing both static configuration and dynamic state data.
        """
        self.model = model
        
        # Power on the IMU and initialize
        self.power_on_sensor()

        # Initialize I2C
        self.i2c = I2C(0, scl=Pin(model.config.I2C_SCL_PIN), sda=Pin(model.config.I2C_SDA_PIN), freq=model.config.I2C_FREQ)

        # Initialize the IMU
        self.init_IMU()

        # Time tracking
        self.last_update = ticks_ms()

    def power_on_sensor(self):
        """Power on the IMU sensor by setting VDD and GND pins."""
        # Initialize power pins for the IMU
        self.i2c_vdd_pin = Pin(self.model.config.I2C_VDD_PIN, Pin.OUT)
        self.i2c_gnd_pin = Pin(self.model.config.I2C_GND_PIN, Pin.OUT)

        # Power on the IMU
        self.i2c_gnd_pin.value(0)  # GND is connected
        self.i2c_vdd_pin.value(1)  # VDD is set to power on

        # Allow power to stabilize
        sleep(0.1)  # 100 ms delay for power supply stabilization

    def write_register(self, register, value):
        self.i2c.writeto_mem(self.model.config.IMU_ADDRESS, register, bytes([value]))

    def read_register(self, register, length=1):
        return self.i2c.readfrom_mem(self.model.config.IMU_ADDRESS, register, length)

    def init_IMU(self):
        """Initialize the IMU sensor with the required settings."""
        who_am_i = self.read_register(self.model.config.WHO_AM_I_REG)[0]
        if who_am_i != 0x69:
            raise RuntimeError("LSM6DS3 not detected")
        # Accelerometer: 1.66 kHz, ±2g full scale
        self.write_register(self.model.config.CTRL1_XL, 0x60)
        # Gyroscope: 1.66 kHz, 245 dps full scale
        self.write_register(self.model.config.CTRL2_G, 0x60)

    def twos_complement(self, val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def read_IMU(self):
        """Read accelerometer and gyroscope data from the IMU."""
        try:
            # Read accelerometer data
            accel_data = self.read_register(self.model.config.OUTX_L_XL, 6)
            accel_x = self.twos_complement(accel_data[1] << 8 | accel_data[0], 16)
            accel_y = self.twos_complement(accel_data[3] << 8 | accel_data[2], 16)
            accel_z = self.twos_complement(accel_data[5] << 8 | accel_data[4], 16)
            accel_sensitivity = 0.000061  # Sensitivity for ±2g full scale
            accel_x_g = (accel_x * accel_sensitivity)
            accel_y_g = (accel_y * accel_sensitivity)
            accel_z_g = (accel_z * accel_sensitivity)


            # Remap accelerometer axes for 90° CCW rotation
            accel_x_g, accel_y_g = accel_y_g, -accel_x_g

            # Read gyroscope data
            gyro_data = self.read_register(self.model.config.OUTX_L_G, 6)
            gyro_x = self.twos_complement(gyro_data[1] << 8 | gyro_data[0], 16)
            gyro_y = self.twos_complement(gyro_data[3] << 8 | gyro_data[2], 16)
            gyro_z = self.twos_complement(gyro_data[5] << 8 | gyro_data[4], 16)
            gyro_sensitivity = 0.00875  # Sensitivity for ±245 dps full scale
            gyro_x_dps = (gyro_x * gyro_sensitivity)
            gyro_y_dps = (gyro_y * gyro_sensitivity)
            gyro_z_dps = (gyro_z * gyro_sensitivity)


            # Remap gyroscope axes for 90° CCW rotation
            gyro_x_dps, gyro_y_dps = gyro_y_dps, -gyro_x_dps

            return accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps
        except Exception as e:
            print(f"[{ticks_ms()}] read_IMU error: {e}")
            return None

    def update_orientation(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt):
        """Update the roll, pitch, and yaw using a complementary filter."""
        alpha = self.model.config.COMPLEMENTARY_FILTER_ALPHA

        # Roll and pitch estimation using accelerometer
        accel_roll = math.atan2(accel_y, accel_z) * (180 / math.pi)
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * (180 / math.pi)
        
        #Debug
        #print(f"{accel_roll} {gyro_x}")
        #print(f"{accel_pitch} {gyro_y}")
        #print(f"{gyro_z}")
        
        # Apply complementary filter to estimate roll and pitch
        self.model.data.roll = alpha * (self.model.data.roll + gyro_x * dt) + (1 - alpha) * accel_roll
        self.model.data.pitch = alpha * (self.model.data.pitch + gyro_y * dt) + (1 - alpha) * accel_pitch
        self.model.data.yaw += gyro_z * dt  # Yaw is integrated directly from gyro Z

    def detect_crash(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        """Detect crash based on accelerometer, gyroscope, roll, and pitch thresholds."""
        # Calculate the magnitude of the acceleration
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

        # Check for crash conditions with individual conditions for detailed feedback

        if accel_magnitude > self.model.config.ACCEL_THRESHOLD:
            #print("Crash condition triggered: Acceleration magnitude exceeded threshold.")
            self.model.data.crash_detected = True

        if abs(gyro_x) > self.model.config.GYRO_THRESHOLD:
            #print("Crash condition triggered: Gyroscope X-axis angular velocity exceeded threshold.")
            self.model.data.crash_detected = True

        if abs(gyro_y) > self.model.config.GYRO_THRESHOLD:
            #print("Crash condition triggered: Gyroscope Y-axis angular velocity exceeded threshold.")
            self.model.data.crash_detected = True

        if abs(gyro_z) > self.model.config.GYRO_THRESHOLD:
            #print("Crash condition triggered: Gyroscope Z-axis angular velocity exceeded threshold.")
            self.model.data.crash_detected = True

        if abs(self.model.data.roll) > self.model.config.ROLL_THRESHOLD:
            #print("Crash condition triggered: Roll angle exceeded threshold.")
            self.model.data.crash_detected = True

        if abs(self.model.data.pitch) > self.model.config.PITCH_THRESHOLD:
            #print("Crash condition triggered: Pitch angle exceeded threshold.")
            self.model.data.crash_detected = True

#         if accel_z < -0.2:
#             print("Crash condition triggered: Accelerometer Z-axis value too low, possible free fall.")
#             self.model.data.crash_detected = True

            
    async def calibrate_IMU(self, num_samples=100):
        """
        Calibrate the IMU by averaging sensor readings over a number of samples.

        :param num_samples: The number of samples to use for calibration.
        """
        print("Starting IMU calibration...")
        
        await asyncio.sleep(1.0)
        
        accel_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        gyro_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        for n in range(num_samples):
            # Read IMU data
            sensor_data = self.read_IMU()
            if sensor_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = sensor_data

                # Accumulate sensor readings
                accel_sum['x'] += accel_x
                accel_sum['y'] += accel_y
                accel_sum['z'] += accel_z - 1.0  # Subtract gravity (1g) from z-axis
                gyro_sum['x'] += gyro_x
                gyro_sum['y'] += gyro_y
                gyro_sum['z'] += gyro_z

            await asyncio.sleep(0.02)  # Wait 20 ms before next sample

        # Calculate average values for calibration
        self.model.data.imu_calib['accel_offset']['x'] = accel_sum['x'] / num_samples
        self.model.data.imu_calib['accel_offset']['y'] = accel_sum['y'] / num_samples
        self.model.data.imu_calib['accel_offset']['z'] = accel_sum['z'] / num_samples
        self.model.data.imu_calib['gyro_drift']['x'] = gyro_sum['x'] / num_samples
        self.model.data.imu_calib['gyro_drift']['y'] = gyro_sum['y'] / num_samples
        self.model.data.imu_calib['gyro_drift']['z'] = gyro_sum['z'] / num_samples
        
        self.model.data.imu_calib['calibrated?'] = True

        print(self.model.data.imu_calib)
        print("IMU calibration complete.")
        
    async def read_sensor(self):
        """Asynchronously read data from the IMU and update the quadcopter state."""
        while True:
            if self.model.data.flight_mode == "ARMED" and not self.model.data.imu_calib['calibrated?']:
                await self.calibrate_IMU(self.model.config.IMU_CALIB_SAMPLES)
            elif self.model.data.flight_mode != "DISARMED": 
                sensor_data = self.read_IMU()
                if sensor_data:
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = sensor_data
                   
                    # Apply calibration offsets
                    accel_x -= self.model.data.imu_calib['accel_offset']['x']
                    accel_y -= self.model.data.imu_calib['accel_offset']['y']
                    accel_z -= self.model.data.imu_calib['accel_offset']['z']
                    gyro_x -= self.model.data.imu_calib['gyro_drift']['x']
                    gyro_y -= self.model.data.imu_calib['gyro_drift']['y']
                    gyro_z -= self.model.data.imu_calib['gyro_drift']['z']
                   
                    current_time = ticks_ms()
                    dt = ticks_diff(current_time, self.last_update) / 1000.0  # Convert ms to seconds
                    self.last_update = current_time

                    # Update orientation based on IMU data
                    self.update_orientation(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt)

                    # Update QuadcopterState with IMU data
                    self.model.data.imu_data = {
                        'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
                        'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                        'temperature': 0.0  # Placeholder for temperature reading
                    }

                    # Check for crash conditions
                    self.detect_crash(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

                else:
                    print("Failed to read IMU data.")

            await asyncio.sleep(0.02)  # Adjust the update rate as needed

if __name__ == '__main__':
    from heartbeat import Heartbeat

    async def main():
        # Instantiate the quadcopter model
        quad_model = QuadcopterModel()

        # Create IMUReader instance
        imu_reader = IMUReader(quad_model)
        heartbeat  = Heartbeat(quad_model)

        # Start the IMU reading loop and heartbeat
        imu_task = asyncio.create_task(imu_reader.read_sensor())
        heartbeat_task = asyncio.create_task(heartbeat.blink())

        # Assume it's armed initially for testing
        quad_model.data.flight_mode = 'ARMED'

        try:
            # Main loop to print IMU data
            while True:
                if quad_model.data.crash_detected:
                    quad_model.data.flight_mode = "CRASHED"
                # Fetch latest IMU data from the quadcopter state
                accel = quad_model.data.imu_data['acceleration']
                gyro = quad_model.data.imu_data['gyroscope']
                roll = quad_model.data.roll
                pitch = quad_model.data.pitch
                yaw = quad_model.data.yaw

                # Print accelerometer, gyroscope, and orientation values in fixed-width format
                print(f"Accel X: {accel['x']:+7.3f} | Y: {accel['y']:+7.3f} | Z: {accel['z']:+7.3f} || ",
                      f"Gyro X: {gyro['x']:+7.3f} | Y: {gyro['y']:+7.3f} | Z: {gyro['z']:+7.3f} || ",
                      f"Roll: {roll:+7.2f}° | Pitch: {pitch:+7.2f}° | Yaw: {yaw:+7.2f}°")

                await asyncio.sleep(0.5)  # Print every 500 ms
        except KeyboardInterrupt:
            # Graceful shutdown on Ctrl+C
            imu_task.cancel()
            heartbeat_task.cancel()
            print("IMU reading stopped by user.")

    # Run the asyncio event loop
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program terminated by user.")
