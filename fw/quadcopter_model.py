# quadcopter_model.py

class QuadcopterModel:
    """The quadcopter model."""
    def __init__(self) -> None:
        self.config     = QuadcopterConfig()
        self.data       = QuadcopterState()

class QuadcopterConfig:
    """Static configuration parameters for the quadcopter."""
    def __init__(self):
        # I2C configuration for IMU LSM6DS3
        self.I2C_SDA_PIN = 4   # GPIO pin for SDA
        self.I2C_SCL_PIN = 5   # GPIO pin for SCL
        self.I2C_VDD_PIN = 3   # GPIO pin for VDD (Power)
        self.I2C_GND_PIN = 2   # GPIO pin for GND
        self.IMU_ADDRESS = 0x6B  # I2C address for LSM6DS3 IMU
        self.I2C_FREQ = 400000  # Frequency for I2C communication
        self.WHO_AM_I_REG = 0x0F
        self.CTRL1_XL 	= 0x10
        self.CTRL2_G 	= 0x11
        self.OUTX_L_G 	= 0x22  # Gyro X-axis data register (Low byte)
        self.OUTX_L_XL 	= 0x28  # Accel X-axis data register (Low byte)
        
        #
        self.IMU_CALIB_SAMPLES = 200
        
        # Motor PWM pin configuration
        self.MOTOR_PINS = {
            "front_right"	: 20,  	# Motor 3
            "rear_right"	: 16,   # Motor 1
            "front_left"	: 11,   # Motor 2
            "rear_left"		: 15    # Motor 4
        }

        # Kill switch configuration
        self.KILL_SWITCH_PIN = 28 

        # Heartbeat configuration
        self.HEARTBEAT_PIN = 25  # Onboard LED pin (example)
        
        # Heartbeat frequency per states
        self.HEARTBEAT_FREQ = {
            'DISARMED' 	:	0.5,
            'ARMED'		: 	1.0,
            'FLYING'   	:	2.0,
            'CRASHED'	:	4.0
            }
        
        self.MIN_MOTOR_DUTY = 0.0
        self.MAX_MOTOR_DUTY = 1.0

        # IMU filtering
        self.COMPLEMENTARY_FILTER_ALPHA = 0.98

        # Crash detection        
        self.ACCEL_THRESHOLD = 15.0       # G-force threshold for crash detection
        self.GYRO_THRESHOLD = 400.0      # Angular velocity threshold in degrees/sec
        self.ROLL_THRESHOLD = 60.0       # Roll threshold in degrees for crash detection
        self.PITCH_THRESHOLD = 60.0      # Pitch threshold in degrees for crash detection

        
class QuadcopterState:
    """Dynamic state data for the quadcopter."""
    def __init__(self):
        # Kill switch state
        self.kill_switch_is_active = True

        # Flight mode ('disarmed', 'armed', 'flying', 'crashed')
        self.flight_mode = 'DISARMED'
        self.flight_sequence_started = False
        self.flight_sequence_done = False

        # Crash detected
        self.crash_detected = False

        # IMU data placeholders
        self.imu_data = {
            'acceleration'	: {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyroscope'		: {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        
        self.imu_calib = {
            'accel_offset'	: {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro_drift'	: {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'calibrated?'	: False
        }
        

        # Orientation in degrees
        self.roll 	= 0.0   # Rotation around X-axis
        self.pitch 	= 0.0   # Rotation around Y-axis
        self.yaw 	= 0.0   # Rotation around Z-axis

        # Target orientation in degrees (setpoints)
        self.target_roll 	= 0.0
        self.target_pitch 	= 0.0
        self.target_yaw 	= 0.0

        # Motor speeds (0 to 100%)
        self.motor_speeds = {
            "front_right": 0.0,
            "rear_right": 0.0,
            "front_left": 0.0,
            "rear_left": 0.0
        }
        
        self.corrections = [0.0, 0.0, 0.0]
