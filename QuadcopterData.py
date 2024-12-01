class QuadcopterData:
    def __init__(self):
        # System Status
        self.state = "IDLE"  # Operational state: IDLE, ARMING, FLYING, DISARMING, ERROR
        self.flight_mode = "Manual"  # Flight mode: Manual, Stabilized, Altitude Hold, Autonomous
        self.error_flags = []  # List of active error flags

        # Sensor Data
        self.imu = {
            "roll": 0.0,  # Roll angle in degrees
            "pitch": 0.0,  # Pitch angle in degrees
            "yaw": 0.0,  # Yaw angle in degrees
            "gyro": [0.0, 0.0, 0.0],  # Angular velocities (rad/s)
            "accel": [0.0, 0.0, 0.0],  # Linear accelerations (m/s^2)
        }
        self.barometer = {"altitude": 0.0, "pressure": 0.0}  # Altitude (m), Pressure (Pa)
        self.gps = {
            "lat": 0.0,  # Latitude (degrees)
            "lon": 0.0,  # Longitude (degrees)
            "altitude": 0.0,  # Altitude (m)
            "speed": 0.0,  # Ground speed (m/s)
            "satellites": 0,  # Number of satellites
        }
        self.magnetometer = {
            "heading": 0.0,  # Heading in degrees
            "mag_field": [0.0, 0.0, 0.0],  # Magnetic field components (X, Y, Z)
        }

        # Motor Data
        self.motors = [0.0, 0.0, 0.0, 0.0]  # Motor duty cycles (0.0 to 1.0)
        self.motor_health = [True, True, True, True]  # Health status of each motor

        # Control Inputs
        self.rc_inputs = {
            "throttle": 0.0,  # Normalized throttle input (-1.0 to 1.0)
            "roll": 0.0,  # Normalized roll input (-1.0 to 1.0)
            "pitch": 0.0,  # Normalized pitch input (-1.0 to 1.0)
            "yaw": 0.0,  # Normalized yaw input (-1.0 to 1.0)
        }
        self.autonomous_commands = {
            "waypoints": [],  # List of waypoints (lat, lon, alt)
            "target_velocity": [0.0, 0.0, 0.0],  # Target velocity (X, Y, Z)
        }

        # Battery and Power System
        self.battery = {
            "voltage": 12.5,  # Battery voltage (V)
            "current": 0.0,  # Current draw (A)
            "capacity_remaining": 100.0,  # Remaining capacity (%)
        }

        # Environmental Data
        self.environment = {
            "wind_speed": 0.0,  # Wind speed (m/s)
            "wind_direction": 0.0,  # Wind direction (degrees)
            "temperature": 25.0,  # External temperature (°C)
            "air_density": 1.225,  # Air density (kg/m^3)
        }

        # Flight Metrics
        self.flight_metrics = {
            "time": 0.0,  # Time since takeoff (seconds)
            "distance": 0.0,  # Total distance traveled (m)
            "altitude_max": 0.0,  # Maximum altitude (m)
            "velocity_max": 0.0,  # Maximum velocity (m/s)
        }

        # Debugging and Diagnostics
        self.diagnostics = {
            "loop_time": 0.0,  # Time for the main loop to execute (ms)
            "cpu_usage": 0.0,  # CPU usage (%)
            "free_memory": 0.0,  # Free memory (KB)
            "error_counters": {},  # Error counters by type
        }

    def __str__(self):
        """Provide a readable string representation of the quadcopter's current state."""
        return (
            f"State: {self.state}, Flight Mode: {self.flight_mode}\n"
            f"IMU: Roll={self.imu['roll']:.2f}, Pitch={self.imu['pitch']:.2f}, Yaw={self.imu['yaw']:.2f}\n"
            f"Motors: {self.motors}\n"
            f"Battery: {self.battery['voltage']}V, {self.battery['capacity_remaining']}% remaining\n"
            f"Altitude: {self.barometer['altitude']}m, GPS: ({self.gps['lat']}, {self.gps['lon']})\n"
        )
