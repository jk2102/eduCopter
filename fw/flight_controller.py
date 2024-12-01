import uasyncio as asyncio
import time
from quadcopter_model import QuadcopterModel

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, setpoint, measurement, dt):
        # Calculate error
        error = setpoint - measurement

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Output
        output = proportional + integral + derivative
        return output

class FlightController:
    def __init__(self, model):
        self.model = model

        # PID controllers for roll, pitch, and yaw
        self.roll_pid 	= PIDController(kp=0.01, ki=0.00, kd=0.000)
        self.pitch_pid 	= PIDController(kp=0.01, ki=0.00, kd=0.000)
        self.yaw_pid 	= PIDController(kp=0.001, ki=0.00, kd=0.000)

        # Base motor speed for hovering
        self.base_throttle = 0.0
                
        self.last_update = time.ticks_ms() / 1000.0

    async def control_loop(self):
        while True:
            # Handle flight mode
            if self.model.data.flight_mode == 'CRASHED':
                self.model.data.flight_mode == 'CRASHED'
            elif (self.model.data.flight_mode == 'FLYING') and self.model.data.crash_detected:
                self.model.data.flight_mode = 'CRASHED'
            elif self.model.data.flight_sequence_done:
                self.model.data.flight_mode = 'DISARMED'                
            elif not self.model.data.kill_switch_is_active:
                self.model.data.flight_mode = 'ARMED'
                if self.model.data.flight_sequence_started and self.model.data.imu_calib['calibrated?']:
                    self.model.data.flight_mode = 'FLYING'                    
            else:
                self.model.data.flight_mode = 'DISARMED'

            # Handle PID  motor control
            if self.model.data.flight_mode == 'FLYING':
                # Time delta calculation
                current_time = time.ticks_ms()
                dt = time.ticks_diff(current_time, self.last_update) / 1000.0
                self.last_update = current_time
                
                if dt >= 0.05 or dt <= 0:
                    continue

                # Get current orientation from IMU data
                roll 	= self.model.data.roll
                pitch 	= self.model.data.pitch
                yaw 	= self.model.data.yaw

                # Calculate PID outputs for roll, pitch, and yaw
                roll_output 	= self.roll_pid.update(self.model.data.target_roll, roll, dt)
                pitch_output 	= self.pitch_pid.update(self.model.data.target_pitch, pitch, dt)
                yaw_output 		= self.yaw_pid.update(self.model.data.target_yaw, yaw, dt)
                
                # print(roll_output,  pitch_output, yaw_output)
                self.model.data.corrections = [f"""{self.model.data.target_roll-roll} -> {roll_output}, {self.model.data.target_pitch- pitch} -> {pitch_output}, {self.model.data.target_yaw- yaw} -> {yaw_output}"""]
                # Motor mixing to adjust each motor's speed
                self.model.data.motor_speeds["front_right"] 	= self.base_throttle + roll_output + pitch_output - yaw_output
                self.model.data.motor_speeds["rear_right"] 		= self.base_throttle + roll_output - pitch_output + yaw_output
                self.model.data.motor_speeds["front_left"] 		= self.base_throttle - roll_output + pitch_output + yaw_output
                self.model.data.motor_speeds["rear_left"] 		= self.base_throttle - roll_output - pitch_output - yaw_output

#                 self.model.data.motor_speeds["front_right"] 	= self.base_throttle  + yaw_output
#                 self.model.data.motor_speeds["rear_right"] 		= self.base_throttle  - yaw_output
#                 self.model.data.motor_speeds["front_left"] 		= self.base_throttle  - yaw_output
#                 self.model.data.motor_speeds["rear_left"] 		= self.base_throttle  + yaw_output

                # Clamp motor speeds between 0 and 100
                for motor in self.model.data.motor_speeds:
                    self.model.data.motor_speeds[motor] = max(0.0, min(1.0, self.model.data.motor_speeds[motor]))
                    # print(motor,  self.model.data.motor_speeds[motor])

            # Delay to run control loop at a fixed rate (e.g., 50 Hz)
            await asyncio.sleep(0.02)

if __name__ == '__main__':
    from quadcopter_model import QuadcopterModel
    from imu_reader import IMUReader
    from heartbeat import Heartbeat
    from kill_switch import KillSwitchMonitor
    from motor_control import MotorController

    async def main():
        # Instantiate the quadcopter model
        quad_model = QuadcopterModel()

        # Create IMUReader, FlightController, and Heartbeat instances
        imu_reader 			= IMUReader(quad_model)
        flight_controller 	= FlightController(quad_model)
        heartbeat 			= Heartbeat(quad_model)
        kill_switch_monitor = KillSwitchMonitor(quad_model)
        motor_controller    = MotorController   (quad_model)

        # Start IMU reader, flight controller, and heartbeat tasks
        imu_task 			= asyncio.create_task(imu_reader.read_sensor())
        flight_control_task = asyncio.create_task(flight_controller.control_loop())
        heartbeat_task 		= asyncio.create_task(heartbeat.blink())
        kill_switch_task 	= asyncio.create_task(kill_switch_monitor.monitor())
        motor_control_task 	= asyncio.create_task(motor_controller.control_loop())
        
        print("All tasks running!")
        
        # Wait before arming to allow system to stabilize
        while quad_model.data.flight_mode == 'DISARMED':
            await asyncio.sleep(1)
            
        print("Quadcopter ARMED!")

        # Wait before arming to allow system to stabilize
        await asyncio.sleep(5)
        
        print("Starting flight sequence!")
        quad_model.data.flight_sequence_started = True

        try:
            # Takeoff sequence (5 seconds)
            print("Taking off...")
#             quad_model.data.target_pitch 	= quad_model.data.pitch
#             quad_model.data.target_roll 	= quad_model.data.roll
#             quad_model.data.target_yaw 		= quad_model.data.yaw
            quad_model.data.target_pitch 	= 0.0
            quad_model.data.target_roll 	= 0.0
            quad_model.data.target_yaw 		= 0.0
            
            print(f"Targeting: {quad_model.data.target_pitch}, {quad_model.data.target_roll}, {quad_model.data.target_yaw}.")
            
            # Throttle control logic with three phases: Ramp Up, Hold, and Ramp Down.
            max_throttle = 0.9  # Specify your max throttle value (e.g., 80%)
            ramp_duration = 3.0  # Duration for each phase in seconds
            steps = 50  # Number of steps for ramping phases

            # Ramp Up Phase: Increase throttle from 0 to max_throttle over `ramp_duration` seconds.
            for step in range(steps):
                flight_controller.base_throttle = (step / steps) * max_throttle
                print(quad_model.data.motor_speeds, quad_model.data.corrections)
                await asyncio.sleep(ramp_duration / steps)  # Wait for the calculated time per step

            print("Throttle reached maximum. Holding at max throttle.")

            # Hold Phase: Hold at max_throttle for `ramp_duration` seconds.
            flight_controller.base_throttle = max_throttle
            await asyncio.sleep(5.0)

            print("Holding complete. Reducing throttle.")

            # Ramp Down Phase: Decrease throttle from max_throttle to 0 over `ramp_duration` seconds.
            for step in range(steps):
                flight_controller.base_throttle = max_throttle * (1 - step / steps)
                print(flight_controller.base_throttle, quad_model.data.motor_speeds, quad_model.data.corrections)
                await asyncio.sleep(ramp_duration / steps)  # Wait for the calculated time per step

            # Ensure throttle is set to zero.
            flight_controller.base_throttle = 0.0
            print("Throttle reduced to zero.")
            # Disarm the system
            quad_model.data.flight_sequence_done = True
            print('Flight sequence done!')
            while True:
                await asyncio.sleep(1)

        except KeyboardInterrupt:
            # Graceful shutdown on Ctrl+C
            imu_task.cancel()
            flight_control_task.cancel()
            heartbeat_task.cancel()
            kill_switch_task.cancel()
            print("Flight control stopped by user.")

    # Run the asyncio event loop
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program terminated by user.")


