import uasyncio as asyncio
from machine import Pin, PWM
from quadcopter_model import QuadcopterModel

class MotorController:
    def __init__(self, model):
        """
        Initialize the MotorController module.

        :param model: QuadcopterModel instance containing static configuration and dynamic state data.
        """
        self.model = model
        self.pwm_pins = {
            motor: PWM(Pin(pin), freq=15000)  # PWM at 50 Hz for ESC control
            for motor, pin in model.config.MOTOR_PINS.items()
        }

    def set_motor_speed(self, motor_name, speed):
        """
        Set the speed of a specific motor.

        :param motor_name: The name of the motor to control (e.g., 'front_right').
        :param speed: Speed (0.0 to 1.0).
        """
        if motor_name not in self.pwm_pins:
            raise ValueError(f"Motor '{motor_name}' does not exist.")

        if not 0.0 <= speed <= 1.0:
            raise ValueError("Speed must be between 0.0 and 1.0.")

        # Map speed to duty cycle
        min_duty = self.model.config.MIN_MOTOR_DUTY
        max_duty = self.model.config.MAX_MOTOR_DUTY
        duty = min_duty + (max_duty - min_duty) * speed if self.model.data.flight_mode == "FLYING" else 0.0
        #print(motor_name,  int(duty * 65535))
        # Set PWM duty cycle for the motor
        self.pwm_pins[motor_name].duty_u16(int(duty * 65535))  # Normalize to 16-bit range for duty_u16()

    async def control_loop(self):
        """
        Asynchronous control loop to adjust motor speeds based on the quadcopter state.
        """
        try:
            while True:
                # Loop over each motor and set the speed as per the current state
                for motor_name, speed in self.model.data.motor_speeds.items():
                    if self.model.data.flight_mode == "FLYING":
                        self.set_motor_speed(motor_name, speed)
                    else:
                        # Safety feature, if disarmed or crashed, shut down the motors 
                        self.set_motor_speed(motor_name, 0.0)

                # Add a delay to make this loop execute at a fixed rate (e.g., 50 Hz)
                await asyncio.sleep(0.02)  # 50 Hz control update rate
                
        except asyncio.CancelledError:
            # Gracefully handle the motor shutdown if the task is cancelled
            print("MotorController task cancelled. Shutting down all motors.")
            for motor_name in self.model.data.motor_speeds.keys():
                self.set_motor_speed(motor_name, 0.0)

        finally:
            # Ensure all motors are completely stopped
            print("Final shutdown of all motors.")
            for motor_name in self.model.data.motor_speeds.keys():
                self.set_motor_speed(motor_name, 0.0)


if __name__ == '__main__':
    from heartbeat import Heartbeat
    from kill_switch import KillSwitchMonitor
    # Instantiate the quadcopter model
    quad_model = QuadcopterModel()
    
    # Max duty cycle for powering motors via USB
    quad_model.config.MAX_MOTOR_DUTY = 0.3

    # Create an instances
    motor_controller    = MotorController   (quad_model)
    kill_switch_monitor = KillSwitchMonitor	(quad_model)
    heartbeat 			= Heartbeat			(quad_model)
    
    async def main():
        try:
            # Start the tasks concurrently
            motor_control_task = asyncio.create_task(motor_controller.control_loop())
            kill_switch_task = asyncio.create_task(kill_switch_monitor.monitor())
            heartbeat_task = asyncio.create_task(heartbeat.blink())

            # Wait to be armed
            while quad_model.data.kill_switch_is_active:
                await asyncio.sleep(1.0)

            # Arm the system
            quad_model.data.flight_mode = 'FLYING'
            print('Flight mode changed to FLYING!')

            # Pause before test
            await asyncio.sleep(5)

            # Test each motor sequentially by updating the state
            for motor_name in quad_model.config.MOTOR_PINS.keys():
                print(f"Testing motor: {motor_name}")

                # Increase speed from 0 to 100%
                for speed in range(0, 101, 5):
                    quad_model.data.motor_speeds[motor_name] = speed / 100.0
                    await asyncio.sleep(0.25)  # Increase speed over 5 seconds (5% per 0.25s)

                # Hold at 100% speed for 5 seconds
                await asyncio.sleep(5)

                # Decrease speed from 100% to 0%
                for speed in range(100, -1, -5):
                    quad_model.data.motor_speeds[motor_name] = speed / 100.0
                    await asyncio.sleep(0.25)  # Decrease speed over 5 seconds (5% per 0.25s)

                # Ensure motor is completely stopped
                quad_model.data.motor_speeds[motor_name] = 0
                print(f"Finished testing motor: {motor_name}")
                
            print("Finished testing all motors")

            # Keep running other tasks
            while True:
                await asyncio.sleep(1)

        except asyncio.CancelledError:
            # Handle any cleanup here if needed
            print("Tasks were cancelled gracefully.")

    # Run the asyncio event loop for motor control and testing
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Motor testing and control stopped by user")
        print("Shutting down all motors!")
        for motor_name in quad_model.config.MOTOR_PINS.keys():
            quad_model.data.motor_speeds[motor_name] = 0.0

