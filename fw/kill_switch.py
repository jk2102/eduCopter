import uasyncio as asyncio
from machine import Pin
from quadcopter_model import QuadcopterModel

class KillSwitchMonitor:
    def __init__(self, model):
        """
        Initializes the KillSwitchMonitor.

        :param model: QuadcopterModel instance which includes config and state.
        """
        # Set up GP28 as an input with an internal pull-up resistor
        self.kill_switch_pin = Pin(model.config.KILL_SWITCH_PIN, Pin.IN, Pin.PULL_UP)
        self.model = model

    async def monitor(self):
        """
        Asynchronously monitors the kill switch status.
        """
        while True:
            # Read the value of the kill switch pin
            # When the kill switch is connected (closed), the pin reads low (0)
            # When the kill switch is disconnected (open), the pin reads high (1)
            is_active = self.kill_switch_pin.value() == 0  # Active when pin is low
            self.model.data.kill_switch_is_active = is_active
            await asyncio.sleep(0.1)  # Check every 100 ms

if __name__ == '__main__':
    from heartbeat import Heartbeat

    # Instantiate the quadcopter model
    quad_model = QuadcopterModel()

    # Create an instance of KillSwitchMonitor and Heartbeat
    kill_switch_monitor = KillSwitchMonitor	(quad_model)
    heartbeat 			= Heartbeat			(quad_model)

    async def main():
        # Start the KillSwitchMonitor and Heartbeat blink tasks concurrently
        kill_switch_task = asyncio.create_task(kill_switch_monitor.monitor())
        heartbeat_task = asyncio.create_task(heartbeat.blink())

        # Run a loop to monitor the kill switch and set heartbeat frequency accordingly
        while True:
            quad_model.data.flight_mode = 'ARMED' if not quad_model.data.kill_switch_is_active else 'DISARMED'
            print(f"Flight mode is {quad_model.data.flight_mode}!")
            # Add a short delay before re-checking kill switch state
            await asyncio.sleep(1.0)

        # Run both tasks indefinitely
        await asyncio.gather(kill_switch_task, heartbeat_task)

    # Run the asyncio event loop
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        print("Quadcopter system stopped by user")
        
