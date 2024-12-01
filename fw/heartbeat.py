# heartbeat.py

import uasyncio as asyncio
from machine import Pin

class Heartbeat:
    def __init__(self, model):
        """
        Initialize the Heartbeat module.

        :param config: QuadcopterConfig instance containing static configuration.
        :param state: QuadcopterState instance containing dynamic state data.
        """
        self.led_pin = Pin(model.config.HEARTBEAT_PIN, Pin.OUT)
        self.model = model

    async def blink(self):
        """
        Asynchronous task that controls the LED blinking with a 10% duty cycle.
        """
        while True:
            # Get the heartbeat freqeuncy based on current state
            frequency = self.model.config.HEARTBEAT_FREQ[self.model.data.flight_mode]

            if frequency > 0:
                # Calculate the period (T = 1 / f)
                period = 1.0 / frequency

                # Calculate on and off durations based on 50ms on-period
                on_duration = 0.05   
                off_duration = period - on_duration 
            else:
                # If frequency is zero or negative, keep the LED off
                on_duration = 0
                off_duration = 1.0  # Wait for 1 second before checking again

            # Turn LED on if on_duration is greater than zero
            if on_duration > 0:
                self.led_pin.on()
                await asyncio.sleep(on_duration)
            else:
                self.led_pin.off()

            # Turn LED off
            self.led_pin.off()
            await asyncio.sleep(off_duration)


if __name__ == '__main__':
    from quadcopter_model import QuadcopterModel
    
    # Instantiate the quadcopter model
    quad_model = QuadcopterModel()

    # Create an instance of Heartbeat with the quadcopter's configuration and state
    heartbeat = Heartbeat(quad_model)

    async def main():
        # Start the heartbeat blinking
        await heartbeat.blink()

    # Run the asyncio event loop
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        print("Heartbeat stopped by user")
