from machine import Pin, Timer
import time

# Initialize the onboard LED (usually on GPIO 25)
led = Pin(25, Pin.OUT)

# Pulse parameters
pulse_period = 1.0  # Time (seconds) for a full pulse cycle (on + off)
pulse_duty = 0.5    # Fraction of time the LED is on (50%)

# Timer to handle the pulsing LED
timer = Timer()

# Heartbeat function
def heartbeat(timer):
    current_time = time.ticks_ms() / 1000  # Current time in seconds
    phase = current_time % pulse_period
    if phase < pulse_period * pulse_duty:
        led.on()
    else:
        led.off()

# Start the pulsing LED
timer.init(freq=10, mode=Timer.PERIODIC, callback=heartbeat)

# Print status messages
print("Raspberry Pi Pico setup complete!")
print("Onboard LED is pulsing like a heartbeat.")
print("Check REPL for additional status updates.")

# Main loop to simulate periodic system checks
try:
    while True:
        print("System running... Time:", time.ticks_ms() // 1000, "seconds")
        time.sleep(5)
except KeyboardInterrupt:
    print("Program interrupted by user. Cleaning up...")
    timer.deinit()
    led.off()
