# main.py

import uasyncio as asyncio
from quadcopter_config import QuadcopterConfig
from quadcopter_state import QuadcopterState
from kill_switch import KillSwitchMonitor
from heartbeat import Heartbeat
#from motor_control import MotorController
from flight_controller import FlightController
# Import other modules as needed

# Initialize configuration and state
config = QuadcopterConfig()
state = QuadcopterState()

# Initialize modules
kill_switch_monitor = KillSwitchMonitor(config, state)
heartbeat = Heartbeat(config, state)
#motor_controller = MotorController(config, state)
flight_controller = FlightController(config, state)
# Initialize other modules as needed

async def main():
    # Create tasks
    kill_switch_task = asyncio.create_task(kill_switch_monitor.monitor())
    heartbeat_task = asyncio.create_task(heartbeat.blink())
    #motor_control_task = asyncio.create_task(motor_controller.control_loop())
    flight_control_task = asyncio.create_task(flight_controller.control_loop())
    # Create other tasks as needed

    # Run tasks concurrently
    await asyncio.gather(
        kill_switch_task,
        heartbeat_task,
        flight_control_task,
        # Include other tasks here
    )

# Run the main event loop
asyncio.run(main())
