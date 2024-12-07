import streamlit as st
import numpy as np
import pandas as pd
import altair as alt

# Simulate roll dynamics with a PID controller
class PIDSimulator:
    def __init__(self, kp, ki, kd, dt, damping, inertia):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.damping = damping
        self.inertia = inertia

        # PID state
        self.integral = 0
        self.prev_error = 0

        # System state
        self.roll_angle = 0
        self.roll_rate = 0

    def update(self, target_roll):
        error = target_roll - self.roll_angle
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        # PID control signal
        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update dynamics
        acceleration = control / self.inertia - self.damping * self.roll_rate
        self.roll_rate += acceleration * self.dt
        self.roll_angle += self.roll_rate * self.dt

        # Save current error for next derivative calculation
        self.prev_error = error

        return self.roll_angle, error, control


# Streamlit interface
st.title("PID Tuning for Quadcopter Roll")

st.sidebar.header("PID Parameters")
kp = st.sidebar.slider("Proportional Gain (Kp)", 0.0, 2.0, 1.0, step=0.01)
ki = st.sidebar.slider("Integral Gain (Ki)", 0.0, 2.0, 0.0, step=0.01)
kd = st.sidebar.slider("Derivative Gain (Kd)", 0.0, 2.0, 0.0, step=0.01)

st.sidebar.header("System Parameters")
damping = st.sidebar.slider("Damping Coefficient", 0.0, 1.0, 0.2, step=0.05)
inertia = st.sidebar.slider("Inertia", 0.1, 5.0, 1.0, step=0.1)
target_roll = st.sidebar.slider("Target Roll Angle (degrees)", -45.0, 45.0, 10.0, step=1.0)

# Simulation settings
duration = st.sidebar.slider("Simulation Duration (seconds)", 2, 20, 15, step=1)
dt = 0.01  # Time step
num_steps = int(duration / dt)

# Initialize simulator
simulator = PIDSimulator(kp, ki, kd, dt, damping, inertia)

# Run simulation
time = np.linspace(0, duration, num_steps)
roll_angles = []
errors = []
controls = []

for t in time:
    roll_angle, error, control = simulator.update(target_roll)
    roll_angles.append(roll_angle)
    errors.append(error)
    controls.append(control)

# Prepare data for plotting
data = pd.DataFrame({
    "Time (s)": time,
    "Roll Angle (deg)": roll_angles,
    "Error (deg)": errors,
    "Control Signal": controls,
    "Target Roll (deg)": [target_roll] * len(time),
})

# Visualization
st.header("Simulation Results")

st.subheader("Roll Angle")
roll_chart = alt.Chart(data).mark_line().encode(
    x="Time (s)",
    y=alt.Y("Roll Angle (deg)", title="Roll Angle (deg)"),
    tooltip=["Time (s)", "Roll Angle (deg)"]
).properties(title="Roll Angle Response")

target_chart = alt.Chart(data).mark_line(color="red", strokeDash=[5, 5]).encode(
    x="Time (s)",
    y=alt.Y("Target Roll (deg)", title="Target Roll (deg)"),
    tooltip=["Time (s)", "Target Roll (deg)"]
)

st.altair_chart(roll_chart + target_chart, use_container_width=True)

st.subheader("Error Signal")
st.line_chart(data[["Time (s)", "Error (deg)"]].set_index("Time (s)"))

st.subheader("Control Signal")
st.line_chart(data[["Time (s)", "Control Signal"]].set_index("Time (s)"))

# Explanation
st.subheader("What is PID Control?")
st.markdown("""
A PID controller is a feedback control system that uses three terms:
- **Proportional (P)**: Corrects the error based on its magnitude.
- **Integral (I)**: Corrects based on the accumulated past errors.
- **Derivative (D)**: Predicts and corrects based on the rate of error change.

The goal is to achieve a stable and responsive control system. By tuning **Kp**, **Ki**, and **Kd**, you can balance responsiveness, stability, and precision.

**Workshop Task**:
1. Adjust the PID parameters using the sliders.
2. Observe the roll angle's behavior in response to the target.
3. Aim for minimal overshoot, fast settling time, and no steady-state error.
""")
