import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

def generate_data(noise_level, gyro_drift, true_angle, dt=0.01, duration=10):
    time = np.arange(0, duration, dt)
    # True angle (sinusoidal movement for simplicity)
    true_angles = true_angle * np.sin(2 * np.pi * 0.1 * time)

    # Simulated accelerometer data with noise
    accel_data = true_angles + noise_level * np.random.randn(len(time))

    # Simulated gyroscope data with drift
    gyro_data = np.gradient(true_angles, dt) + gyro_drift * np.cumsum(np.ones(len(time)) * dt)

    return time, true_angles, accel_data, gyro_data

def complementary_filter(accel_data, gyro_data, alpha, dt):
    filtered_angle = 0
    filtered_angles = []
    for i in range(len(accel_data)):
        # Complementary filter equation
        filtered_angle = alpha * (filtered_angle + gyro_data[i] * dt) + (1 - alpha) * accel_data[i]
        filtered_angles.append(filtered_angle)
    return np.array(filtered_angles)

# Streamlit interface
st.title("Complementary Filter Demo for Quadcopter")

st.sidebar.header("Filter Parameters")
alpha = st.sidebar.slider("Alpha (Filter Weight)", 0.0, 1.0, 0.98, step=0.01)
noise_level = st.sidebar.slider("Accelerometer Noise Level", 0.0, 2.0, 0.2, step=0.1)
gyro_drift = st.sidebar.slider("Gyroscope Drift Level", 0.0, 10.0, 1.0, step=0.1)
true_angle = st.sidebar.slider("True Angle Amplitude", 0.0, 90.0, 45.0, step=5.0)
duration = st.sidebar.slider("Simulation Duration (s)", 5, 20, 10, step=1)

# Generate and filter data
dt = 0.01  # Time step
time, true_angles, accel_data, gyro_data = generate_data(noise_level, gyro_drift, true_angle, dt, duration)
filtered_angles = complementary_filter(accel_data, gyro_data, alpha, dt)

# Plotting
st.header("Data Visualization")
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Raw accelerometer data
ax[0].plot(time, accel_data, label="Accelerometer Data", color="blue", alpha=0.7)
ax[0].plot(time, true_angles, label="True Angle", color="green", linestyle="--")
ax[0].set_ylabel("Angle (deg)")
ax[0].set_title("Accelerometer Data")
ax[0].legend()

# Raw gyroscope data
ax[1].plot(time, gyro_data, label="Gyroscope Data (Rate)", color="orange", alpha=0.7)
ax[1].set_ylabel("Rate (deg/s)")
ax[1].set_title("Gyroscope Data")
ax[1].legend()

# Filtered data
ax[2].plot(time, filtered_angles, label="Filtered Angle", color="red", alpha=0.7)
ax[2].plot(time, true_angles, label="True Angle", color="green", linestyle="--")
ax[2].set_ylabel("Angle (deg)")
ax[2].set_title("Filtered Output")
ax[2].set_xlabel("Time (s)")
ax[2].legend()

st.pyplot(fig)

# Explanation
st.subheader("What is a Complementary Filter?")
st.markdown("""
A complementary filter is a simple way to combine noisy accelerometer data and drift-prone gyroscope data to estimate orientation. 

**Alpha (\u03B1)** is the weight that determines how much we trust the gyroscope vs. the accelerometer:
- High \u03B1 (\u03B1 → 1): Trust the gyroscope more (suitable for smooth data or high accelerometer noise).
- Low \u03B1 (\u03B1 → 0): Trust the accelerometer more (suitable for low noise but low drift scenarios).

**How It Works:**
- Gyroscope provides short-term orientation changes (angle rates).
- Accelerometer provides long-term stability (absolute angle).
- The complementary filter blends the two using the equation:
  \[
  \text{Filtered Angle} = \alpha (\text{Gyro Angle}) + (1 - \alpha) (\text{Accel Angle})
  \]
""")
