import numpy as np

# Define time step
dt = 0.1  # Time step in seconds

# Define the state transition matrix F
F = np.array([[1, dt],
              [0, 1]])

# Define the measurement matrix H (identity since we directly measure h and v)
H = np.array([[1, 0],
              [0, 1]])

# Process noise covariance Q (tune these values)
Q = np.array([[0.01, 0],
              [0, 0.01]])

# Measurement noise covariance R (tune these values based on sensor characteristics)
R = np.array([[0.1, 0],
              [0, 0.1]])

# Initial state (e.g., altitude = 0 m, velocity = 0 m/s)
x = np.array([0, 0])

# Initial error covariance matrix P
P = np.eye(2)

# Example sensor measurements (replace with real sensor data)
# For example, altitude and velocity at each time step:
sensor_readings = get_sensor_data()

# Kalman filter loop
for z in sensor_readings:
    z = np.array(z)  # Current measurement [altitude, velocity]

    # Prediction step
    x_pred = F @ x
    P_pred = F @ P @ F.T + Q

    # Update step
    y = z - H @ x_pred  # Measurement residual
    S = H @ P_pred @ H.T + R  # Residual covariance
    K = P_pred @ H.T @ np.linalg.inv(S)  # Kalman gain

    x = x_pred + K @ y  # Updated state estimate
    P = (np.eye(2) - K @ H) @ P_pred  # Updated error covariance

    # Print the filtered altitude and velocity
    print(f"Filtered altitude: {x[0]}, Filtered velocity: {x[1]}")
