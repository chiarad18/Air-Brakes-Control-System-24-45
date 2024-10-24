import numpy as np

# Define constants
g = 9.81  # Gravity (m/s^2)
rho = 1.225  # Air density at sea level (kg/m^3)
Cd = 0.5  # Drag coefficient (adjust based on flap position)
A_base = 0.01  # Base cross-sectional area (m^2)
A_flaps = 0.02  # Additional area when flaps are fully deployed (m^2)
m = 10  # Mass of the rocket (kg)
dt = 0.1  # Time step (s)

# Define drag force function
def drag(v, flap_position):
    A = A_base + (flap_position / 100) * A_flaps  # Adjust area based on flap position
    return 0.5 * rho * v**2 * Cd * A

# RK4 function for post-burnout
def rk4_step(t, h, v, flap_position):
    def f(t, h, v):
        F_drag = drag(v, flap_position)
        F_gravity = m * g
        dv_dt = -(F_drag + F_gravity) / m
        dh_dt = v
        return np.array([dh_dt, dv_dt])

    k1 = f(t, h, v)
    k2 = f(t + dt / 2, h + k1[0] * dt / 2, v + k1[1] * dt / 2)
    k3 = f(t + dt / 2, h + k2[0] * dt / 2, v + k2[1] * dt / 2)
    k4 = f(t + dt, h + k3[0] * dt, v + k3[1] * dt)

    # Update altitude and velocity using RK4 formula
    h_next = h + (dt / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    v_next = v + (dt / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])

    return h_next, v_next

# Initial conditions
t = 0
h = 0  # Initial altitude (m)
v = 300  # initial velocity (m/s) at the start of coast phase
flap_position = 0  # Start with flaps retracted

# Run the RK4 loop for prediction
while v > 0:  # While the rocket is ascending
    h, v = rk4_step(t, h, v, flap_position)
    t += dt


# At the end of loop, h should be close to predicted apogee
