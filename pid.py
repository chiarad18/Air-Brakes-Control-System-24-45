import time
import math
import tdqm
from predictions.py import PD

# PID coefficients
Kp = 1.0
Ki = 0.1
Kd = 0.05

error = 0
integral = 0
previous_error = 0
flap_position = 0  # Start with flaps fully retracted
target_apogee = 5000  # target apogee in meters
dt = 0.1  # Time step in seconds

while rocket_in_flight:
    # Predict the apogee with Runge Kutta
    projected_apogee = predict_apogee()

    # Calculate error
    error = projected_apogee - target_apogee

    # PID calculations
    integral += error * dt
    derivative = (error - previous_error) / dt
    PID_output = Kp * error + Ki * integral + Kd * derivative

    # Adjust flap position based on PID output
    flap_position = clamp(flap_position + PID_output, 0, 100)  # 0 to 100% actuation

    # Set flap position to the servo
    set_servo_position(flap_position)
    # Update previous error
    previous_error = error

    time.sleep(dt)


