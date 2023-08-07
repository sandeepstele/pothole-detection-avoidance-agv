import matplotlib.pyplot as plt
import numpy as np
import math
import time
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

odom_time = []
pos = []
with open(f'/home/vrl/FYP_agvpotholes/pothole_detection_avoidance_agv/car_data/newer/pwm_130', 'r') as f:
    # Read each line
    for line in f:
        # Split the line into a list of strings
        values = line.split()
        odom_time.append(float(values[0]))
        pos.append([float(value) for value in values[1:]])
pos = np.array(pos)
odom_time = np.array(odom_time)
odom_time_reshaped = odom_time[:, np.newaxis]
vel = (pos[1:] - pos[:-1]) / (odom_time_reshaped[1:] - odom_time_reshaped[:-1])
total_vel = 2*np.sqrt(vel[:,1]**2+vel[:,0]**2)
window_size = 15

# Define the moving average filter
weights = np.repeat(1.0, window_size) / window_size
ma_vel = np.convolve(total_vel, weights)

k_p = 2
k_i= 0.25
k_d = 0

V_des = 4
error_i = 0
error = []
for V_x in ma_vel:
    error.append(V_des - V_x)
    if len(error)>=2:
        error_i += (error[-2]+error[-1])/ 2 * 0.01
    ax_des = k_p *(error[-1]) + k_i*(error_i)
    if ax_des>0:
        pwm_des = min(75.92156736664518+18.1655985*V_x + 0.63604731*ax_des - 1.35068359*V_x**2  - 0.80396293*V_x*ax_des + 1.00006849*ax_des**2, 13000)
    else:
        pwm_des = min(75.92156736664518 + 18.1655985*V_des - 1.35068359*V_des**2, 13000)
    print([V_x, error[-1], ax_des, pwm_des])