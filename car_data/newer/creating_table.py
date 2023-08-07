# from simple_pid import PID
import matplotlib.pyplot as plt
import numpy as np
import math
<<<<<<< Updated upstream
# import control
=======
>>>>>>> Stashed changes
import time
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from scipy.signal import butter, filtfilt
import bottleneck as bn


scatterpoints = []
m = 300 #kg double check later 
gain = 1 #change later to something appropriate
pwms = [80, 85, 90, 95, 100, 110, 115, 125, 130]
endpoints = [
    [133, 280],
    [369, 560],
    [65, 207],
    [255, 337],
    [176, 260],
    [1023, 1130],
    [80, 187],
    [159, 218],
    [55, 110]
]
for k,pwm in enumerate(pwms):
    odom_time = []
    pos = []
    with open(f'/home/vrl/FYP_agvpotholes/pothole_detection_avoidance_agv/car_data/newer/pwm_{pwm}', 'r') as f:
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

    window_size = 20

    # Define the moving average filter
    weights = np.repeat(1.0, window_size) / window_size
    ma_vel = np.convolve(total_vel, weights)

    accel = (ma_vel[1:-window_size+1] - ma_vel[:-window_size])/(odom_time[2:] - odom_time[1:-1])
    ma_accel = np.convolve(accel, weights)

    # plt.figure(1)
    # # print(pwm)
    # # # plt.plot(odom_time,pos[:,0])
    # plt.plot(total_vel)
    # plt.plot(ma_vel)
    # # # plt.plot(accel)
    # # plt.plot(ma_accel)
    # plt.show()

    scatterpoints.append(np.stack([ma_vel[endpoints[k][0]:endpoints[k][1]], ma_accel[endpoints[k][0]:endpoints[k][1]], np.full(endpoints[k][1]-endpoints[k][0], pwm)], axis=1))
    
scatterpoints = np.vstack(scatterpoints)

## Regression
X = scatterpoints[:,:2]
Y = scatterpoints[:, 2]
poly = PolynomialFeatures(degree=2, include_bias=False)
X_poly = poly.fit_transform(X)


model = LinearRegression()
model.fit(X_poly, Y)
coef = model.coef_
intercept = model.intercept_
powers = poly.powers_

print(f'PWM = {coef} {intercept}')

# PWM = 3.24723234*vel + 10.54925882*accel + 80.51997208398443

x = np.linspace(X[:,0].min(), X[:,0].max(), 10)
y = np.linspace(X[:,1].min(), X[:,1].max(), 10)
x, y = np.meshgrid(x, y)
z = coef[0]*x + coef[1]*y + intercept \
    + coef[2]*x*x + coef[3]*x*y + coef[4]*y*y \
# PWM = 18.1655985*vel + 0.63604731*ax - 1.35068359*vel**2  - 0.80396293vel*ax + 1.00006849*ax**2
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(scatterpoints[:,0], scatterpoints[:,1], scatterpoints[:,2])
ax.plot_surface(x, y, z, alpha=0.5)
ax.set_xlabel('Velocity')
ax.set_ylabel('Acceleration')
ax.set_zlabel('PWM')
plt.show()

