import matplotlib.pyplot as plt
import numpy as np
import math
import time
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


table = {}
scatterpoints = []
values = []
torques = []
m = 300 #kg double check later 
gain = 1 #change later to something appropriate
pwms = [80, 85, 90, 95, 100, 105, 115, 120, 125]
biases = [
    0.10524125139112583,
    0.1035383265770506,
    0.12927084412635304,
    0.09604255048383493,
    0.07029334552923683,
    0.15010266931029037,
    0.13800859794602732,
    0.15127199800894597,
    0.15447727671987377
]
start_indices = [
    10,
    11,
    7,
    11,
    7,
    23,
    1,
    6,
    27
]
# for k,pwm in enumerate(pwms):
IMU_time = []
accel_x = []

with open(f'/home/vrl/FYP_agvpotholes/pothole_detection_avoidance_agv/car_data/new/accel_80', 'r') as f:
    # Read each line
    for line in f:
        # Split the line into a list of strings
        values = line.split()
        IMU_time.append(float(values[1]))
        accel_x.append(float(values[2])*9.81)
accel_x = np.array(accel_x)
IMU_time = np.array(IMU_time)
vel = np.cumsum((accel_x[1:] + accel_x[:-1]) / 2 * (IMU_time[1:] - IMU_time[:-1]))[start_indices[0]:]
accel_x = accel_x[start_indices[0]:]
IMU_time = IMU_time[start_indices[0]:]
# bias = np.average(accel_x[0:40])
accel_x = accel_x - biases[0]
# for i in range(len(accel_x)):
#     if accel_x[i] > 0.03:
#         accel_x = accel_x[i:]
#         IMU_time = IMU_time[i:]
#         break
for i in range(len(accel_x)):
    if accel_x[i] < -0.15:
        accel_x = accel_x[:i]
        IMU_time = IMU_time[:i]
        vel = vel[:i]
        plt.figure(1)
        plt.plot(IMU_time[:i], vel)
        # print(pwm)
        plt.figure(2)
        plt.plot(IMU_time[:i], accel_x)
        plt.show()
        break
    # calculate the velocity using the trapezoidal rule
    
    # add initial velocity to the velocity array
    # vel = np.insert(vel, 0, 0)
    # accel_x = np.insert(accel_x, 0, 0)

    # velocities = list(vel)
#     torques = accel_x*9.81*gain + 0.01*9.81*m
#     # print (torques)
#     # Create a nested dictionary for the table
#     table[pwm] = np.stack([vel, accel_x], axis=0)
#     scatterpoints.append(np.stack([vel, torques, np.full(vel.shape, pwm)], axis=1))
    
# scatterpoints = np.vstack(scatterpoints)
# print(scatterpoints.shape)
# print(scatterpoints[1])

# ## Regression
# X = scatterpoints[:,:2]
# Y = scatterpoints[:, 2]
# poly = PolynomialFeatures(degree=1, include_bias=False)
# X_poly = poly.fit_transform(X)


# model = LinearRegression()
# model.fit(X_poly, Y)
# coef = model.coef_
# intercept = model.intercept_
# powers = poly.powers_

# print(f'PWM = {coef} {intercept}')

# # PWM = 3.24723234*vel + 10.54925882*accel + 80.51997208398443

# x = np.linspace(X[:,0].min(), X[:,0].max(), 10)
# y = np.linspace(X[:,1].min(), X[:,1].max(), 10)
# x, y = np.meshgrid(x, y)
# z = coef[0]*x + coef[1]*y + intercept \
#     # + coef[2]*x*x + coef[3]*x*y + coef[4]*y*y \
    

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.scatter(scatterpoints[:,0], scatterpoints[:,1], scatterpoints[:,2])
# ax.plot_surface(x, y, z, alpha=0.5)
# ax.set_xlabel('Velocity')
# ax.set_ylabel('Torque')
# ax.set_zlabel('PWM')
# plt.show()

## Controller design

k_p = 3
k_i= 0.25
k_d = 0
# tau = 0.5
# num_v = [k_p, k_i]
# den_v = [tau, 1, k_p, k_i]

# V_x from imu
# V_des from pure pursuit

V_des = 1
error_i = 0
error = []
for V_x in vel:
    error.append(V_des - V_x)
    if len(error)>=2:
        error_i += (error[-2]+error[-1])/ 2 * 0.01
    ax_des = k_p *(error[-1]) + k_i*(error_i)
    pwm_des = min(26.905878236162497*V_x + 10.85662832777007*ax_des + 74.15191487795666, 130)
    print([V_x, error[-1], ax_des, pwm_des])


# ax_des = k_p *(error[-1]) + k_i*(error_i)

# Torque = ax_des*9.81*gain + 0.01*9.81*m
# velocity =  some_Imu_measurement

# velocity = 2
# Torque = 26
# print(len(scatterpoints))
# for i in range(len(scatterpoints[:,0])):
#     if scatterpoints[i][0]== velocity and scatterpoints[i][1]==Torque:
#         des_PWM = scatterpoints[i][3]
#         break
# print(des_PWM)

# if scatterpoints[:,0] == velocity and scatterpoints[:,1]== Torque:
#     des_PWM = 
# print(des_PWM)


#But this assumes that the velocity needs to be dead on, so need to fix that
# for pwm, v in table.items():
#         if V_x in v and abs(v[V_x[i]]-Torque)<0.5:
#             print(pwm)
        
#         else:
#             print("no no no")
# print(type(v_out))

# print(len(y)) (776)

# # Plot the input and output signals
# import matplotlib.pyplot as plt
# plt.plot(t,v_out, label='Input')
# # plt.plot(t, y, label='Output')
# plt.legend()
# plt.show()
