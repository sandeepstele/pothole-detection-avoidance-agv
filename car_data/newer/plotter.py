import numpy as np
from matplotlib import pyplot as plt
import scipy

def resample_by_interpolation(signal, input_fs, output_fs):
    scale = output_fs / input_fs
    n = round(len(signal) * scale)

    resampled_signal = np.interp(
        np.linspace(0.0, 1.0, n, endpoint=False),  # where to interpret
        np.linspace(0.0, 1.0, len(signal), endpoint=False),  # known positions
        signal,  # known data points
    )
    return resampled_signal

odom_time = []
pos = []
with open(f'avoid_left_1686249386_odom', 'r') as f:
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

path_pos = []
path_timestamp = []
with open('avoid_left_1686249386_path', 'r') as f:
# Read each line
    for line in f:
        # Split the line into a list of strings
        values = line.split()
        path_pos.append([float(value) for value in values[1:3]])
        path_timestamp.append(values[0])

bounding_box = []
with open('avoid_left_1686249386_bb', 'r') as f:
# Read each line
    for line in f:
        # Split the line into a list of strings
        values = line.split()
        bounding_box.append([float(value) for value in values])

potholes = []
for bb in bounding_box:
    if abs(5-bb[0])< 1 and abs(bb[1])< 1 and abs(7.5-bb[3])< 1 and abs(0.5-bb[4])< 1:
        potholes.append([[bb[0], bb[1]], [bb[3], bb[4]]])

sum = np.array([0.,0.,0.,0.])
for bb in potholes:
    sum += np.array([bb[0][0], bb[0][1], bb[1][0], bb[1][1]])
pothole = (sum/len(potholes)).tolist()

point1 = pothole[0:2]
point2 = pothole[2:4]

square_x = [point1[0], point2[0], point2[0], point1[0], point1[0]]
square_y = [point1[1], point1[1], point2[1], point2[1], point1[1]]

path_pos = np.array(path_pos)
path_pos_list = []

for i, n in enumerate(path_pos[:,0]):
    if n > pos[-1, 0]:
        index = i+1
        break

new_path_pos_x = resample_by_interpolation(path_pos[:index,0], len(path_pos[:index,0]), len(pos[:,0]))
new_path_pos_y = resample_by_interpolation(path_pos[:index,1], len(path_pos[:index,1]), len(pos[:,1]))

error = []

for i ,x_pos in enumerate(pos[:,0]):
    min_distance = float('inf')
    y_pos = pos[i,1]
    for j ,x_path_pos in enumerate(new_path_pos_x):
        y_path_pos = new_path_pos_y[j]
        distance = np.sqrt((x_pos - x_path_pos)**2+(y_pos-y_path_pos)**2)
        if distance < min_distance:
            min_distance = distance
            min_index = j
    error.append(min_distance)

# print(pwm)

### plots path

# plt.plot([0,10], [0,0], linestyle='dashed', color = 'black')
# # plt.plot(path_pos[:index,0], path_pos[:index,1])
# plt.plot(new_path_pos_x, new_path_pos_y, color = 'blue')
# plt.plot(pos[:,0], pos[:,1], color = 'red')
# plt.plot(square_x, square_y, color = 'black')
# plt.axis([0,10, -2, 2])
# plt.legend(["Path Without Deviations", "Reference Trajectory", "Tracked Trajectory", "Pothole"])
# plt.title("Reference Trajectory versus Traced Trajectory")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")

###

### plots error

plt.plot(pos[:,0], error, color = 'black')
plt.title("Absolute Lateral Error At Every Longitudinal Position")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

###

### plots velocity

# plt.plot(total_vel)
# plt.plot(ma_vel)
# # plt.plot(accel)
# plt.plot(ma_accel)

###

plt.show()


# path = [0,0, 0.5,0,3,0.3, 5,0.6,7,0.6,9,0.3,11,0,12,0,13,0,14,0,15,0,17,0,19,0]
# x = [path[i] for i in range(0,len(path)-2,2)]
# y = [path[i] for i in range(1,len(path)-1,2)]
# # plt.plot(x,y)
# plt.show()

