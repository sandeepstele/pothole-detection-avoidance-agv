import matplotlib.pyplot as plt

# Open the file for reading
with open("/home/vrl/FYP_agvpotholes/pothole_detection_avoidance_agv/car_data/new/accel_100", "r") as f:
    # Initialize empty lists to store position and time data
    pos = []
    times = []

    # Loop over each line in the file
    for line in f:
        # Split the line into position and time data
        timetime, posit, time = line.strip().split(" ")

        # Convert the position and time data to floats and append to the lists
        pos.append(float(posit))
        times.append(float(time))

# Plot the position vs. time data
plt.plot(times, pos)
plt.xlabel("Time")
plt.ylabel("Position")
plt.show()