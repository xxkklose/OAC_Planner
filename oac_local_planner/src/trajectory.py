import matplotlib.pyplot as plt

# Read trajectory.txt file
file_path = "/home/beihang705/catkin_mpc/src/OAC_Planner/oac_local_planner/src/trajectory.txt"
with open(file_path, "r") as file:
    lines = file.readlines()

# Initialize lists for storing data
time = []
yaw_rate = []
angle_path_edge = []
yaw = []
angle_to_goal = []

# Parse data from each line
for line in lines:
    data = line.strip().split()
    time.append(float(data[1]))
    yaw_rate.append(float(data[3]))
    angle_path_edge.append(float(data[5]))
    yaw.append(float(data[7]))
    angle_to_goal.append(float(data[9]))

# Plot four graphs
plt.figure(figsize=(10, 8))

plt.subplot(2, 2, 1)
plt.plot(time, yaw_rate)
plt.xlabel("Time")
plt.ylabel("Yaw Rate")
plt.title("Yaw Rate vs Time")

plt.subplot(2, 2, 2)
plt.plot(time, angle_path_edge)
plt.xlabel("Time")
plt.ylabel("Angle Path Edge")
plt.title("Angle Path Edge vs Time")

plt.subplot(2, 2, 3)
plt.plot(time, yaw)
plt.xlabel("Time")
plt.ylabel("Yaw")
plt.title("Yaw vs Time")

plt.subplot(2, 2, 4)
plt.plot(time, angle_to_goal)
plt.xlabel("Time")
plt.ylabel("Angle to Goal")
plt.title("Angle to Goal vs Time")

plt.tight_layout()
plt.show()
