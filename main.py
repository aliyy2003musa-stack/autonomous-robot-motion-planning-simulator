import numpy as np
import matplotlib.pyplot as plt

#----------------------------------******** PARAMETERS *********----------------------------

step_size = 0.04
threshold = 0.05
turn_speed = 0.15
obstacle_influence_range = 1.0

#___________________________________________********** INITIAL STATE **********____________________
initial_point = np.array([7.0, 9.0, 3.0])
theta = 0.0
path_x = []
path_y = []

target_points = np.array([[2, 8, 4],
                          [12, 5, 8],
                          [9, 4, 6]
                          ])

obstacle_point = np.array([6.2, 6.552, 9.0])

#___________________________________************* SIMULATION LOOP *____________________________________
for target_point in target_points:
    step_count = 0
    max_steps = 5000
    while True:
        step_count += 1
        if step_count >= max_steps:
            print("Hey! Maximum steps reached. Now moving to the next target point.")
            break

        displacement = target_point - initial_point
        obstacle_displ=initial_point-obstacle_point
        obstacle_distance=float(np.linalg.norm(obstacle_displ[:2]))
        distance = float(np.linalg.norm(displacement[:2]))
        angle = float(np.arctan2(displacement[1], displacement[0]))
        avoid_angle =float(np.arctan2(obstacle_displ[1], obstacle_displ[0]))
        desired_angle = angle - theta


        #_________________________*********** NORMALIZE ANGLE ERROR *****************_______________________________
        desired_angle = (desired_angle + np.pi) % (2 * np.pi) - np.pi
        obstacle_angle = (avoid_angle - theta + np.pi) % (2 * np.pi) - np.pi


        #_________________________************* STEERING LOGIC ****************__________________________
        if obstacle_distance < obstacle_influence_range:
            theta+=0.3*desired_angle + 0.7 * obstacle_angle
        else:
            theta += desired_angle*turn_speed


        #_____________________________*************** MOVEMENT OF THE ROBOT *****************_______________________________
        direction = np.array([np.cos(theta), np.sin(theta),  0.0])
        if distance > 0.05:
            step=min(step_size, distance * 0.5)
            initial_point += direction * step
            path_x.append(initial_point[0])
            path_y.append(initial_point[1])


        #___________________________***************** GOAL REACHED NOTIFICATION *****************______________________

        if distance <= threshold:
            print(f"Congratulation! You have reached the target point:  {target_point}")
            break
        print(f" You  are at a distance: x:{initial_point[0]:.2f}, y:{initial_point[1]:.2f}, z:{initial_point[2]:.2f}, with a magnitude: {obstacle_distance:.3f} meter from obstacle point {obstacle_point} and of magnitude: {distance:.3f} meter from the target point: {target_point} ")
plt.figure()

# path line
plt.plot(path_x, path_y, label="Robot Path")

# start point
plt.scatter(path_x[0], path_y[0], color='green', label="Start")

# end point
plt.scatter(path_x[-1], path_y[-1], color='red', label="End")

# obstacles
plt.scatter(obstacle_point[0], obstacle_point[1], color='black', marker='x', label="Obstacle")

# targets
for t in target_points:
    plt.scatter(t[0], t[1], marker='o', label="Target")

plt.title("Robot Motion Path Planning Simulation")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.grid()
plt.show()