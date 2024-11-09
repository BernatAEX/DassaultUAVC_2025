import numpy as np
import matplotlib.pyplot as plt

GRAVITY = 9.81
Cd_horizontal = 0.5
Cd_vertical = 2
rho = 1.225
m = 0.5
A = 0.1

drone_speed = 12.0
wind_speed = -5.0
alpha = 0.0
initial_height = 20.0


alpha_rad = np.radians(alpha)

initial_velocity_x = drone_speed * np.cos(alpha_rad) + wind_speed
initial_velocity_y = drone_speed * np.sin(alpha_rad)


def air_resistance_force(v, Cd, A, rho):
    return 0.5 * Cd * A * rho * v**2


def compute_accelerations(vx, vy):
    drag_force_x = air_resistance_force(vx, Cd_horizontal,
                                        A, rho) if vx != 0 else 0
    drag_force_y = air_resistance_force(vy, Cd_vertical,
                                        A, rho) if vy != 0 else 0

    ax = -np.sign(vx) * drag_force_x / m
    ay = -GRAVITY - (np.sign(vy) * drag_force_y / m)
    return ax, ay


dt = 0.01
x, y = 0, initial_height
vx, vy = initial_velocity_x, initial_velocity_y


trajectory_x = [x]
trajectory_y = [y]

while y > 0:
    ax, ay = compute_accelerations(vx, vy)

    vx += ax * dt
    vy += ay * dt

    x += vx * dt
    y += vy * dt
    trajectory_x.append(x)
    trajectory_y.append(y)

plt.plot(trajectory_x, trajectory_y, label="Payload Trajectory with Parachute")
plt.xlabel("Horizontal Distance (m)")
plt.ylabel("Height (m)")
plt.title("Payload Landing Trajectory with Vertical and Horizontal Drag")
plt.legend()
plt.grid()
plt.show()

total_distance = trajectory_x[-1]
fall_time = len(trajectory_x) * dt

print(f"Total distance reached by the payload: {total_distance:.2f} meters")
print(f"Fall time: {fall_time:.2f} seconds")
