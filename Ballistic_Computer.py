import numpy as np
import time


def compute_landing_position(mass, Cd_horizontal, Cd_vertical, area,
                             initial_height, drone_speed, wind_speed,
                             heading, wind_direction, alpha):
    """
    Computes the landing position of a payload with a parachute released from
    a moving drone.

    Parameters:
    - mass: Mass of the payload in kg.
    - Cd_horizontal: Horizontal drag coefficient.
    - Cd_vertical: Vertical drag coefficient.
    - area: Frontal area of the parachute in m^2.
    - initial_height: Release height in meters.
    - drone_speed: Speed of the drone in m/s.
    - wind_speed: Speed of the wind in m/s.
    - heading: Heading of the drone at release in degrees.
    - wind_direction: Direction from which the wind blows in degrees.
    - alpha: Angle of attack of the drone at release in degrees.

    Returns:
    - (x, z): Final landing position in meters from the release point.
    """

    GRAVITY = 9.81  # m/s^2
    rho = 1.225  # kg/m^3

    alpha_rad = np.radians(alpha)
    heading_rad = np.radians(heading)
    wind_direction_rad = np.radians(wind_direction)

    initial_velocity_x = drone_speed * np.cos(alpha_rad) * np.cos(heading_rad)
    initial_velocity_y = drone_speed * np.sin(alpha_rad)
    initial_velocity_z = drone_speed * np.cos(alpha_rad) * np.sin(heading_rad)

    wind_velocity_x = wind_speed * np.cos(wind_direction_rad)
    wind_velocity_z = wind_speed * np.sin(wind_direction_rad)

    vx = initial_velocity_x + wind_velocity_x
    vy = initial_velocity_y
    vz = initial_velocity_z + wind_velocity_z

    def air_resistance_force(v, Cd, A, rho):
        return 0.5 * Cd * A * rho * v**2

    time = 0
    dt = 0.01
    x, y, z = 0, initial_height, 0

    while y > 0:
        time += dt
        v_total_horizontal = np.sqrt(vx**2 + vz**2)
        v_total_vertical = abs(vy)

        drag_force_horizontal = air_resistance_force(v_total_horizontal, Cd_horizontal, area, rho) if v_total_horizontal != 0 else 0
        drag_force_vertical = air_resistance_force(v_total_vertical, Cd_vertical, area, rho) if v_total_vertical != 0 else 0

        ax = - np.sign(vx) * drag_force_horizontal / mass
        az = - np.sign(vz) * drag_force_horizontal / mass
        ay = - GRAVITY - (np.sign(vy) * drag_force_vertical / mass)

        vx += ax * dt
        vy += ay * dt
        vz += az * dt

        x += vx * dt
        y += vy * dt
        z += vz * dt

    print("Fall time: " + str(round(time, 4)) + " seconds")

    return x, z


if __name__ == "__main__":
    mass = 0.5
    CdH = 0.5
    CdV = 2
    A = 0.1
    h = 20
    v = 12
    w = -5
    hdg = 0
    wdir = 8
    alpha = 0

    t0 = time.time()
    lan_x, lan_z = compute_landing_position(mass, CdH, CdV, A, h,
                                            v, w, hdg, wdir, alpha)

    print("Dx: " + str(lan_x))
    print("Dz: " + str(lan_z))
    print("Time elapsed for the computation : " + str(time.time()-t0)
          + " seconds")
