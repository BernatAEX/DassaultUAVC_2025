import math
from dronekit import LocationGlobalRelative
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon

# Geocerca y parámetros de simulación
GEOFENCE_VERTICES = [
    (-35.362938, 149.164832), (-35.363327, 149.166054), 
    (-35.364073, 149.165660), (-35.363685, 149.164438)
]
geofence_polygon = Polygon(GEOFENCE_VERTICES)
TEARDROP_DISTANCE = 0.00005  # Distancia aproximada en grados

def calculate_new_position(start_location, heading, distance):
    """
    Calculate a new position based on a starting location, heading, and distance.
    """
    new_lat = start_location.lat + distance * math.cos(math.radians(heading))
    new_lon = start_location.lon + distance * math.sin(math.radians(heading))
    return LocationGlobalRelative(new_lat, new_lon, start_location.alt)

def simulate_teardrop_trajectory(start_location, heading):
    """
    Simulate the teardrop maneuver and return a list of trajectory points.
    """
    trajectory = [start_location]

    # Phase 1: Move forward in current heading direction
    forward_position = calculate_new_position(start_location, heading, TEARDROP_DISTANCE)
    trajectory.append(forward_position)

    # Phase 2: Reverse direction for the teardrop turn
    return_heading = (heading + 180) % 360  # Reverse heading
    turn_position = calculate_new_position(forward_position, return_heading, TEARDROP_DISTANCE)
    trajectory.append(turn_position)

    # Phase 3: Return to original vector direction in opposite sense
    final_position = calculate_new_position(turn_position, return_heading, TEARDROP_DISTANCE)
    trajectory.append(final_position)

    return trajectory

def plot_teardrop_trajectory(trajectory, geofence_polygon):
    """
    Plot the teardrop trajectory and geofence using matplotlib.
    """
    # Extract lat/lon from trajectory points
    latitudes = [point.lat for point in trajectory]
    longitudes = [point.lon for point in trajectory]

    # Plot the geofence
    x, y = geofence_polygon.exterior.xy
    plt.plot(x, y, color="blue", linewidth=1, label="Geofence Boundary")

    # Plot the teardrop trajectory
    plt.plot(longitudes, latitudes, color="red", marker="o", label="Teardrop Trajectory")
    
    # Mark start and end points
    plt.scatter(longitudes[0], latitudes[0], color="green", label="Start Position")
    plt.scatter(longitudes[-1], latitudes[-1], color="purple", label="End Position")

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Teardrop Maneuver Trajectory within Geofence")
    plt.legend()
    plt.grid(True)
    plt.show()

# Ejemplo de uso
start_location = LocationGlobalRelative(-35.362938, 149.164832, 100)  # Posición inicial simulada
heading = 45  # Heading inicial para el teardrop en grados

# Simula la trayectoria y visualiza el resultado
trajectory = simulate_teardrop_trajectory(start_location, heading)
plot_teardrop_trajectory(trajectory, geofence_polygon)
