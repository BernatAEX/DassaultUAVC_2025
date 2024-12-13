from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from shapely.geometry import Point, Polygon  # Para verificar si un punto está dentro del polígono

# Conexión al vehículo
vehicle = connect("127.0.0.1:14550", wait_ready=True)  # Cambia la dirección a la de tu dron

# Geocerca en forma de polígono (lista de vértices en latitud y longitud)
GEOFENCE_VERTICES = [
    (-35.362938, 149.164832), (-35.363327, 149.166054), 
    (-35.364073, 149.165660), (-35.363685, 149.164438)
]
geofence_polygon = Polygon(GEOFENCE_VERTICES)

# Parámetros de vuelo y altitudes de maniobra
TARGET_ALTITUDE = 100  # Altura para detección en metros
DROP_ALTITUDE = 50  # Altura para liberar la carga en metros

def is_within_geofence(location):
    """
    Verifica si una ubicación está dentro del polígono de geocerca.
    """
    point = Point(location.lon, location.lat)
    return geofence_polygon.contains(point)

def simulate_teardrop(start_location, heading, drop_altitude):
    """
    Simula la maniobra de teardrop para verificar si se mantiene dentro de la geocerca.
    """
    simulation_heading = heading
    position = start_location

    # Simula la primera mitad del "teardrop" en dirección contraria
    for _ in range(10):  # Ajusta para más precisión de simulación
        # Ajusta posición simulada, velocidad y dirección
        simulation_heading = (simulation_heading + 180) % 360
        new_lat = position.lat + 0.00001 * math.cos(math.radians(simulation_heading))
        new_lon = position.lon + 0.00001 * math.sin(math.radians(simulation_heading))
        position = LocationGlobalRelative(new_lat, new_lon, drop_altitude)

        # Si sale del polígono, devuelve False
        if not is_within_geofence(position):
            return False
    return True

def find_teardrop_heading():
    """
    Encuentra una dirección de teardrop que mantenga al dron dentro de la geocerca.
    """
    current_heading = vehicle.heading
    start_location = vehicle.location.global_relative_frame

    for _ in range(4):  # Prueba cuatro direcciones diferentes, rotando 90 grados cada vez
        if simulate_teardrop(start_location, current_heading, DROP_ALTITUDE):
            return current_heading  # Retorna el heading adecuado
        current_heading = (current_heading + 90) % 360  # Rota 90 grados

    print("No se encontró un rumbo válido para el teardrop dentro de la geocerca.")
    return None

def descend_to_altitude(altitude):
    """
    Cambia la altitud del dron a una altitud objetivo.
    """
    vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_relative_frame.lat,
                                               vehicle.location.global_relative_frame.lon,
                                               altitude))
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if abs(current_altitude - altitude) < 1:
            break
        time.sleep(1)

def perform_teardrop(heading):
    """
    Realiza la maniobra de teardrop en la dirección especificada y verifica que se mantiene dentro de la geocerca.
    """
    print("Iniciando maniobra en teardrop...")
    
    vehicle.mode = VehicleMode("GUIDED")
    new_heading = (heading + 180) % 360  # Gira en dirección contraria

    # Configuración de velocidad y altitud durante la maniobra
    descend_to_altitude(DROP_ALTITUDE)
    
    # Primer paso del giro
    for _ in range(10):  # Ajusta el número de pasos para controlar el giro
        if not is_within_geofence(vehicle.location.global_relative_frame):
            print("Saliendo de la geocerca, maniobra abortada.")
            return
        
        # Ajusta la orientación y la posición
        vehicle.simple_goto(vehicle.location.global_relative_frame, groundspeed=10)
        time.sleep(1)
    
    print("Teardrop completado, regresando al punto inicial con AoA 0.")
    
    # Finaliza la maniobra ajustando el Ángulo de Ataque (AoA) a 0
    set_aoa(0)

def set_aoa(angle_of_attack):
    """
    Configura el Ángulo de Ataque del dron.
    """
    print(f"Setting Angle of Attack (AoA) to {angle_of_attack} degrees.")
    # Implementación específica para ajustar el AoA, si es aplicable.

def release_payload():
    """
    Suelta el payload en la posición actual.
    """
    print("Releasing payload...")
    # Código para activar el mecanismo de liberación de carga útil

def main():
    # Asegúrate de estar en modo GUIDED
    if vehicle.mode != VehicleMode("GUIDED"):
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    
    # Despegue hasta la altitud de detección
    vehicle.simple_takeoff(TARGET_ALTITUDE)
    while vehicle.location.global_relative_frame.alt < TARGET_ALTITUDE * 0.95:
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt}")
        time.sleep(1)
    
    # Espera a que se realice la detección
    # --- Aquí iría la llamada a la función de detección ---
    # Ejemplo: if detect_target(): ...

    # Si la detección es exitosa, desciende y encuentra el heading del teardrop
    descend_to_altitude(DROP_ALTITUDE)
    
    # Verifica la dirección segura para el teardrop
    heading = find_teardrop_heading()
    if heading is not None:
        perform_teardrop(heading)
        
    # Liberación de la carga útil en AoA 0
    # --- Aquí iría la llamada a la función de liberación ---
    # Ejemplo: release_payload()

# Ejecución del flujo principal
try:
    main()
finally:
    # Asegúrate de aterrizar el dron o cambiar a un modo seguro
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

"""
Coses que falten, comprovar si realment entrarà el dron o no alhroa de fer el 
"""


"""
1. Detection with CV
2. Tag drop (FWD Ballistic)
3. 


"""