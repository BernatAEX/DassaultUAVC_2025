from dronekit import connect
import time
import pigpio

# Connect to the drone
vehicle = connect('/dev/ttyAMA0', wait_ready=True)
# We may need to change the connection for the drone

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon")
    exit()

ROLL_SERVO_PIN = 17
PITCH_SERVO_PIN = 18
YAW_SERVO_PIN = 27

# Servo angle ranges
SERVO_MIN_PULSE = 1000  # Min pulse width in µs (adjust based on your servo)
SERVO_MAX_PULSE = 2000  # Max pulse width in µs (adjust based on your servo)
SERVO_NEUTRAL = (SERVO_MIN_PULSE + SERVO_MAX_PULSE) // 2


def angle_to_pulse(angle, min_pulse=SERVO_MIN_PULSE,
                   max_pulse=SERVO_MAX_PULSE):
    # Converts an angle (-45 to +45 degrees) to servo pulse width
    return int(SERVO_NEUTRAL + (angle / 45.0) * ((max_pulse - min_pulse) / 2))


def stabilize_camera():
    while True:

        pitch = vehicle.attitude.pitch
        roll = vehicle.attitude.roll
        yaw = vehicle.attitude.yaw

        # Convert radians to degrees
        pitch_deg = pitch * (180.0 / 3.14159)
        roll_deg = roll * (180.0 / 3.14159)
        yaw_deg = yaw * (180.0 / 3.14159)

        pitch_pulse = angle_to_pulse(-pitch_deg)
        roll_pulse = angle_to_pulse(-roll_deg)
        yaw_pulse = angle_to_pulse(yaw_deg)

        # Move the servos
        pi.set_servo_pulsewidth(PITCH_SERVO_PIN, pitch_pulse)
        pi.set_servo_pulsewidth(ROLL_SERVO_PIN, roll_pulse)
        pi.set_servo_pulsewidth(YAW_SERVO_PIN, yaw_pulse)

        time.sleep(0.05)


try:
    stabilize_camera()
except KeyboardInterrupt:
    print("Stabilization stopped by user.")
finally:
    # Cleanup
    vehicle.close()
    pi.set_servo_pulsewidth(PITCH_SERVO_PIN, 0)
    pi.set_servo_pulsewidth(ROLL_SERVO_PIN, 0)
    pi.set_servo_pulsewidth(YAW_SERVO_PIN, 0)
    pi.stop()
    print("Servos stopped and resources cleaned up.")
