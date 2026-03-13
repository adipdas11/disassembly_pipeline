import sys
import machine
import time

# --- Configuration ---
# Original Servos
servo1 = machine.PWM(machine.Pin(14))
servo2 = machine.PWM(machine.Pin(15))
# New Grabber Servo on Pin 26
servo_grab = machine.PWM(machine.Pin(26))

# Set frequency for all
for s in [servo1, servo2, servo_grab]:
    s.freq(50)

# Constants for Screw Servos
SERVO1_MIN, SERVO1_MAX = 0, 10
SERVO2_MIN, SERVO2_MAX = 30, 40

# Constants for Grabber (Pin 26)
GRAB_ANGLE = 155
RELEASE_ANGLE = 0

# Defaults
default1 = SERVO1_MAX
default2 = SERVO2_MIN
default_grab = RELEASE_ANGLE

def set_servo_angle(servo, angle):
    """Maps 0-180 degrees to Pico duty cycle."""
    angle = max(0, min(180, angle))
    min_us, max_us = 500, 2500
    us = min_us + (max_us - min_us) * angle / 180
    duty = int(us * 65535 / 20000)
    servo.duty_u16(duty)

# --- Initialization ---
# Start at 0 for grabber and defaults for screw servos
set_servo_angle(servo1, default1)
set_servo_angle(servo2, default2)
set_servo_angle(servo_grab, default_grab)

while True:
    line = sys.stdin.readline()
    if not line:
        break
    
    cmd = line.strip()

    if cmd.lower() == 'q':
        break

    # Check for valid integer commands
    if cmd not in ('1', '0', '-1', '2', '3'):
        continue

    # Logic for Screw Servos (1, 0, -1)
    if cmd == '1':
        set_servo_angle(servo1, SERVO1_MIN)
        set_servo_angle(servo2, SERVO2_MIN)
    elif cmd == '0':
        set_servo_angle(servo1, default1)
        set_servo_angle(servo2, default2)
    elif cmd == '-1':
        set_servo_angle(servo1, SERVO1_MAX)
        set_servo_angle(servo2, SERVO2_MAX)
    
    # Logic for Grabber Servo (2, 3)
    elif cmd == '2':
        set_servo_angle(servo_grab, GRAB_ANGLE)
    elif cmd == '3':
        set_servo_angle(servo_grab, RELEASE_ANGLE)

    # Send success response
    sys.stdout.write("true\n")

# Reset to safe positions on exit
set_servo_angle(servo1, default1)
set_servo_angle(servo2, default2)
set_servo_angle(servo_grab, RELEASE_ANGLE)
