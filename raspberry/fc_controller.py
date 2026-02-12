import socket
import time
import sys
from pymavlink import mavutil

INPUT_IP = "0.0.0.0"
INPUT_PORT = 9002
SERVO_MIN = 700
SERVO_MAX = 2200

# FC Connection
DEVICE = '/dev/serial0'
BAUD = 921600

print(f"[*] Connecting to FC on {DEVICE}...")
master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
master.wait_heartbeat()
print("[+] Connected to FC.")

def set_param(name, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.02)

def configure_and_arm():
    print("--- Configuring FC ---")
    set_param("MOT_PWM_TYPE", 0)
    set_param("RC_OVERRIDE_TIME", 0)
    set_param("BRD_SAFETYENABLE", 0)
    set_param("ARMING_CHECK", 0)
    set_param("SERVO_GPIO_MASK", 0)
    for i in range(1, 5):
        set_param(f"SERVO{i}_FUNCTION", 0)

    print("--- Arming ---")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
    )
    time.sleep(1)
    print("[+] Armed and Ready.")

# =============================
# Helpers
# =============================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize(values):
    max_mag = max(abs(v) for v in values)
    if max_mag > 1:
        return [v / max_mag for v in values]
    return values

def to_pwm(v):
    v = clamp(v, -1, 1)
    return int((v + 1) / 2 * (SERVO_MAX - SERVO_MIN) + SERVO_MIN)


def mix_xtail(p, y):
    # Pair A: Fins 1 & 3
    # Pair B: Fins 2 & 4
    pair_A_val = p + y
    pair_B_val = p - y

    # Applying the Flip to Pair A (Motors 1 & 3)
    pair_A_val = pair_A_val * -1

    norm = normalize([pair_A_val, pair_B_val])

    # Mapping back to the 4 motors
    # M1 & M3 use Pair A | M2 & M4 use Pair B
    return [norm[0], norm[1], norm[0], norm[1]]


configure_and_arm()

sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.bind((INPUT_IP, INPUT_PORT))

print(f"Listening UDP on {INPUT_PORT}...")
last_heartbeat = time.time()

while True:
    if time.time() - last_heartbeat > 0.5:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_heartbeat = time.time()

    data, addr = sock_in.recvfrom(1024)

    try:
        message = data.decode().strip()
        yaw, vertical, forward, mode = message.split(",")

        yaw = float(yaw)
        vertical = float(vertical)

        yaw = clamp(yaw, -1, 1)
        vertical = clamp(vertical, -1, 1)

        servo_norm = mix_xtail(vertical, yaw)
        servo_pwm = [to_pwm(v) for v in servo_norm]

        for i in range(4):
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                i + 1, servo_pwm[i], 0, 0, 0, 0, 0
            )

        print(f"IN: {message} | OUT PWM: {servo_pwm}")

    except Exception as e:
        print("Parse error:", e)
