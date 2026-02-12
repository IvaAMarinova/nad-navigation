import socket
import time
import sys
from pymavlink import mavutil



INPUT_IP = "0.0.0.0"
INPUT_PORT = 9002  

THROTTLE_OFF = 1000   
THROTTLE_MAX = 2000   

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
    print("--- Configuring M1 for DShot ESC ---")
    # 6 = DShot600 (Most common for H7 boards)
    # 4 = DShot150, 5 = DShot300
    set_param("MOT_PWM_TYPE", 6)       
    
    set_param("SERVO1_FUNCTION", 0)    # Manual Control
    set_param("RC_OVERRIDE_TIME", 0)   
    set_param("BRD_SAFETYENABLE", 0)   
    set_param("ARMING_CHECK", 0)       
    
    print("--- Arming ---")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
    )
    time.sleep(1)
    print("[+] Armed (DShot Protocol Active).")

def percent_to_pwm(percent):
    p = max(0.0, min(1.0, float(percent)))
    # Even in DShot mode, ArduPilot translates 1000-2000 MAVLink signals 
    # into the digital DShot frame.
    return int(THROTTLE_OFF + (p * (THROTTLE_MAX - THROTTLE_OFF)))


configure_and_arm()

sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.bind((INPUT_IP, INPUT_PORT))

print(f"Listening UDP on {INPUT_PORT} for DShot Commands...")
last_heartbeat = time.time()

while True:
    if time.time() - last_heartbeat > 0.5:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, 
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_heartbeat = time.time()

    data, addr = sock_in.recvfrom(1024)

    try:
        message = data.decode().strip()
        parts = message.split(",")
        
        if len(parts) == 4:
            forward_val = float(parts[2]) 
            mode = parts[3].upper()       

            target_val = THROTTLE_OFF 

            if mode == "FLY_STRAIGHT":
                target_val = percent_to_pwm(forward_val)
            elif mode == "STOP":
                target_val = THROTTLE_OFF
            
            # Sending to M1
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                1, target_val, 0, 0, 0, 0, 0
            )

            print(f"DSHOT MODE: {mode} | PWR: {forward_val*100:.1f}%")

    except Exception as e:
        print("Parse error:", e)
