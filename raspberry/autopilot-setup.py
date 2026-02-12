import time
from pymavlink import mavutil

class RocketMaster:
    def __init__(self, device='/dev/serial0', baud=921600):
        self.master = mavutil.mavlink_connection(device, baud=baud)
        print("[*] Connecting to Kakute H7...")
        self.master.wait_heartbeat()
        print(f"[+] System {self.master.target_system} Online.")

    def set_param(self, name, value, p_type):
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            name.encode('utf-8'), value, p_type
        )
        time.sleep(0.05)

    def get_param(self, name):
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode('utf-8'), -1
        )
        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        return msg.param_value if msg else None

    def full_setup_and_arm(self):
        # 1. Critical Boot-Level Check
        scr_state = self.get_param("SCR_ENABLE")
        if scr_state != 1.0:
            print("[!] Scripting engine not initialized. Configuring and Rebooting...")
            self.set_param("SCR_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            self.set_param("SCR_HEAP_SIZE", 65536, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            # Mandatory Reboot for memory allocation
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0
            )
            print("[!] Rebooting... Wait 10s and run this script again.")
            return

        # 2. Hardware Mapping (M6-M11 set to Script1)
        print("[*] Mapping Physical Pins M6-M11 to Script1...")
        for i in range(5, 11):
            self.set_param(f"SERVO{i}_FUNCTION", 94, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # 3. Virtual Pipes (12-14 mapped to Autopilot Functions)
        print("[*] Mapping Virtual Pipes (12=Ail, 13=Elev, 14=Rud)...")
        self.set_param("SERVO12_FUNCTION", 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)  # Aileron
        self.set_param("SERVO13_FUNCTION", 19, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # Elevator
        self.set_param("SERVO14_FUNCTION", 21, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # Rudder

        # 4. Global Safety & Mode Config
        self.set_param("BRD_SAFETYENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        self.set_param("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        # Set Mode to ACRO
        acro_id = self.master.mode_mapping()['ACRO']
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, acro_id)

        # 5. Final Force-Arm
        print("[*] Sending Force-Arm (21196)...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
        )
        print("[!!!] DONE. Script 1 is now in control of M6-M11.")

if __name__ == "__main__":
    rocket = RocketMaster()
    rocket.full_setup_and_arm()
