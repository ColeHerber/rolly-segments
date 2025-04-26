import pygame
import dearpygui.dearpygui as dpg
import time
import json
# import threading # No longer needed
import math
from paho.mqtt import client as mqtt_client
import sys # To check platform

# --- Configuration ---
BROKER = 'rollyserver.local'
PORT = 1883
TOPIC_PREFIX = "rolly/cmd/"
CLIENT_ID = 'dpg_pygame_controller_mac_mainthread' # Updated client ID

# Controller Settings (Adjust as needed)
THROTTLE_AXIS = 0
STEERING_AXIS = 1
INVERT_THROTTLE = True
BUTTON_MODE_1 = 0
BUTTON_MODE_2 = 1
BUTTON_MODE_3 = 2
BUTTON_TOGGLE_ENABLE = 3
# BUTTON_EXIT_CONTROLLER_THREAD = 8 # No longer needed

DEADBAND = 0.4
SPEED_LIMITS = {
    1: 20.0,
    2: 40.0,
    3: 80.0,
}

# --- Global Variables ---
# No lock needed as everything is on the main thread
variables = {
    "target0": 0.0,
    "target1": 0.0,
    "vel_p": 0.025,
    "vel_i": 0.3,
    "vel_d": 0.0,
    "vel_lpf": 0.0,
}
input_ids = {}

# State variables
enable_flag = False
current_mode = 1
max_speed = SPEED_LIMITS[current_mode]
controller_target0 = 0.0 # Target calculated by controller
controller_target1 = 0.0 # Target calculated by controller
last_published_target0 = None # Last value sent to MQTT
last_published_target1 = None # Last value sent to MQTT

# MQTT and Pygame globals
mqtt_client_instance = None
joystick = None
pygame_initialized = False
joystick_initialized = False

# --- MQTT Functions ---
def connect_mqtt():
    """Connects to the MQTT broker."""
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("MQTT: Connected!")
        else:
            print(f"MQTT: Failed connection, code {reason_code}")

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, CLIENT_ID)
    client.on_connect = on_connect
    try:
        client.connect(BROKER, PORT)
        client.loop_start() # Start background thread for MQTT network I/O
        return client
    except Exception as e:
        print(f"MQTT: Connection error: {e}")
        return None

def publish(subtopic, payload_dict):
    """Publishes a JSON message to a specific subtopic."""
    global mqtt_client_instance
    if mqtt_client_instance and mqtt_client_instance.is_connected():
        topic = f"{TOPIC_PREFIX}"
        msg = json.dumps(payload_dict, separators=(',', ':'))
        # Optional: Add try-except around publish if network issues are frequent
        result = mqtt_client_instance.publish(topic, msg)
        status = result[0]
        # Reduce logging noise
        print("published:"+str(payload_dict))
        if status != 0 and status != mqtt_client.MQTT_ERR_QUEUE_SIZE:
             print(f"MQTT: Publish failed (rc={status}) for {topic}")
        time.sleep(0.1)

# --- Helper Functions ---
def apply_deadband(value, deadband_threshold):
    if abs(value) < deadband_threshold:
        return 0.0
    else:
        return value

def set_shared_state(new_mode=None, new_enable=None):
    """ Updates shared state variables (now safely called only from main thread). """
    global current_mode, max_speed, enable_flag, last_published_target0, last_published_target1
    # No lock needed

    changed = False
    if new_mode is not None and new_mode != current_mode and new_mode in SPEED_LIMITS:
        current_mode = new_mode
        max_speed = SPEED_LIMITS[current_mode]
        print(f"State: Set Mode {current_mode} (Max Speed: {max_speed})")
        changed = True

    if new_enable is not None and new_enable != enable_flag:
        enable_flag = new_enable
        print(f"State: System {'Enabled' if enable_flag else 'Disabled'}")
        changed = True
        # Publish enable/disable state immediately
        if enable_flag:
            publish("state", {"enable": True, "disable": False})
        else:
            # When disabling, send stop command
            publish("state", {"enable": False, "disable": True})
            publish("motors", {"target0": 0.0, "target1": 0.0})
            # Reset last published targets so stop command isn't filtered
            last_published_target0 = 0.0
            last_published_target1 = 0.0
    return changed

# --- DearPyGui Callbacks ---
def input_callback(sender, app_data, user_data):
    """Callback for DPG input fields."""
    key = user_data
    global last_published_target0, last_published_target1
    try:
        # Update the variable dictionary used by DPG
        new_value = float(app_data)
        variables[key] = new_value

        # Publish the manually entered value immediately via MQTT
        publish(key, {key: new_value}) # Assuming topic matches variable name for simplicity
        print(f"DPG: Manually set {key} to {new_value} and published.")

        # If target0 or target1 was set manually, update last published value
        if key == "target0":
             last_published_target0 = new_value
        elif key == "target1":
             last_published_target1 = new_value

    except ValueError:
        print(f"DPG: Invalid input for {key}: {app_data}")
        dpg.set_value(sender, str(variables[key])) # Revert GUI

def toggle_enable_dpg():
    """Callback for DPG 'Enable' button."""
    # Read current state and toggle
    set_shared_state(new_enable=not enable_flag)
    # Label update happens in update_gui_elements

# --- GUI Update Function ---
def update_gui_elements():
    """Updates DPG widgets based on current state (run in DPG's loop)."""
    # No lock needed, reading global state directly from main thread

    # Update Status Text
    status_text = f"Mode: {current_mode} | Controller Targets: L={controller_target0:.1f} R={controller_target1:.1f}"
    if dpg.does_item_exist("status_text"):
         dpg.set_value("status_text", status_text)

    # Update Enable Button Label
    if dpg.does_item_exist("enable_button"):
         dpg.set_item_label("enable_button", f"System Enabled: {enable_flag}")

    # Update Target0/Target1 input fields ONLY if enabled
    if enable_flag:
         if dpg.does_item_exist(input_ids["target0"]):
              dpg.set_value(input_ids["target0"], f"{controller_target0:.2f}")
         if dpg.does_item_exist(input_ids["target1"]):
              dpg.set_value(input_ids["target1"], f"{controller_target1:.2f}")
         # Optionally update variables dict too, though controller sets its own vars primarily
         variables["target0"] = controller_target0
         variables["target1"] = controller_target1

# --- Main Execution ---
if __name__ == "__main__":
    print(f"Running on platform: {sys.platform}")

    # --- Initialize MQTT ---
    mqtt_client_instance = connect_mqtt()
    if not mqtt_client_instance:
        print("FATAL: Could not connect to MQTT. Exiting.")
        exit()

    # --- Initialize DearPyGui ---
    dpg.create_context()
    dpg.create_viewport(title="Tank Control Panel (DPG + Pygame Main Thread)", width=700, height=400)

    # --- Initialize Pygame (on Main Thread) ---
    try:
        print("Initializing Pygame...")
        pygame.init()
        pygame.joystick.init()
        pygame_initialized = True
        print("Pygame initialized.")

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("Warning: No joystick detected by Pygame.")
            joystick_initialized = False
        else:
            try:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                print(f"Joystick Initialized: {joystick.get_name()}")
                joystick_initialized = True
            except pygame.error as e:
                 print(f"Error initializing joystick 0: {e}")
                 joystick_initialized = False

    except Exception as e:
        print(f"Error initializing Pygame: {e}")
        pygame_initialized = False # Ensure flag is false if init fails

    # --- Create DPG Window ---
    # Define window content before setting up and showing viewport
    with dpg.window(label="Control Panel", width=-1, height=-1) as main_window:
        dpg.add_text("Status:")
        dpg.add_text("Initializing...", tag="status_text")
        dpg.add_separator()

        dpg.add_text("Manual Parameter Control:")
        items_per_row = 3
        keys = list(variables.keys())

        for i in range(0, len(keys), items_per_row):
            row_keys = keys[i:i + items_per_row]
            with dpg.group(horizontal=True):
                for key in row_keys:
                    with dpg.group():
                        dpg.add_text(f"{key}")
                        input_id = dpg.add_input_text(
                            default_value=str(variables[key]),
                            callback=input_callback,
                            user_data=key,
                            width=120,
                            tag=f"input_{key}" # Unique tag for each input
                        )
                        input_ids[key] = input_id

        dpg.add_spacer(height=10)
        dpg.add_button(label="System Enabled: False", callback=toggle_enable_dpg, tag="enable_button", width = -1)
        dpg.add_button(label="Exit Application", callback=lambda: dpg.stop_dearpygui(), width = -1)


    # --- DPG Setup and Main Loop ---
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window(main_window, True)

    # Send initial state (disabled)
    set_shared_state(new_enable=False)

    # Pygame clock for potential frame limiting
    clock = pygame.time.Clock()

    print("Starting main loop...")
    # Main loop interleaving DPG rendering and Pygame polling
    while dpg.is_dearpygui_running():

        # --- Pygame Event Polling & Input Reading ---
        if pygame_initialized:
            try:
                # Process events like button presses
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN and joystick_initialized:
                        # Handle button presses for mode/enable
                        mode_change = None
                        enable_change = None
                        if event.button == BUTTON_MODE_1: mode_change = 1
                        elif event.button == BUTTON_MODE_2: mode_change = 2
                        elif event.button == BUTTON_MODE_3: mode_change = 3
                        elif event.button == BUTTON_TOGGLE_ENABLE: enable_change = not enable_flag

                        if mode_change is not None or enable_change is not None:
                            set_shared_state(new_mode=mode_change, new_enable=enable_change)
                    # Handle other Pygame events if needed (e.g., pygame.QUIT)
                    elif event.type == pygame.QUIT:
                         print("Pygame QUIT event received.")
                         dpg.stop_dearpygui() # Trigger DPG shutdown


                # Read Joystick Axes if initialized
                if joystick_initialized:
                    raw_throttle = joystick.get_axis(THROTTLE_AXIS)
                    raw_steering = joystick.get_axis(STEERING_AXIS)

                    if INVERT_THROTTLE: raw_throttle = -raw_throttle

                    throttle = apply_deadband(raw_throttle, DEADBAND)
                    steering = apply_deadband(raw_steering, DEADBAND)

                    left_mix = throttle + steering
                    right_mix = throttle - steering

                    max_mix = max(abs(left_mix), abs(right_mix))
                    if max_mix > 1.0:
                        left_mix /= max_mix
                        right_mix /= max_mix

                    # Calculate controller targets based on current mode's speed
                    ctl_t0 = round(left_mix * max_speed, 2)
                    ctl_t1 = round(right_mix * max_speed, 2)

                    # Update global controller targets (for GUI display)
                    controller_target0 = ctl_t0
                    controller_target1 = ctl_t1

                    # Publish Motor Commands via MQTT (if enabled and changed)
                    if enable_flag:
                        if ctl_t0 != last_published_target0 or ctl_t1 != last_published_target1:
                            publish("motors", {"target0": ctl_t0, "target1": ctl_t1})
                            last_published_target0 = ctl_t0
                            last_published_target1 = ctl_t1
                    # Stop command sent by set_shared_state when disabled

            except Exception as e:
                print(f"Error during Pygame processing: {e}")
                # Consider disabling Pygame part if errors persist
                # pygame_initialized = False
                # joystick_initialized = False


        # --- Update DPG GUI Elements ---
        update_gui_elements()

        # --- Render DPG Frame ---
        # This is the blocking call that waits for user input, etc.
        dpg.render_dearpygui_frame()

        # Optional: Limit frame rate slightly if needed, but DPG render might be sufficient
        # clock.tick(120) # Limit to 120 FPS max


    # --- Cleanup ---
    print("DPG: Exiting main loop...")

    if mqtt_client_instance:
        # Send final stop/disable commands
        print("MQTT: Sending final disable/stop commands.")
        set_shared_state(new_enable=False) # Ensure disable and stop command sent
        time.sleep(0.2) # Give MQTT time to send
        mqtt_client_instance.loop_stop()
        mqtt_client_instance.disconnect()
        print("MQTT: Disconnected.")

    if pygame_initialized:
        print("Pygame: Quitting...")
        pygame.quit()
        print("Pygame: Quit.")

    print("DPG: Destroying context...")
    dpg.destroy_context()
    print("Application finished.")