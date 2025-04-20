import random
import time
import random
import threading
import dearpygui.dearpygui as dpg
# Define all variables
import json

variables = {
    "target0":0.0,
    "target1":0.0,
    "vel_p":0.0,
    "vel_i":0.0,
    "vel_d":0.0,
    "vel_lpf":0.0,
}
input_ids = {}
enable_flag = False

from paho.mqtt import client as mqtt_client

broker = 'rollyserver.local'
port = 1883
topic = "rolly/cmd/"#the slashes matter, yay
# Generate a Client ID with the publish prefix.
client_id = f'mac'
# username = 'emqx'
# password = 'public'
properties = "hello"

def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties):
        if flags.session_present:
            print("WTF")
        if reason_code == 0:            
            print("Connected to MQTT Broker!")
        if reason_code > 0:
            print("Failed to connect, return code %d\n", reason_code)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    #THIS STUFF WAS UDPATED TO BE BREAKGIN IN 2024, old tutorials are not updated
   
   # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    #adding properties here breaks
    return client


def publish(client,msg):
    result = client.publish(topic, msg)
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")


# Input callback: only print the variable being updated
def input_callback(sender, app_data, user_data):
    key = user_data
    try:
        if key == "mode":
            variables[key] = int(app_data)
        else:
            variables[key] = float(app_data)
        msg = f"{key}: {variables[key]}"

        publish(client, json.dumps(variables, separators=(',', ':')))
    except ValueError:
        pass
    

# Enable/Disable toggle button
def toggle_enable():
    global enable_flag
    enable_flag = not enable_flag
    dpg.set_value("enable_button", f"Enabled: {enable_flag}")
    if enable_flag:
        msg = "enable"
    else:
        msg = "disable"
    msg = json.dumps(msg)
    publish(client, msg)

# GUI setup
dpg.create_context()
dpg.create_viewport(title="Control Panel", width=700, height=350)

with dpg.window(label="Edit Parameters", width=680, height=330) as main_window:
    dpg.add_text("Motor / Servo Configurator", color=(255, 255, 0))
    dpg.add_separator()

    # Grouping variables side by side (3 per row)
    items_per_row = 3
    keys = list(variables.keys())
    

    for i in range(0, len(keys), items_per_row):
        row_keys = keys[i:i + items_per_row]
        with dpg.group(horizontal=True):
            for key in row_keys:
                with dpg.group():
                    dpg.add_text(f"{key}")
                    input_id = dpg.add_input_text(default_value=str(variables[key]),
                                                  callback=input_callback,
                                                  user_data=key,
                                                  width=100)
                    input_ids[key] = input_id

    dpg.add_spacer(height=10)
    dpg.add_button(label="Enabled", callback=toggle_enable, tag="enable_button")
    dpg.add_button(label="Exit", callback=lambda: dpg.stop_dearpygui())


client = connect_mqtt()
client.loop_start()
client.loop_stop()

# Start GUI
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window(main_window, True)
dpg.start_dearpygui()
dpg.destroy_context()
