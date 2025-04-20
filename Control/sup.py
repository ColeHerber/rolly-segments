import dearpygui.dearpygui as dpg

# Define all variables
variables = {
    "motor0": 0.0,
    "motor1": 0.0,
    "mode": 0,
    "vel_p": 0.0,
    "vel_i": 0.0,
    "vel_d": 0.0,
    "vel_lpf": 0.0,
    "servo0": 0.0,
    "servo1": 0.0,
    "servo2": 0.0,
    "servo3": 0.0,
}
input_ids = {}
enable_flag = True

# Input callback: only print the variable being updated
def input_callback(sender, app_data, user_data):
    key = user_data
    try:
        if key == "mode":
            variables[key] = int(app_data)
        else:
            variables[key] = float(app_data)
        print(f"{key}: {variables[key]}")
    except ValueError:
        pass

# Enable/Disable toggle button
def toggle_enable():
    global enable_flag
    enable_flag = not enable_flag
    dpg.set_value("enable_button", f"Enabled: {enable_flag}")
    print(f"enable: {enable_flag}")

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
    dpg.add_button(label="Enabled: True", callback=toggle_enable, tag="enable_button")
    dpg.add_button(label="Exit", callback=lambda: dpg.stop_dearpygui())

# Start GUI
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window(main_window, True)
dpg.start_dearpygui()
dpg.destroy_context()