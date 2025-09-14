import paho.mqtt.client as mqtt
import json
import pygame
import time

# MQTT Configuration
MQTT_BROKER = "rollyServer.local"
MQTT_PORT = 1883
MQTT_SUB_TOPIC = "rolly"
MQTT_PUB_TOPIC = "rolly/cmd"

# MQTT Setup
def on_connect(client, userdata, flags, rc):
    print("✅ Connected to MQTT with result code", rc)
    client.subscribe(MQTT_SUB_TOPIC)

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        # print("\n📡 Telemetry:")
        # for key, value in data.items():
            # print(f"  {key}: {value}")

    except Exception as e:
        print("❌ Failed to parse telemetry:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

# Controller Setup
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ No controller found. Connect an Xbox controller and restart.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 Controller connected: {joystick.get_name()}")
def apply_deadband(value, threshold=0.3):
    return 0 if abs(value) < threshold else value

def get_motor_values():
    pygame.event.pump()

    raw_y = -joystick.get_axis(1)  # Invert Y (forward is negative)
    raw_x = joystick.get_axis(0)   # X is left/right turn

    # Apply 30% deadband
    y = apply_deadband(raw_y)
    x = apply_deadband(raw_x)

    # Clamp to [-1, 1] just in case
    y = max(-1, min(1, y))
    x = max(-1, min(1, x))

    base_speed = y * 5
    turn_adjust = x * 5

    motor0 = round(base_speed - turn_adjust, 2)
    motor1 = round(base_speed + turn_adjust, 2)

    # Clamp motor values
    motor0 = max(-5, min(5, motor0))
    motor1 = max(-5, min(5, motor1))

    return motor0, motor1# Main Loop
try:
    print("\n🔁 Streaming controller data. Press Enter anytime to enter PID values.")
    while True:
        motor0, motor1 = get_motor_values()
        payload = {
            "motor0": float(4),
            "motor1": float(4),
            "enable": True
        }
        client.publish(MQTT_PUB_TOPIC, json.dumps(payload))
        print(f"🎮 Motor Command: motor0={motor0}, motor1={motor1}", end="\r")

        # # Check if user wants to enter PID
        # if input("\n↩️  Press Enter to tune PID or Ctrl+C to quit: ") == "":
        #     try:
        #         p = input("P gain (vel_p): ")
        #         i = input("I gain (vel_i): ")
        #         d = input("D gain (vel_d): ")

        #         pid_cmd = {}
        #         if p.strip(): pid_cmd["vel_p"] = float(p)
        #         if i.strip(): pid_cmd["vel_i"] = float(i)
        #         if d.strip(): pid_cmd["vel_d"] = float(d)

        #         if pid_cmd:
        #             pid_cmd["enable"] = True
        #             client.publish(MQTT_PUB_TOPIC, json.dumps(pid_cmd))
        #             print(f"🚀 Sent PID: {pid_cmd}")
        #         else:
        #             print("⚠️  No PID values entered.")
        #     except Exception as e:
        #         print("❌ Invalid PID input:", e)

        # time.sleep(0.1)

except KeyboardInterrupt:
    print("\n👋 Exiting...")
finally:
    client.loop_stop()
    client.disconnect()
    pygame.quit()