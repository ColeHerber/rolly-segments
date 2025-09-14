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
        #     print(f"  {key}: {value}")

    except Exception as e:
        print("❌ Failed to parse telemetry:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

"""
Keyboard controls (requires a tiny Pygame window to capture input):
  - W/Up: forward
  - S/Down: reverse
  - A/Left: turn left
  - D/Right: turn right
  - Space: stop (zero both motors)
  - Esc or window close: quit
Speeds are mapped to [-MAX_SPEED, MAX_SPEED].
"""

# Keyboard Setup
pygame.init()
try:
    # Create a small window so pygame can receive keyboard events
    screen = pygame.display.set_mode((360, 120))
    pygame.display.set_caption("Rolly Keyboard Control")
except Exception:
    # Fallback to headless if needed
    pygame.display.init()

MAX_SPEED = 6.0

def get_motor_values_from_key(key):
    """Return a single-shot (target0, target1) for a given key, or None.
    Standard controls: WASD or arrows. Space = stop.
    Enforces target1 = -target0.
    """
    # Stop
    if key == pygame.K_SPACE:
        return 0.0, 0.0

    # Forward
    if key in (pygame.K_w, pygame.K_UP):
        t0 = MAX_SPEED
        return t0, -t0

    # Reverse
    if key in (pygame.K_s, pygame.K_DOWN):
        t0 = -MAX_SPEED
        return t0, -t0

    # Turn left (spin)
    if key in (pygame.K_a, pygame.K_LEFT):
        t0 = -MAX_SPEED
        return t0, -t0

    # Turn right (spin)
    if key in (pygame.K_d, pygame.K_RIGHT):
        t0 = MAX_SPEED
        return t0, -t0

    return None

# Main Loop
try:
    print("\n🔁 Keyboard control: press keys to send one-shot commands. Space=stop, Esc=quit.")
    pygame.key.set_repeat(0)  # Disable key repeat to avoid multiple KEYDOWNs
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    raise KeyboardInterrupt

                result = get_motor_values_from_key(event.key)
                if result is None:
                    continue
                target0, target1 = result
                payload = {
                    "target0": float(target0),
                    "target1": float(target1),
                    "enable": True

                }
                client.publish(MQTT_PUB_TOPIC, json.dumps(payload))
                print(f"\n⌨️  Sent target0={target0}, target1={target1}", end="")

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

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n👋 Exiting...")
finally:
    client.loop_stop()
    client.disconnect()
    pygame.quit()
