import paho.mqtt.client as mqtt
import json
import pygame
import time

# MQTT Configuration
MQTT_BROKER = "rollyserver.local"
MQTT_PORT = 1883
MQTT_SUB_TOPIC = "rolly"
MQTT_PUB_TOPIC = "rolly/cmd/pc"

# --- Pygame GUI Setup ---
pygame.init()

# Colors
COLOR_INACTIVE = pygame.Color('lightskyblue3')
COLOR_ACTIVE = pygame.Color('dodgerblue2')
COLOR_TEXT = pygame.Color('black')
COLOR_BG = pygame.Color('white')
COLOR_ENABLED = pygame.Color('green')
COLOR_DISABLED = pygame.Color('red')


# Screen
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 700
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Rolly Control (Pygame)")
FONT = pygame.font.Font(None, 28)

class InputBox:
    def __init__(self, x, y, w, h, key, label='', text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = COLOR_INACTIVE
        self.text = text
        self.key = key
        self.label = label
        self.txt_surface = FONT.render(text, True, COLOR_TEXT)
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = not self.active
            else:
                self.active = False
            self.color = COLOR_ACTIVE if self.active else COLOR_INACTIVE
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_RETURN:
                    self.active = False
                    self.color = COLOR_INACTIVE
                elif event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    self.text += event.unicode
                self.txt_surface = FONT.render(self.text, True, COLOR_TEXT)

    def draw(self, screen):
        # Draw the label
        label_surface = FONT.render(self.label, True, COLOR_TEXT)
        screen.blit(label_surface, (self.rect.x - 200, self.rect.y + 5))
        # Draw the input box
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))
        pygame.draw.rect(screen, self.color, self.rect, 2)

class RollyControlPygame:
    def __init__(self):
        self.clock = pygame.time.Clock()
        self.running = True
        self.telemetry_data = {}
        self.enabled = False
        self.initial_values_set = False
        self.last_reconnect_attempt = 0

        # --- UI Elements ---
        self.input_boxes = []
        defaults = {
            "bal_p_theta": "10", "bal_i_theta": "1.2", "bal_d_theta": "-2",
            "bal_p_theta_dot": "0", "bal_i_theta_dot": "0", "bal_d_theta_dot": "0",
        }
        pids = {
            "bal_p_theta": "Theta P", "bal_i_theta": "Theta I", "bal_d_theta": "Theta D",
            "bal_p_theta_dot": "Theta Dot P", "bal_i_theta_dot": "Theta Dot I", "bal_d_theta_dot": "Theta Dot D",
        }
        y_offset = 20
        for key, label in pids.items():
            self.input_boxes.append(InputBox(250, y_offset, 140, 32, key, label, text=defaults[key]))
            y_offset += 40

        self.send_button = pygame.Rect(200, y_offset + 40, 140, 40)
        self.enable_button = pygame.Rect(360, y_offset + 40, 140, 40)

        # --- MQTT Client ---
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.loop_start()
        self.connect_mqtt()

    def connect_mqtt(self):
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        except Exception as e:
            print(f"❌ Could not connect to MQTT Broker: {e}")
            self.last_reconnect_attempt = time.time()

    def on_connect(self, client, userdata, flags, rc):
        print("✅ Connected to MQTT with result code", rc)
        client.subscribe(MQTT_SUB_TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            self.telemetry_data = json.loads(msg.payload.decode())

            if not self.initial_values_set:
                for box in self.input_boxes:
                    if box.key in self.telemetry_data:
                        new_text = str(self.telemetry_data[box.key])
                        box.text = new_text
                        box.txt_surface = FONT.render(box.text, True, COLOR_TEXT)

                if 'enable' in self.telemetry_data:
                    self.enabled = self.telemetry_data['enable']
                self.initial_values_set = True
        except Exception as e:
            print(f"❌ Failed to parse telemetry: {e}")

    def send_commands(self):
        payload = {}
        for box in self.input_boxes:
            if box.text.strip():
                try:
                    payload[box.key] = float(box.text)
                except ValueError:
                    print(f"⚠️ Invalid value for {box.label}: {box.text}")
        
        if payload:
            self.client.publish(MQTT_PUB_TOPIC, json.dumps(payload))
            print(f"🚀 Sent Command: {payload}")
        else:
            print("⚠️ No values entered to send.")

    def toggle_enable(self):
        self.enabled = not self.enabled
        payload = {"enable": self.enabled}
        self.client.publish(MQTT_PUB_TOPIC, json.dumps(payload))
        print(f"🚀 Sent Command: {payload}")


    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.send_button and self.send_button.collidepoint(event.pos):
                    self.send_commands()
                elif self.enable_button.collidepoint(event.pos):
                    self.toggle_enable()
            for box in self.input_boxes:
                box.handle_event(event)

    def draw_telemetry(self):
        y_offset = 450
        telemetry_title = FONT.render("--- Telemetry ---", True, COLOR_TEXT)
        screen.blit(telemetry_title, (50, y_offset))
        y_offset += 30
        if not self.telemetry_data:
            no_data_text = FONT.render("Waiting for data...", True, COLOR_TEXT)
            screen.blit(no_data_text, (50, y_offset))
            return

        filtered = {k: v for k, v in self.telemetry_data.items() if not k.startswith("bal_")}
        for key, value in filtered.items():
            text = f"{key}: {value}"
            telemetry_surface = FONT.render(text, True, COLOR_TEXT)
            screen.blit(telemetry_surface, (50, y_offset))
            y_offset += 25
            if y_offset > SCREEN_HEIGHT - 20: # Don't draw off-screen
                break

    def draw(self):
        screen.fill(COLOR_BG)

        # Draw input boxes
        for box in self.input_boxes:
            box.draw(screen)

        # Draw send button
        pygame.draw.rect(screen, COLOR_INACTIVE, self.send_button)
        send_text = FONT.render("Send Params", True, COLOR_TEXT)
        screen.blit(send_text, (self.send_button.x + 10, self.send_button.y + 10))

        # Draw enable/disable button
        if self.enabled:
            btn_color = COLOR_ENABLED
            btn_text = "Enabled"
        else:
            btn_color = COLOR_DISABLED
            btn_text = "Disabled"
        pygame.draw.rect(screen, btn_color, self.enable_button)
        enable_text_surface = FONT.render(btn_text, True, COLOR_TEXT)
        screen.blit(enable_text_surface, (self.enable_button.x + 25, self.enable_button.y + 10))


        # Draw telemetry
        self.draw_telemetry()

        pygame.display.flip()
        
    def run(self):
        while self.running:
            if not self.client.is_connected():
                now = time.time()
                if now - self.last_reconnect_attempt > 5:
                    print("🔁 Attempting MQTT reconnect...")
                    try:
                        self.client.reconnect()
                    except Exception as e:
                        print(f"❌ MQTT reconnect failed: {e}, retrying full connect...")
                        self.connect_mqtt()
                    self.last_reconnect_attempt = now

            self.handle_events()
            self.draw()
            self.clock.tick(30)

        # Cleanup
        print("\n👋 Exiting...")
        self.client.loop_stop()
        self.client.disconnect()
        pygame.quit()

if __name__ == "__main__":
    app = RollyControlPygame()
    app.run()
