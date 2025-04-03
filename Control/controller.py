import pygame
import serial
import time

# Bluetooth Serial Port (Check System Preferences > Bluetooth for ESP32 name)
bt_port = "/dev/cu.YOUR_ESP32_BT_NAME"  # Adjust to ESP32's Bluetooth device
baud_rate = 115200

# Connect to ESP32
ser = serial.Serial(bt_port, baud_rate)
print("Connected to ESP32 over Bluetooth!")

# Initialize Pygame for joystick input
pygame.init()
pygame.joystick.init()

# Get the first connected joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using controller: {joystick.get_name()}")

def send_command(forward, turn, s1, s2, s3, s4):
    """
    Send the motor and servo commands to ESP32 over Bluetooth
    """
    command = f"{forward:.2f},{turn:.2f},{s1},{s2},{s3},{s4}\n"
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")

try:
    while True:
        pygame.event.pump()

        # Joystick axes
        forward_axis = -joystick.get_axis(1)  # Left stick Y-axis (inverted)
        turn_axis = joystick.get_axis(2)      # Right stick X-axis

        # Convert to motor commands
        forward_velocity = forward_axis * 30  # Scale to motor max speed
        turn_velocity = turn_axis * 15        # Scale to max turn rate

        # Button controls for servos
        s1 = 45 if joystick.get_button(0) else 90  # Button 0
        s2 = 45 if joystick.get_button(1) else 90  # Button 1
        s3 = 135 if joystick.get_button(2) else 90 # Button 2
        s4 = 0 if joystick.get_button(3) else 90   # Button 3

        send_command(forward_velocity, turn_velocity, s1, s2, s3, s4)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    pygame.quit()