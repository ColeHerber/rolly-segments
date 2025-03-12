import pygame
import serial
import time

# Initialize Joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Bluetooth port (replace with your COM port)
bt_port = 'COM3'
baud_rate = 115200
ser = serial.Serial(bt_port, baud_rate)

print("Joystick started. Sending commands...")

try:
    while True:
        pygame.event.pump()

        # Joystick axes:
        forward_axis = -joystick.get_axis(1)  # Usually forward/backward
        turn_axis = joystick.get_axis(3)      # Usually left/right

        # Scale velocities
        forward_velocity = forward_axis * 30  # Max speed: 30 rad/s
        turn_velocity = turn_axis * 15        # Turn scaling

        # Servo angle example using button 0
        servo_angle = 90
        if joystick.get_button(0):
            servo_angle = 0   # e.g., button pressed moves servo angle
        elif joystick.get_button(1):
            servo_angle = 90

        # Format command string
        command = f"{forward_velocity:.2f},{turn_velocity:.2f},{servo_angle}\n"
        ser.write(command.encode())
        
        print(f"Sent: {command.strip()}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()