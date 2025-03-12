import pygame
import serial
import time

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

bt_port = 'COM3'  # Change to your BT COM port
baud_rate = 115200
ser = serial.Serial(bt_port, baud_rate)

print("Joystick started!")

try:
    while True:
        pygame.event.pump()

        # Joystick axes
        forward_axis = -joystick.get_axis(1) # Forward/Backward
        turn_axis = joystick.get_axis(3)     # Turning

        forward_velocity = forward_axis * 30 # rad/s scaling
        turn_velocity = turn_axis * 15       # rad/s scaling

        # Buttons controlling servos
        servo_angles = [90,90,90,90] # default angles

        if joystick.get_button(0):
            servo_angle_1 = 0
        else:
            servo_angle = 90

        if joystick.get_button(1):
            servo_angle2 = 45
        else:
            servo_angle2 = 90

        if joystick.get_button(2):
            servo_angle3 = 135
        else:
            servo_angle3 = 90

        if joystick.get_button(3):
            servo_angle4 = 0
        else:
            servo_angle4 = 90

        command = f"{forward_velocity:.2f},{turn_velocity:.2f},{servo_angle},{servo_angle2},{servo_angle3},{servo_angle4}\n"
        ser.write(command.encode())
        
        print(f"Sent: {command.strip()}")
        pygame.time.wait(100)

except KeyboardInterrupt:
    print("Exiting...")

ser.close()