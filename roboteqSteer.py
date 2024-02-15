import pygame
import serial
import serial.tools.list_ports
import time

import sys
# Initialize Pygame
pygame.init()

# Function to list available serial ports
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for index, port in enumerate(ports, start=1):
            print(f"{index}. {port.device}")
        return ports

# Function to select a serial port
def select_serial_port(ports):
    try:
        choice = int(input("Select a serial port (enter the number): "))
        if 1 <= choice <= len(ports):
            selected_port = ports[choice - 1].device
            print(f"Selected serial port: {selected_port}")
            return selected_port
        else:
            print("Invalid choice. Please enter a valid number.")
            return select_serial_port(ports)
    except ValueError:
        print("Invalid input. Please enter a number.")
        return select_serial_port(ports)

# Open a serial connection to the selected port
def open_serial_port(selected_port, baud_rate):
    ser = serial.Serial(selected_port, baud_rate)
    return ser

# Function to send commands to the Roboteq controllers
def send_roboteq_command(ser, controller_id, command):
    full_command = f"@0{controller_id}!{command}\r\n"
    print(full_command)
    ser.write(full_command.encode())


def control_motors(ser, controller_id1, controller_id2):
    # Initialize the joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Initialize dead man's switch
    deadman_pressed = True
    side_move = False
    differentialSteer = False
    motor1_speed = 0
    motor2_speed = 0
    motor1_angle = 0
    motor2_angle = 0
    deadband =0.1
    motorSpeedMax = 500
    motorAngleConst = 18000/90
    axis0 =0
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Read joystick axis values
                axis0 = joystick.get_axis(0)
                axis1 = joystick.get_axis(1)
                # apply dead band to the joy stick to stop drifting 
                if abs(axis0) < deadband:
                    axis0 = 0.0
                if abs(axis1) < deadband:
                    axis1 = 0.0
                # Convert joystick values to motor commands
                # Modify these commands to suit your specific setup
                if side_move:
                    motor1_speed = int(axis1 * motorSpeedMax )
                    motor2_speed = int(axis1 * motorSpeedMax )
                elif differentialSteer:
                    motor1_speed = int(axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor2_speed = int(-axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                
                else:
                
                    #motor1_speed = int(axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    #motor2_speed = int(-axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor1_speed = int( -axis1 * motorSpeedMax)
                    motor2_speed = int( -axis1 * motorSpeedMax)


                # Print debug messages without scrolling
                print(f"Joystick Axis Values - Axis 0: {axis0}, Axis 1: {axis1}")
                print(f"Motor Commands - Motor 1: {motor1_speed}, Motor 2: {motor2_speed}")
                print(f"Dead Man's Switch Status: {'Pressed' if deadman_pressed else 'Released'}")
                
        
                # Clear the console
        sys.stdout.write("\033[H\033[J")
        # Check if the dead man's switch is pressed
        if deadman_pressed:
            if side_move :
            # set commands to control motors on both controllers
                
                motor1_angle = (90*motorAngleConst)+(axis0*8000)
                motor2_angle = (90*motorAngleConst)+(axis0*8000)
                motor2_speed *=-1
            elif not differentialSteer:
                #send_roboteq_command(ser, 0, f"P 02 {0}")
                motor1_angle = (0*motorAngleConst)+(axis0*18000)
                motor2_angle = (0*motorAngleConst)+(-axis0*18000)
            # Send commands to control motors on both controllers for Drive motors
            send_roboteq_command(ser, controller_id1, f"G 01 {motor1_speed}")
            send_roboteq_command(ser, controller_id2, f"G 01 {motor2_speed}")
            
            # Send commands to control motors on both controllers for steering motors
            send_roboteq_command(ser, controller_id1, f"P 02 {motor1_angle}")
            send_roboteq_command(ser, controller_id2, f"P 02 {motor2_angle}")
        else:
            
            # Stop motors when the dead man's switch is released
            send_roboteq_command(ser, controller_id1, "G 01 0")
            send_roboteq_command(ser, controller_id2, "G 01 0")
         

        # Check the status of the dead man's switch (you can customize this logic)
        # For example, you can use a button on the joystick or a physical switch
        deadman_button = joystick.get_button(4)  # Adjust the button number
        deadman_pressed = bool(deadman_button)

                # For example, you can use a button on the joystick or a physical switch
        side_move = bool(joystick.get_button(5))  # Adjust the button number
        
        #differential steer option.
        differentialSteer = bool(joystick.get_button(0))  # Adjust the button number

        # Add a slight delay to avoid rapid changes in motor speed
        time.sleep(0.1)

if __name__ == "__main__":
    ports = list_serial_ports()
    if not ports:
        exit("No serial ports found.")
    
    selected_port = select_serial_port(ports)
    baud_rate = 115200  # Adjust to your controller's baud rate

    controller_id1 = 1  # Replace with the actual ID of the first controller
    controller_id2 = 2  # Replace with the actual ID of the second controller

    ser = open_serial_port(selected_port, baud_rate)
     # Add a slight delay to avoid rapid changes in motor speed
    time.sleep(0.1)
    full_command = f"@00!CCS 2 0\r\n"
    #print(full_command)
    ser.write(full_command.encode())
    time.sleep(0.1)
    full_command = f"@00!CCS 2 0\r\n"
   # print(full_command)
    ser.write(full_command.encode())
    control_motors(ser, controller_id1, controller_id2)
