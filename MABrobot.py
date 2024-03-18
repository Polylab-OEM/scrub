import pygame

import time
import math
import re
import sys
import pyCandle
import time
import math
import sys  

# Initialize Pygame
pygame.init()
matcherend = re.compile('\r')


motor1Counter =0
motor2Counter =0
steer1Counter =0
steer2Counter =0
    



def control_motors():
	# Create CANdle object and set FDCAN baudrate to 1Mbps
    candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

# Ping FDCAN bus in search of drives
    ids = candle.ping()

    if len(ids) == 0: # If no drives found -> quit
        sys.exit("EXIT FALIURE") 
    steerL = 200
    driveL = 201
    steerR = 203
    driveR = 204
    # Add all found to the update list
    for id in ids:
        candle.addMd80(id)

    LeftSteerMotor =   candle.md80s[0]
    LeftDriveMotor =   candle.md80s[1]
    RightSteerMotor =   candle.md80s[2]
    RightDriveMotor =   candle.md80s[3]

    candle.controlMd80SetEncoderZero(steerL)               #  Reset encoder at current position for drive motors
    candle.controlMd80Mode(steerL, pyCandle.IMPEDANCE)  # Set mode to position control
    candle.controlMd80Enable(steerL, True)                 # Enable the drive

    candle.controlMd80SetEncoderZero(driveL)               #  Reset encoder at current position for drive motors
    candle.controlMd80Mode(driveL, pyCandle.VELOCITY_PID)  # Set mode to velocity control
    candle.controlMd80Enable(driveL, True)                 # Enable the drive

    candle.controlMd80SetEncoderZero(steerR)               #  Reset encoder at current position for drive motors
    candle.controlMd80Mode(steerR, pyCandle.IMPEDANCE)  # Set mode to position control
    candle.controlMd80Enable(steerR, True)                 # Enable the drive

    candle.controlMd80SetEncoderZero(driveR)               #  Reset encoder at current position
    candle.controlMd80Mode(driveR, pyCandle.VELOCITY_PID)  # Set mode to velocity control
    candle.controlMd80Enable(driveR, True)    
    
    candle.begin()
    # Initialize the joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    


    # Initialize dead man's switch
    deadman_pressed = True
    side_move = False
    differentialSteer = False
    cleaner_motor = False
    cleaner_speed =0
    motor1_speed = 0
    motor2_speed = 0
    motor1_angle = 0
    motor2_angle = 0
    deadband =0.2
    motorSpeedMax = 10
    motorAngleConst = 18000/90
    axis0 =0
    yaw =0
    Wheelbase = 450
    cleaner_trigger =0
    

    

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Read joystick axis values
                axis0 = joystick.get_axis(0)
                axis1 = joystick.get_axis(1)
                yaw = joystick.get_axis(2)
                cleaner_trigger = joystick.get_axis(4)
                # apply dead band to the joy stick to stop drifting 
                if abs(axis0) < deadband:
                    axis0 = 0.0
                if abs(axis1) < deadband:
                    axis1 = 0.0
                # Convert joystick values to motor commands
                # Modify these commands to suit your specific setup
                if side_move:
                    motor1_speed = (axis1 * motorSpeedMax )
                    motor2_speed = (axis1 * motorSpeedMax )
                elif differentialSteer:
                    motor1_speed = int(axis0*0.2 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor2_speed = int(-axis0*0.2 * motorSpeedMax - axis1 * motorSpeedMax)
                
                else:
                
                    #motor1_speed = int(axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    #motor2_speed = int(-axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor1_speed = int( -axis1 * motorSpeedMax)
                    motor2_speed = int( -axis1 * motorSpeedMax)
                    #motor1_speed = int(yaw * motorSpeedMax - axis1 * motorSpeedMax)
                    #motor2_speed = int(-yaw * motorSpeedMax - axis1 * motorSpeedMax)
                    
                print(motor1_speed)
                # Print debug messages without scrolling
                #sys.stdout.write(f"Joystick Axis Values - Axis 0: {axis0}, Axis 1: {axis1}")
               # sys.stdout.write(f"Motor Commands - Motor 1: {motor1_speed}, Motor 2: {motor2_speed}")
                #sys.stdout.write(f"Dead Man's Switch Status: {'Pressed' if deadman_pressed else 'Released'}")
                
        
                # Clear the console
       # sys.stdout.write("\033[H\033[J")
        # Check if the dead man's switch is pressed
        if deadman_pressed:
            if side_move :
            # set commands to control motors on both controllers
                
                motor1_angle = 3.14/2
                motor2_angle = 3.14/2
                
            elif not differentialSteer:
                #send_roboteq_command(ser, 0, f"P 02 {0}")
                motor1_angle = (0*motorAngleConst)+(axis0*3.14)
                motor2_angle = (0*motorAngleConst)+(axis0*3.14)
               
                
            #math to work out speed each motor should be to steer like a car
            SteeringAngleRadians = (motor1_angle/motorAngleConst) * (math.pi / 180)
            try:
                TurningRadius= Wheelbase/math.tan(SteeringAngleRadians)
                #FrontWheelRPMDifference = ((motor1_speed*0.190*math.pi) / (2 * math.pi * TurningRadius)) * 60
            except:
                FrontWheelRPMDifference=1
                
            if cleaner_motor:
                cleaner_speed = -120 *cleaner_trigger
            else:
                 cleaner_speed = 0
            #print(FrontWheelRPMDifference)
            if side_move :
                # Send commands to control motors on both controllers for Drive motors
                #send_roboteq_command(ser, controller_id1, f"!G 01 {motor1_speed}")
                #send_roboteq_command(ser, controller_id2, f"!G 01 {-motor2_speed}")
                LeftDriveMotor.setTargetVelocity(motor1_speed)
                RightDriveMotor.setTargetVelocity(motor2_speed)
                
            else:
                 # Send commands to control motors on both controllers for Drive motors
                #send_roboteq_command(ser, controller_id1, f"!G 01 {motor1_speed}")
                #send_roboteq_command(ser, controller_id2, f"!G 01 {motor2_speed}")
                LeftDriveMotor.setTargetVelocity(-motor1_speed)
                RightDriveMotor.setTargetVelocity(motor2_speed)
           
            # Send commands to control motors on both controllers for steering motors
            #send_roboteq_command(ser, controller_id1, f"!P 02 {motor1_angle}")
            #send_roboteq_command(ser, controller_id2, f"!P 02 {motor2_angle}")
            LeftSteerMotor.setTargetPosition(motor1_angle)
            RightSteerMotor.setTargetPosition(motor2_angle)
            
            # send command for cleaner motor
            #send_roboteq_command(ser, controller_id3, f"!G 01 {cleaner_speed}")
        else:
            
            # Stop motors when the dead man's switch is released
           LeftDriveMotor.setTargetVelocity(0)
           RightDriveMotor.setTargetVelocity(0)
  
        # Check the status of the dead man's switch (you can customize this logic)
        # For example, you can use a button on the joystick or a physical switch
        deadman_button = joystick.get_button(4)  # Adjust the button number
        deadman_pressed = bool(deadman_button)

                # For example, you can use a button on the joystick or a physical switch
        side_move = bool(joystick.get_button(5))  # Adjust the button number
        if(cleaner_trigger >0):
            cleaner_motor = True  # Adjust the button number
        else:
             cleaner_motor = False
        #differential steer option.
        differentialSteer = bool(joystick.get_button(0))  # Adjust the button number
        #if (joystick.get_button(3)):
         #   exampleAuto()
        # Add a slight delay to avoid rapid changes in motor speed
        time.sleep(0.01)

if __name__ == "__main__":
    
    control_motors()
