import pygame
import serial
import serial.tools.list_ports
import time
import math
import re
import sys
# Initialize Pygame
pygame.init()
matcherend = re.compile('\r')


motor1Counter =0
motor2Counter =0
steer1Counter =0
steer2Counter =0
    
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
    ser = serial.Serial(selected_port, baud_rate, timeout=0.01, xonxoff=False, rtscts=False, dsrdtr=False)
    return ser
def handle_data(data):
    #print(data)
    if not LastCommand.empty():
        com = LastCommand.get()
    else:
        com =""

    if com == '?V 2\r':
        try:
            Batt = float(str(data[2]+data[3]+data[4]))/10
            #print Batt
            #print (Batt, intrn)
            BatteryQ.queue.clear()
            BatteryQ.put(Batt)
        except:
            a=0
    elif com == '?C 01\r':
        #print (LastCommand.get())
        Mcnt1 = ''
        for i in range(2,len(data)):
            if data[i] != '\r':
                Mcnt1 +=data[i]
            
        
        MCnt1.queue.clear()
        MCnt1.put(Mcnt1)

    elif com == '?C 02\r':
        #print (LastCommand.get())
        Mcnt2 = ''
        for i in range(2,len(data)):
            if data[i] != '\r':
                Mcnt2 +=data[i]
        MCnt2.queue.clear()
        MCnt2.put(Mcnt2)
            #print (MCnt1)
    

    elif data == 'W':
        a=0
    else:
        #if message is not an expected result it might be a command message
        
        LastCommand.put(data)
    RawData.queue.clear()
    RawData.put(data)
        #print "Watchdog"
# Function to send commands to the Roboteq controllers
def send_roboteq_command(ser, controller_id, command):
    full_command = f"@0{controller_id}{command}\r\n"
    #print(full_command +"\r\r")
    ser.write(full_command.encode())
    line = []
    #print(ser)
    reading = ""
    rx = ser.readline()
    #time.sleep(0.1)
    # ext = c.split('\\r')
    # if len(ext) >1:
        # if ext[1] == '+':
            # print ("success")
        # else:
            # print("fail")
    #print (matcherend.search(reading))
    out = ""
    for r in rx:
       
        if (chr(r) == '\r'):
            print(str(reading +'\r'))
            outish = reading.split('=')
            if(len(outish)>1):
                out = outish[1]
            reading = ""
            
        else:
            reading += chr(r)
            
        
    #handle_data(bytes(reading))
    #print(reading +"\r\r")
    return out
    
    
def exampleAuto():
    counter = 10000
    movecomplete =False
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")
    while not movecomplete:
        motor1Counter = int(send_roboteq_command(ser, controller_id1, "?CSS 01 "))
        motor2Counter = int(send_roboteq_command(ser, controller_id2, "?CSS 01 "))
        steerCounter1 = int(send_roboteq_command(ser, controller_id1, "?CSS 02"))
        steerCounter2 = int(send_roboteq_command(ser, controller_id2, "?CSS 02"))
        if(motor1Counter < counter):
            send_roboteq_command(ser, controller_id1, f"!G 01 {500}")
        else:
            send_roboteq_command(ser, controller_id1, f"!G 01 {0}")
        if(motor2Counter < counter):
            send_roboteq_command(ser, controller_id2, f"!G 01 {500}")
        else:
            send_roboteq_command(ser, controller_id2, f"!G 01 {0}")
        if(motor1Counter >= counter) and(motor2Counter >= counter):
            movecomplete =True
    time.sleep(1)
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")
    
    counter = 18000
    movecomplete =False
    while not movecomplete:
        motor1Counter = int(send_roboteq_command(ser, controller_id1, "?CSS 01 "))
        motor2Counter = int(send_roboteq_command(ser, controller_id2, "?CSS 01 "))
        steerCounter1 = int(send_roboteq_command(ser, controller_id1, "?CSS 02"))
        steerCounter2 = int(send_roboteq_command(ser, controller_id2, "?CSS 02"))
        
        send_roboteq_command(ser, controller_id1, f"!P 02 {18000}")

   
        send_roboteq_command(ser, controller_id2, f"!P 02 {18000}")
        time.sleep(1)
        movecomplete = True
        
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")
    time.sleep(1)
    counter = 1000
    movecomplete =False
    while not movecomplete:
        motor1Counter = int(send_roboteq_command(ser, controller_id1, "?CSS 01 "))
        motor2Counter = int(send_roboteq_command(ser, controller_id2, "?CSS 01 "))*-1
        steerCounter1 = int(send_roboteq_command(ser, controller_id1, "?CSS 02"))
        steerCounter2 = int(send_roboteq_command(ser, controller_id2, "?CSS 02"))
        if(motor1Counter < counter):
            send_roboteq_command(ser, controller_id1, f"!G 01 {500}")
        else:
            send_roboteq_command(ser, controller_id1, f"!G 01 {0}")
        if(motor2Counter < counter):
            send_roboteq_command(ser, controller_id2, f"!G 01 {-500}")
        else:
            send_roboteq_command(ser, controller_id2, f"!G 01 {0}")
        if(motor1Counter >= counter) and(motor2Counter >= counter):
            movecomplete =True
    time.sleep(2)
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")
    
    counter = 0
    movecomplete =False
    while not movecomplete:
        motor1Counter = int(send_roboteq_command(ser, controller_id1, "?CSS 01 "))
        motor2Counter = int(send_roboteq_command(ser, controller_id2, "?CSS 01 "))
        steerCounter1 = int(send_roboteq_command(ser, controller_id1, "?CSS 02"))
        steerCounter2 = int(send_roboteq_command(ser, controller_id2, "?CSS 02"))
        
        send_roboteq_command(ser, controller_id1, f"!P 02 {0}")

   
        send_roboteq_command(ser, controller_id2, f"!P 02 {0}")
        time.sleep(2)
        movecomplete = True
        
    counter = 10000
    movecomplete =False
    while not movecomplete:
        motor1Counter = int(send_roboteq_command(ser, controller_id1, "?CSS 01 "))*-1
        motor2Counter = int(send_roboteq_command(ser, controller_id2, "?CSS 01 "))*-1
        steerCounter1 = int(send_roboteq_command(ser, controller_id1, "?CSS 02"))
        steerCounter2 = int(send_roboteq_command(ser, controller_id2, "?CSS 02"))
        if(motor1Counter < counter):
            send_roboteq_command(ser, controller_id1, f"!G 01 {-500}")
        else:
            send_roboteq_command(ser, controller_id1, f"!G 01 {0}")
        if(motor2Counter < counter):
            send_roboteq_command(ser, controller_id2, f"!G 01 {-500}")
        else:
            send_roboteq_command(ser, controller_id2, f"!G 01 {0}")
        if(motor1Counter >= counter) and(motor2Counter >= counter):
            movecomplete =True
    time.sleep(1)
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")

def control_motors(ser, controller_id1, controller_id2,controller_id3):
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
    motorSpeedMax = 500
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
                    motor1_speed = int(axis1 * motorSpeedMax )
                    motor2_speed = int(axis1 * motorSpeedMax )
                elif differentialSteer:
                    motor1_speed = int(axis0*0.2 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor2_speed = int(-axis0*0.2 * motorSpeedMax - axis1 * motorSpeedMax)
                
                else:
                
                    #motor1_speed = int(axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    #motor2_speed = int(-axis0 * motorSpeedMax - axis1 * motorSpeedMax)
                    motor1_speed = int( -axis1 * motorSpeedMax)
                    motor2_speed = int( -axis1 * motorSpeedMax)
                    motor1_speed = int(yaw * motorSpeedMax - axis1 * motorSpeedMax)
                    motor2_speed = int(-yaw * motorSpeedMax - axis1 * motorSpeedMax)
                    

                # Print debug messages without scrolling
                sys.stdout.write(f"Joystick Axis Values - Axis 0: {axis0}, Axis 1: {axis1}")
                sys.stdout.write(f"Motor Commands - Motor 1: {motor1_speed}, Motor 2: {motor2_speed}")
                sys.stdout.write(f"Dead Man's Switch Status: {'Pressed' if deadman_pressed else 'Released'}")
                
        
                # Clear the console
        sys.stdout.write("\033[H\033[J")
        # Check if the dead man's switch is pressed
        if deadman_pressed:
            if side_move :
            # set commands to control motors on both controllers
                
                motor1_angle = (90*motorAngleConst)+(axis0*8000)
                motor2_angle = (90*motorAngleConst)+(axis0*8000)
                
            elif not differentialSteer:
                #send_roboteq_command(ser, 0, f"P 02 {0}")
                motor1_angle = (0*motorAngleConst)#+(axis0*18000)
                motor2_angle = (0*motorAngleConst)#+(-axis0*18000)
               
                
            #math to work out speed each motor should be to steer like a car
            SteeringAngleRadians = (motor1_angle/motorAngleConst) * (math.pi / 180)
            try:
                TurningRadius= Wheelbase/math.tan(SteeringAngleRadians)
                FrontWheelRPMDifference = ((motor1_speed*0.190*math.pi) / (2 * math.pi * TurningRadius)) * 60
            except:
                FrontWheelRPMDifference=1
                
            if cleaner_motor:
                cleaner_speed = -120 *cleaner_trigger
            else:
                 cleaner_speed = 0
            #print(FrontWheelRPMDifference)
            if side_move :
                # Send commands to control motors on both controllers for Drive motors
                send_roboteq_command(ser, controller_id1, f"!G 01 {motor1_speed}")
                send_roboteq_command(ser, controller_id2, f"!G 01 {-motor2_speed}")
            else:
                 # Send commands to control motors on both controllers for Drive motors
                send_roboteq_command(ser, controller_id1, f"!G 01 {motor1_speed}")
                send_roboteq_command(ser, controller_id2, f"!G 01 {motor2_speed}")
           
            # Send commands to control motors on both controllers for steering motors
            send_roboteq_command(ser, controller_id1, f"!P 02 {motor1_angle}")
            send_roboteq_command(ser, controller_id2, f"!P 02 {motor2_angle}")
            
            # send command for cleaner motor
            send_roboteq_command(ser, controller_id3, f"!G 01 {cleaner_speed}")
        else:
            
            # Stop motors when the dead man's switch is released
            send_roboteq_command(ser, controller_id1, "!G 01 0")
            send_roboteq_command(ser, controller_id2, "!G 01 0")
            send_roboteq_command(ser, controller_id3, "!G 01 0")
        #get motor counters.
        motor1Counter = send_roboteq_command(ser, controller_id1, "?CSS 01 ")
        motor2Counter = send_roboteq_command(ser, controller_id2, "?CSS 01 ")
        steerCounter1 = send_roboteq_command(ser, controller_id1, "?CSS 02")
        steerCounter2 = send_roboteq_command(ser, controller_id2, "?CSS 02")    
        
        print(motor1Counter,motor2Counter,steerCounter1,steerCounter2)
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
    ports = list_serial_ports()
    if not ports:
        exit("No serial ports found.")
    
    selected_port = select_serial_port(ports)
    baud_rate = 115200  # Adjust to your controller's baud rate

    controller_id1 = 1  # Replace with the actual ID of the first controller
    controller_id2 = 2  # Replace with the actual ID of the second controller
    controller_id3 = 3

    ser = open_serial_port(selected_port, baud_rate)#2000000, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
     # Add a slight delay to avoid rapid changes in motor speed
    time.sleep(0.1)
    full_command = f"@01!CCS 2 0\r\n" # sets steering motor 1 current position as 0
    #print(full_command)
    ser.write(full_command.encode())
    time.sleep(0.1)
    full_command = f"@02!CCS 2 0\r\n" # sets steering motor 2 current position as 0
   # print(full_command)
    ser.write(full_command.encode())
    
    send_roboteq_command(ser, controller_id1, "!CSS 01 0")
    send_roboteq_command(ser, controller_id2, "!CSS 01 0")
    send_roboteq_command(ser, controller_id1, "!CSS 02 0")
    send_roboteq_command(ser, controller_id2, "!CSS 02 0")
    
    
    control_motors(ser, controller_id1, controller_id2,controller_id3)
