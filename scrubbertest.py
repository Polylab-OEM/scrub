import pyCandle
import time
import math
import sys   

# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

if len(ids) == 0: # If no drives found -> quit
    sys.exit("EXIT FALIURE") 

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

# Reset encoder at current position
#candle.controlMd80SetEncoderZero(ids[0])       
# Set mode to impedance control
candle.controlMd80Mode(ids[1], pyCandle.IMPEDANCE)     
# Enable the drive
candle.controlMd80Enable(ids[1], True);     
scrubber = candle.md80s[0];
lifter  = candle.md80s[1];
print(candle.md80s[0])
t = 0.0
dt = 0.04
candle.controlMd80SetEncoderZero(ids[0])               #  Reset encoder at current position

candle.controlMd80Mode(ids[0], pyCandle.VELOCITY_PID)  # Set mode to velocity control
candle.controlMd80Enable(ids[0], True)                 # Enable the drive

# Begin update loop (it starts in the background)
candle.begin()

scrubber.setTargetVelocity(-600)
for i in range(4):
    t=t+dt
    #  After powerup the drive will load set of default parameters for every controller.
    #  To make the drive move all we got to do is set mode (.controlMd80Mode) and set target
    #  (.setTargetPosition, .setTargetVelocity, .setTargetTorque)
    lifter.setTargetPosition(4)
    time.sleep(2)  # Add some delay
    lifter.setTargetPosition(-1)
    time.sleep(2) 
scrubber.setTargetVelocity(0)
# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS")
