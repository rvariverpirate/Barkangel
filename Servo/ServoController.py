#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial

class servoController():
    ##################
    # Servo Commands #
    ##################
    right = "r"
    left = "l"
    up = "u"
    down = "d"  
    
    
    def __init__(self, serialPort, baudRate):
        ##############################
        # Setup Serial Communication #
        ##############################
        self.avrSerial = serial.Serial(serialPort, baudRate, timeout=3)


    # Puts string into propper format for the microcontroller to receive
    def formatTheta(self, theta):
        if(theta < 10):
            addString = "00"
        elif(theta < 100):
            addString = "0"
        else:
            addString = ""
        return addString + str(theta)


    def updateAngles(self, pan_deg, tilt_deg):
        
        commandString = ""
        sendMessage = False
        
        # Control Pan
        if(pan_deg > 0):
            print("Move Left")
            avrCommand = self.left
            degreeShift = pan_deg
            print("degreeShift = ", degreeShift)
            degreeShift = self.formatTheta(abs(int(degreeShift)))   
            commandString += avrCommand + degreeShift
            sendMessage = True
            
        elif(pan_deg < 0):
            print("Move Right")
            avrCommand = self.right
            degreeShift = pan_deg
            print("degreeShift = ", degreeShift)
            degreeShift = self.formatTheta(abs(int(degreeShift)))    
            commandString += avrCommand + degreeShift
            sendMessage = True
            
        else:
            commandString += "r000"
            print("Pan Good!")
    
        # Control Tilt
        if(tilt_deg > 0):
            print("Move up")
            avrCommand = self.up
            degreeShift = tilt_deg
            print("degreeShift = ", degreeShift)
            degreeShift = self.formatTheta(abs(int(degreeShift))) 
            commandString += avrCommand + degreeShift
            sendMessage = True
            
        elif(tilt_deg < 0):
            print("Move Down")
            avrCommand = self.down
            degreeShift = tilt_deg
            print("degreeShift = ", degreeShift)
            degreeShift = self.formatTheta(abs(int(degreeShift)))   
            commandString += avrCommand + degreeShift
            sendMessage = True
            
        else:
            commandString += "u000"
            print("Tilt Good!")
        
        
        if(sendMessage):
            commandString += "s"
            print(commandString)
            self.avrSerial.write((commandString).encode())
            
    def disconnect(self):
        self.avrSerial.close()
            
        
################
# Test Library #
################
if(False):
    myController = servoController("/dev/serial0", 4800)
    myController.updateAngles(30, -30)
    myController.disconnect()
