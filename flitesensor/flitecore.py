#Python library for Flite keg sensor

#**Hardware Documentation
#Pressure Temperature Sensor I2C Documentation - https://sensing.honeywell.com/i2c-comms-digital-output-pressure-sensors-tn-008201-3-en-final-30may12.pdf
#Distance Sensor API Documentation - https://www.st.com/en/embedded-software/stsw-img005.html

#**This library was developed using the below library developed by John Bryan Moore
#https://github.com/johnbryanmoore/VL53L0X_rasp_python

#Author: DJMarlow - https://github.com/DJMarlow
#Date: 2020-8-26
#Version 1.0.0
        
#MIT License

#Copyright (c) 2020 Derrick Marlow

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import time
import os
from smbus2 import SMBus, i2c_msg
import VL53L0X

class FliteSensor():
    def __init__(self, color, fileDirectory):
        self.color = color
        self.fileDistanceLow = fileDirectory + "distanceLow.txt"
        self.fileLevelLow = fileDirectory + "levelLow.txt"
        self.fileDistanceHigh = fileDirectory + "distanceHigh.txt"
        self.fileLevelHigh = fileDirectory + "levelHigh.txt"
        self.filePSIZero = fileDirectory + "pressureZero.txt"
        self.level = 0.0
        self.press = 0.0
        self.temp = 0

        if self.color == "BLACK":
            self.VL530X_Address = 0x30
            self.HSC_Address = 0x48
        if self.color == "BLUE":
            self.VL530X_Address = 0x31
            self.HSC_Address = 0x58
        if self.color == "RED":
            self.VL530X_Address = 0x32
            self.HSC_Address = 0x68
        if self.color == "GREEN":
            self.VL530X_Address = 0x33
            self.HSC_Address = 0x78

    def beginSensor(self):
        self.lox = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        #self.lox.change_address(self.VL530X_Address)

    def getTOFDistance(self):
        self.lox.open()
        self.lox.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        self.distance = self.lox.get_distance()
        self.lox.stop_ranging()
        self.lox.close()
        return self.distance
    
    def getLevel(self):
        l = 0.0
        d = self.getTOFDistance()
        
        #If d > 1000 discard reading
        if d < 1000:
            l = self.level
        else:
            m =  (self.getCalibrationLevelHigh() - self.getCalibrationLevelLow()) / (self.getCalibrationDistanceHigh() - self.getCalibrationDistanceLow())
            b = self.getCalibrationLevelHigh() - (m * self.getCalibrationDistanceHigh())
            l = (m * d) + b

            #Limit level reading from 0 - 5
            if l < 0.0:
                l = 0.0

            if l > 5.0:
                l = 5.0

            self.level = l
        
        return l

    def calibrateLow(self, l):
        #Store the distance value in file
        self.setCalibrationDistanceLow(self.getTOFDistance())
        #Store the corresponding level value in file
        self.setCalibrationLevelLow(l)

    def setCalibrationDistanceLow(self, d):
        file = open(self.fileDistanceLow, 'w')
        file.write(str(d))
        file.close()

    def getCalibrationDistanceLow(self):
        #If d is null initialize cal distance at 600 mm away
        if os.stat(self.fileDistanceLow).st_size == 0:
            d = 600.0
            return d
        else:
            file = open(self.fileDistanceLow)
            d = file.read()
            return float(d)

    def setCalibrationLevelLow(self, l):
        file = open(self.fileLevelLow, 'w')
        file.write(str(l))
        file.close()

    def getCalibrationLevelLow(self):
        #If level is null initialize level at 0
        if os.stat(self.fileLevelLow).st_size == 0:
            l = 0.0
            return l
        else:
            file = open(self.fileLevelLow)
            l = file.read()
            file.close()
            return float(l)

    def calibrateHigh(self, l):
        #Store the distance value in file
        self.setCalibrationDistanceHigh(self.getTOFDistance())
        #Store the corresponding level value in file
        self.setCalibrationLevelHigh(l)

    def setCalibrationDistanceHigh(self, d):
        file = open(self.fileDistanceHigh, 'w')
        file.write(str(d))
        file.close()

    def getCalibrationDistanceHigh(self):
        #If d is null initialize cal distance at 100 mm away
        if os.stat(self.fileDistanceHigh).st_size == 0:
            d = 100.0
            return d
        else:
            file = open(self.fileDistanceHigh)
            d = file.read()
            return float(d)

    def setCalibrationLevelHigh(self, l):
        file = open(self.fileLevelHigh, 'w')
        file.write(str(l))
        file.close()

    def getCalibrationLevelHigh(self):
        #If level is null initialize level at 5
        if os.stat(self.fileLevelHigh).st_size == 0:
            l = 5.0
            return l
        else:
            file = open(self.fileLevelHigh)
            l = file.read()
            file.close()
            return float(l)

    def calibrateZeroPSI(self):
        self.setCalibrationZeroPSI(self.getRawPressure())

    def setCalibrationZeroPSI(self, p):
        file = open(self.filePSIZero, 'w')
        file.write(str(p))
        file.close()

    def getCalibrationZeroPSI(self):
        #If p is null initialize zero pressure at 0
        if os.stat(self.filePSIZero).st_size == 0:
            p = 0.0
            return p
        else:
            file = open(self.filePSIZero)
            p = file.read()
            return float(p)

    def getRawPressTemp(self, addr):
        with SMBus(1) as bus:
            bus.write_quick(addr)
            msg = i2c_msg.read(addr, 4)
            bus.i2c_rdwr(msg)
            
        bytes = []
            
        for b in msg:
            bytes.append(b)

        self.status =  (bytes[0] & 0xFF00) >> 6
        self.press = ((bytes[0] & 0x00FF) << 8) + bytes[1]
        self.temp = (((bytes[2] & 0x00FF) << 8) + ((bytes[3] & 0xFF00) >> 5))

    def convertPressure(self, p, oMin, oMax, pMin, pMax):
        pNew = ((p * 1.0) - oMin) * (pMax - pMin) / (oMax - oMin) + pMin
        return pNew

    def convertTemp(self, t):
        tNew = ((t / 2047.0) * 200) - 50
        return tNew

    #Get the raw pressure
    def getRawPressure(self):
        self.getRawPressTemp(self.HSC_Address)
        pNew = self.convertPressure(self.press, 0x666, 0x399A, 0.0, 150.0)
        return pNew

    #Get the corrected pressure based on zero cal
    def getPressure(self):
        p = self.getRawPressure() - self.getCalibrationZeroPSI()
        self.press = p
        return p

    def getTemperature(self):
        self.getRawPressTemp(self.HSC_Address)
        t = self.convertTemp(self.temp)
        t = (t * 9/5) + 32
        self.temp = t
        return t
        
        
