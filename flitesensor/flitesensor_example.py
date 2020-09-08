import time
from flitecore import FliteSensor
'''
Create flitesensor object
Argument 1 - The color of the Flite sensor (BLACK, BLUE, RED, GREEN), BLACK used in this example
Argument 2 - A directory to store the sensor calibration data
'''
fliteSensor = FliteSensor("BLACK", "/home/pi/Documents/FliteCal/")

#Begin the flitesensor object
fliteSensor.beginSensor()

'''
These functions will need to be called by a calibration interface not demonstrated in this example, and will be specific to your interface
Calibration must be performed before sensor provides accurate gallons
Invoke the calibration high and calibration low functions to calibrate the level sensor
Invoke the zeroPSI function to zero the pressure sensor to ambient pressure
Temperature does not require any configuration, and returns the temperature in Degrees F
'''
#fliteSensor.calibrateLow(0.0)
#fliteSensor.calibrateHigh(5.0)

#fliteSensor.calibrateZeroPSI()

'''
Update the sensor data every 5 seconds
'''
while True:
    print("Level:",fliteSensor.getLevel()," Gallons")
    print("Temperature:",fliteSensor.getTemperature()," Deg.F")
    print("Pressure:",fliteSensor.getPressure()," PSI")
    time.sleep(5)

