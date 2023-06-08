import os
import time
import serial

class AWRConfigWriter:
    def __init__(self, configFileName):
        self.dataPortConn = None
        self.controlPortConn = None
        self.configFileName = configFileName
        self.config = None

    # Check presence of configuration file
    def Check_Config_File(self):
        if os.path.isfile(self.configFileName):
            if os.path.splitext(self.configFileName)[1]=='.cfg': # Check Configuration file extension
                print('Configuration file extention in proper format')
                return True
            else:
                print('Configuration file extention not in proper format')
                return False
        else:
            print('Check the configuration file or configuration file not present')
            return False

    # SensorStart
    def SensorStart(self):
        try:
            self.controlPortConn.write(('sensorStart'+'\n').encode())
            return True
        except:
            print("Skipped sensorStop\n")
            return False

    # Sensor Stop
    def SensorStop(self):
        try:
            self.controlPortConn.write(('sensorStop\n').encode())
            print('sensorStop\n')
            return True
        except:
            print("Skipped sensorStop\n")
            return False

    # Read Config File
    def Read_Config_File(self):
        if self.Check_Config_File(): # check config file present or not
            config = [line.rstrip('\r\n') for line in open(self.configFileName)]
            # print(config)
            return config
        else:
            print('Check the configuration file or configuration file not present')
            return None

    # Send config file to AWR2944
    def Config_Write(self,controlPortDev, dataPortDev):
        controlPortConn = None
        dataPortConn = None
        os.system('echo %s|sudo -S %s' % ('zreyas123@', 'sudo chmod a+rw '+controlPortDev))
        os.system('echo %s|sudo -S %s' % ('zreyas123@', 'sudo chmod a+rw '+dataPortDev))
        controlPortConn = serial.Serial(controlPortDev, 115200)
        dataPortConn= serial.Serial(dataPortDev,852272)
        self.config = self.Read_Config_File()    # Verify configuration file
        if self.config is not None:          # Check config file Data
            for i in self.config:
                    controlPortConn.write((i+'\n').encode()) # Writing configuration data to control port
                    print(f' {i}\n')
                    time.sleep(0.01)

        return controlPortConn, dataPortConn
