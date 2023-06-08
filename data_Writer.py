import csv
from curses import textpad
import os
import time
import numpy as np
import cv2
import datetime

"""
CSV Writer Class
"""


class CSVWrite:
    def __init__(self, csvPath, csvFileName):
        if csvPath is not None:
            self.csvFileName = os.path.join(csvPath, csvFileName)
        else:
            self.csvFileName = csvFileName

    def check_csv_file(self):
        if os.path.isfile(os.path.join('CSV', self.csvFileName)):
            return True
        else:
            return False

    def write_data(self, data):
        frameNumber = data["Packet ID"]
        numDetectedObj = data["number of detected object"]
        if self.check_csv_file():
            with open(os.path.join('CSV', self.csvFileName), 'a+') as csvfile:
                # creating a csv writer object
                csvwriter = csv.writer(csvfile)
                # csvwriter.writerows([['', "Packet ID", data['Packet ID']]])
                # csvwriter.writerows([['', "subFrameNum", data['subFrameNum']]])
                # csvwriter.writerows([['', "number of detected object", numDetectedObj]])
                # detObj = {"numObj": float(numDetectedObj), "x": x.astype('float'), "y": y.astype('float'),
                # "z": z.astype('float'), "v": v.astype('float'), 'r': r.astype('float'), 'rangeIdx': rangeIdx,
                # 'dopplerIdx': dopplerIdx}

                if 'detectPoint' in data.keys():
                    csvwriter.writerow(['', "detectPoint"])
                    # if data['dpOk']: #"""To DO add guimonitor condition"""
                    x = [['', '', 'x:'] + list(data['detectPoint']['x']),
                         ['', '', 'y:'] + list(data['detectPoint']['y']),
                         ['', '', 'z:'] + list(data['detectPoint']['z']),
                         ['', '', 'r:'] + list(data['detectPoint']['r']),
                         ['', '', 'v:'] + list(data['detectPoint']['v'])]
                    # y = ['','', 'y:']+list(data['detectPoint']['y'])
                    # z = ['','', 'z:']+list(data['detectPoint']['z'])
                    # r  = ['','', 'r:']+list(data['detectPoint']['r'])
                    # v  = ['','', 'v:']+list(data['detectPoint']['v'])
                    csvwriter.writerows(x)
                if 'rangePoint' in data.keys():
                    csvwriter.writerow(['', "rangePoint"])
                    csvwriter.writerows(
                        [['', '', 'data'] + list(data['rangePoint'])])
                if 'noisData' in data.keys():
                    csvwriter.writerow(['', "noisData"])
                    csvwriter.writerows(
                        [['', '', 'data'] + list(data['noisData'])])
                if 'azimData' in data.keys():
                    csvwriter.writerow(['', "azimData"])
                    csvwriter.writerows(
                        [['', '', 'data'] + list(data['azimData'])])
                if 'dopplerdata' in data.keys():
                    # csvwriter.writerow(['', "dopplerdata"])
                    csvwriter.writerows([['', '', 'data'] + list(data['dopplerdata'])])
                if 'azimElevData' in data.keys():
                    csvwriter.writerow(['', "azimElevData"])
                    csvwriter.writerows(
                        [['', '', 'data'] + list(data['azimElevData'])])
                if 'sideInfooDetectedPoint' in data.keys():
                    csvwriter.writerow(['', "sideInfooDetectedPoint"])
                    csvwriter.writerows(
                        [['', '', 'snr'] + list(data['sideInfooDetectedPoint']['snr']),
                         ['', '', 'noise'] + list(data['sideInfooDetectedPoint']['noise'])])
                if 'dopplerdata' in data.keys():
                    # csvwriter.writerow(['', "dopplerdata"])
                    csvwriter.writerows(
                        [['', '', 'data'] + list(data['dopplerdata'])])
                if '"tempStat"' in data.keys():
                    csvwriter.writerow(['', "tempStat"])
                    csvwriter.writerow([['', '', 'tempReportValid', 'time', 'tmpRx0Sens', 'tmpRx1Sens', 'tmpRx2Sens',
                                         'tmpRx3Sens', 'tmpTx0Sens', 'tmpTx1Sens', 'tmpTx2Sens', 'tmpPmSens',
                                         'tmpDig0Sens', 'tmpDig1Sens'],
                                        ['', '', data['tempStat']['tempReportValid'],
                                         data['tempStat']['time'],
                                         data['tempStat']['tmpRx0Sens'],
                                         data['tempStat']['tmpRx1Sens'],
                                         data['tempStat']['tmpRx2Sens'],
                                         data['tempStat']['tmpRx3Sens'],
                                         data['tempStat']['tmpTx0Sens'],
                                         data['tempStat']['tmpTx1Sens'],
                                         data['tempStat']['tmpTx2Sens'],
                                         data['tempStat']['tmpPmSens'],
                                         data['tempStat']['tmpDig0Sens'],
                                         data['tempStat']['tmpDig1Sens']]])
                # if data['sifdpOk']: #"""To DO add guimonitor condition"""
                #     snr = [['','', 'snr']+list(data['SIDE_INFO_FOR_DETECTED_POINTS']['snr'])]
                #     noise = [['','', 'noise']+list(data['SIDE_INFO_FOR_DETECTED_POINTS']['noise'])]
                #     csvwriter.writerows(snr)
                #     csvwriter.writerows(snr)
        else:
            with open(os.path.join('CSV', self.csvFileName), 'a+') as csvfile:
                # creating a csv writer object
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows([['']])


"""
TXT Writer class
"""


class TXTWrite:
    def __init__(self, txtPath, txtFileName):
        if txtPath is not None:
            self.txtFileName = os.path.join(txtPath, txtFileName)
        else:
            self.txtFileName = txtFileName

    def check_txt_file(self):
        if os.path.isfile(os.path.join('TXT', self.txtFileName)):
            return True
        else:
            return False

    def write_data(self, data):
        frameNumber = data['frameNumber']
        subFrameNumber = data['subFrameNumber']
        magicOk = data['magicOK']
        if magicOk:
            numDetectedObj = data['numDetectedObj']
            if self.check_txt_file():
                with open(os.path.join('TXT', self.txtFileName), 'a+') as file:
                    timestamp = time.time()
                    secs = int(timestamp)
                    nsecs = int((timestamp - secs) * 1000000000)
                    file.write('------\n')
                    file.write('Header\n')
                    file.write('TimeStamp:\n')
                    file.write('secs:' + str(secs) + '\n')
                    file.write('nsecs:' + str(nsecs) + '\n')
                    file.write('frameNumber:' + str(frameNumber) + '\n')
                    file.write('subFrameNumber' + str(subFrameNumber) + '\n')
                    file.write('NumberOfDetectedObject:' +
                               str(numDetectedObj) + '\n')
                    file.write('Information:\n')

                    if data['dpOk']:  # """To DO add guimonitor condition"""
                        x = 'x:' + \
                            ','.join(
                                map(str, list(data['DETECTED_POINTS']['x'])))
                        y = 'y:' + \
                            ','.join(
                                map(str, list(data['DETECTED_POINTS']['y'])))
                        z = 'z:' + \
                            ','.join(
                                map(str, list(data['DETECTED_POINTS']['z'])))
                        r = 'r:' + \
                            ','.join(
                                map(str, list(data['DETECTED_POINTS']['r'])))
                        v = 'v:' + \
                            ','.join(
                                map(str, list(data['DETECTED_POINTS']['v'])))
                        file.write(x + '\n')
                        file.write(y + '\n')
                        file.write(z + '\n')
                        file.write(r + '\n')
                        file.write(v + '\n')
                    if data['sifdpOk']:  # """To DO add guimonitor condition"""
                        snr = 'snr:' + \
                              ','.join(
                                  map(str, list(data['SIDE_INFO_FOR_DETECTED_POINTS']['snr'])))
                        noise = 'noise:' + \
                                ','.join(
                                    map(str, list(data['SIDE_INFO_FOR_DETECTED_POINTS']['noise'])))
                        file.write(snr + '\n')
                        file.write(noise + '\n')
            else:
                with open(os.path.join('TXT', self.txtFileName), 'a+') as file:
                    file.write('\n')


"""
Image Writer
"""


class IMAGEWrite:
    def __init__(self, imagePath, imageFileNameStartWith):
        if imagePath is not None:
            self.imageFileNameStartWith = os.path.join(
                imagePath, imageFileNameStartWith)
        else:
            self.imageFileNameStartWith = imageFileNameStartWith

    def write_data(self, data):
        frameNumber = data['frameNumber']
        subFrameNumber = data['subFrameNumber']
        magicOk = data['magicOK']
        aseOK = data['asemOk']  # """To DO add guimonitor condition"""
        if magicOk:
            if aseOK:
                im = cv2.imwrite(self.imageFileNameStartWith + '_' + str(frameNumber) + '_' + str(
                    subFrameNumber) + '.png', data['AZIMUT_ELEVATION_STATIC_HEAT_MAP'])


"""
Main Data Writer
"""


class DATAWriter:
    def __init__(self, csvWriterEnable, textWriterEnable, imageWriterEnable):
        self.textWriterEnable = textWriterEnable
        self.csvWriterEnable = csvWriterEnable
        # To DO: Add extra condition for image writing with respect to configuration parameters for noise profile, azimuth elecation heat map. etc
        self.imageWriterEnable = imageWriterEnable
        self.imagePath = None
        self.csvPath = None
        self.txtPath = None
        self.txtFileName = datetime.datetime.now().strftime("%c") + '.txt'
        self.csvFileName = datetime.datetime.now().strftime("%c") + '.csv'
        # image file name will be AZimuthData_frameNumber_subframeNumber.png/jpg
        self.imageFileNameStartWith = "AzimuthData_"
        self.dataDirectory = self.generateDirectoryName('results', 'exp', x=0)
        self.checkDirectory()
        self.csvWriter = CSVWrite(self.csvPath, self.csvFileName)
        self.txtWriter = TXTWrite(self.txtPath, self.txtFileName)
        self.imageWriter = IMAGEWrite(
            self.imagePath, self.imageFileNameStartWith)

    def checkDirectory(self):
        if self.csvWriterEnable:
            csvPath = os.path.join(self.dataDirectory, "CSV")
            # if not os.path.exists(csvPath):
            os.mkdir(csvPath)
            self.csvPath = csvPath
        if self.textWriterEnable:
            txtPath = os.path.join(self.dataDirectory, "TXT")
            # if not os.path.exists(txtPath):
            os.mkdir(txtPath)
            self.txtPath = txtPath
        if self.imageWriterEnable:
            imagePath = os.path.join(self.dataDirectory, "IMAGE")
            # if not os.path.exists(imagePath):
            os.mkdir(imagePath)
            self.imagePath = imagePath

    def generateDirectoryName(self, parrentDirectory, name, x=0):
        if not os.path.exists(parrentDirectory):
            os.mkdir(parrentDirectory)
        while True:
            dir_name = (name + (str(x) if x is not 0 else '')).strip()
            dirpath = os.path.join(parrentDirectory, dir_name)
            if not os.path.exists(dirpath):
                os.mkdir(dirpath)
                return dirpath
            else:
                x = x + 1

    def createImageDataSubDirectory(self):
        """
        Create image subdirectory based on gui monitor
        """
        pass

    def dataWrite(self, data):
        if self.textWriterEnable:
            self.txtWriter.write_data(data)
        if self.csvWriterEnable:
            self.csvWriter.write_data(data)
        if self.imageWriterEnable:
            self.imageWriter.write_data(data)


if __name__ == "__main__":
    pass
