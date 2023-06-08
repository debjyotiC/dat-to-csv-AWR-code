import sys
import math
import numpy as np
import struct
import json
import matplotlib.pyplot as plt
from awr_2944_config_parser import AWRConfigparser
from scipy.fft import fft, fftshift
import random
from data_Writer import CSVWrite


def toInt32(x):
    if x > (2 ** 31 - 1):
        return x - 2 ** 32
    else:
        return x


def toInt16(x):
    if x > (2 ** 15 - 1):
        return x - 2 ** 16
    else:
        return x


class AWR2944DataExtractor:
    def __init__(self, configFileName):
        self.configFileName = configFileName
        self.configPhaser = AWRConfigparser(self.configFileName)
        self.configParameters = self.configPhaser.phrase_config_file()
        self.frameNum = 0
        self.subFrameNum = 0

    # match header and length field
    def match_header(self, Data):
        word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
        idX = 0  # Initialize the pointer index
        # Read the header
        Data = np.frombuffer(Data, dtype='uint8')
        magicNumber = Data[idX:idX + 8]
        print(magicNumber)
        idX += 8
        version = format(np.matmul(Data[idX:idX + 4], word), 'x')
        idX += 4
        totalPacketLen = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        platform = format(np.matmul(Data[idX:idX + 4], word), 'x')
        idX += 4
        frameNumber = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        timeCpuCycles = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        numDetectedObj = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        numTLVs = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        subFrameNumber = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        return idX, magicNumber, version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber

    # match TLVtype length and validate

    def match_length(self, Data):
        idX = 0
        Data = np.frombuffer(Data, dtype='uint8')
        word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
        tlv_type = np.matmul(Data[idX:idX + 4], word)  # Check the header of the TLV message
        idX += 4
        tlv_length = np.matmul(Data[idX:idX + 4], word)
        idX += 4
        return tlv_type, tlv_length

    def tlvHeaderDecode(self, data):
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    def detectPoint(self, Data, numDetectedObj):
        # Data = np.frombuffer(Data, dtype='uint8')
        detObj = {}
        x = np.zeros(numDetectedObj, dtype=np.float32)
        y = np.zeros(numDetectedObj, dtype=np.float32)
        z = np.zeros(numDetectedObj, dtype=np.float32)
        v = np.zeros(numDetectedObj, dtype=np.float32)
        r = np.zeros(numDetectedObj, dtype=np.float32)
        idX = 0
        # print('length'.format(len(Data)))
        for objectNum in range(numDetectedObj):
            x[objectNum], y[objectNum], z[objectNum], v[objectNum] = struct.unpack('4f', Data[idX:idX + 16])
            idX += 16
            # print(byteBuffer[idX:idX + 4].view(dtype=np.float32))
            # x corrdinate of detected object
            # x[objectNum] = np.round(
            #     Data[idX:idX + 4].view(dtype=np.float32), 4)
            # idX += 4
            # # y corrdinate of detected object
            # y[objectNum] = np.round(
            #     Data[idX:idX + 4].view(dtype=np.float32), 4)
            # idX += 4
            # # z corrdinate of detected object
            # z[objectNum] = np.round(
            #     Data[idX:idX + 4].view(dtype=np.float32), 4)
            # idX += 4
            # # dopler velocity (m/s) of detected object
            # v[objectNum] = np.round(
            #     Data[idX:idX + 4].view(dtype=np.float32), 4)
            # idX += 4

            # print('x: {}, y: {}, z: {}, v: {}'.format(x,y,z,v))
            # x[objectNum], y[objectNum], z[objectNum], v[objectNum]
            # peakVal, x, y, z = struct.unpack()
            r[objectNum] = np.round(math.sqrt(x[objectNum] * x[objectNum] + y[objectNum] *
                                              y[objectNum] + z[objectNum] * z[objectNum]),
                                    4)  # Radial distance of detected object
        # print('x: {}, y: {}, z: {}, v: {}'.format(x,y,z,v))
        print('\ntest\n')
        rangeIdx = r / self.configParameters['dataPath'][self.subFrameNum]['rangeIdxToMeters']
        dopplerIdx = v / self.configParameters['dataPath'][self.subFrameNum]['dopplerResolutionMps']
        detObj = {"numObj": float(numDetectedObj),
                  "x": x.astype('float'), "y": y.astype('float'), "z": z.astype('float'), "v": v.astype('float'),
                  'r': r.astype('float'), 'rangeIdx': rangeIdx, 'dopplerIdx': dopplerIdx}
        print(detObj)
        return detObj

    def rangeProfile(self, Data, tlvLength):
        Data = np.frombuffer(Data, dtype='uint8')
        data1 = []
        word = [1, 2 ** 8]
        idX = 0
        byteLength = self.configParameters['dataPath'][self.subFrameNum]['numRangeBins'] * 2
        print("byte length: {}, tlvlength: {}".format(byteLength, tlvLength))
        while idX < tlvLength:
            # for i in range(0, tlvLength, 2):
            temp = np.matmul(Data[idX:idX + 2], word)
            if temp > 32767:
                temp = temp - 65536
            data1.append(temp)
            idX += 2
        return data1

    def noiseProfile(self, Data, tlvLength):
        Data = np.frombuffer(Data, dtype='uint8')
        data1 = []
        word = [1, 2 ** 8]
        idX = 0
        while idX < tlvLength:
            # for i in range(0, tlvLength, 2):
            temp = np.matmul(Data[idX:idX + 2], word)
            if temp > 32767:
                temp = temp - 65536
            data1.append(temp)
            idX += 2
        return data1

    def azimuthStaticHeatmap_raw(self, Data, tlvLength):
        Data = np.frombuffer(Data, dtype='uint8')
        word = [1, 2 ** 8]
        idX = 0
        data = []
        while idX < tlvLength:
            data.append(np.matmul(Data[idX:idX + 2], word))
            idX += 2
        return data

    def azimuthStaticHeatmap(self, Data, tlvLength):
        q = np.frombuffer(Data, dtype='uint8')
        NUM_ANGLE_BINS = 64
        numBytes = self.configParameters['dataPath'][self.subFrameNum]['numTxAzimAnt'] * \
                   self.configParameters['dataPath'][self.subFrameNum]['numRxAnt'] * \
                   self.configParameters['dataPath'][self.subFrameNum]['numRangeBins'] * 4
        print('numbytes: {} \n tlvLength: {}'.format(numBytes, tlvLength))
        if int(numBytes) != tlvLength:
            raise "numbyte and tlvLength is not same"
        qrows = self.configParameters['dataPath'][self.subFrameNum]['numTxAzimAnt'] * \
                self.configParameters['dataPath'][self.subFrameNum]['numRxAnt']
        qcols = self.configParameters['dataPath'][self.subFrameNum]['numRangeBins']
        qrows = int(qrows)
        qcols = int(qcols)
        data = np.zeros((qcols, NUM_ANGLE_BINS))
        qidx = 0
        # print(qcols)
        # print(qrows)
        # print(data.shape)
        for tmpc in range(qcols):
            real = np.zeros((NUM_ANGLE_BINS,))
            imag = np.zeros((NUM_ANGLE_BINS,))
            for tmpr in range(qrows):
                real[tmpr] = q[qidx + 1] * 256 + q[qidx]
                if real[tmpr] > 32767:
                    real[tmpr] = real[tmpr] - 65536
                imag[tmpr] = q[qidx + 3] * 256 + q[qidx + 2]
                if imag[tmpr] > 32767:
                    imag[tmpr] = imag[tmpr] - 65536
                qidx = qidx + 4
            # print(real.shape)
            fftdata = np.fft.fft(real + 1j * imag, NUM_ANGLE_BINS)
            # print(fftdata[0])
            fftdata = abs(fftdata)
            # print(fftdata.shape)
            fftdata = np.fft.fftshift(fftdata)
            data[tmpc, :] = fftdata.reshape((1, NUM_ANGLE_BINS))
        # data1 = []
        # word = [1, 2**8]
        # for i in range(0, tlvLength, 2):
        #     temp = np.matmul(Data[i:i+2],word)
        #     if temp>32767:
        #         temp = temp-65536
        #     data1.append(temp)
        # data1 = struct.unpack(str(tlvLength//4)+'f', Data)
        # data = np.array(data1, dtype='int16')
        # print(data.shape)
        # data = 255*(data/32767)
        # data = data.astype('uint32')
        # # print(np.max(data))
        # cv2.imwrite('Azim_data'+str(random.random())+'.jpg', cv2.cvtColor(data, cv2.COLORMAP_JET))
        return data

    def rangeDopplerHeatmap(self, Data, tlvLength):
        Data = np.frombuffer(Data, dtype='uint8')
        word = [1, 2 ** 8]
        idX = 0
        data = []
        while idX < tlvLength:
            data.append(np.matmul(Data[idX:idX + 2], word))
            idX += 2
        return data

    # def rangeDopplerHeatmap(self, Data, tlvLength):  # <=== modify
    # idX = 0
    # Data = np.frombuffer(Data, dtype='uint8')
    # # Get the number of bytes to read
    # numBytes = 16384
    #
    # print('numbytes: {} \n tlvLength: {}'.format(numBytes, tlvLength))
    #
    # # Convert the raw data to int16 array
    # payload = Data[idX:idX + numBytes]
    # idX += numBytes
    # rangeDoppler = payload.view(dtype=np.int16)
    #
    # # Convert the range doppler array to a matrix
    # rangeDoppler = np.reshape(rangeDoppler,
    #                           (32,256),
    #                           'F')  # Fortran-like reshape
    # rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler) / 2):], rangeDoppler[:int(len(rangeDoppler) / 2)],
    #                          axis=0)

    # rangeDoppler = list()
    # for i in range(int(len(Data) / 2)):
    #     rangeDoppler.append(Data[2 * i] + Data[2 * i + 1] * 256)
    #
    # rangeDoppler = np.reshape(rangeDoppler, (16, 256),
    #                           'F')  # Fortran-like reshape
    # rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler) / 2):], rangeDoppler[:int(len(rangeDoppler) / 2)],
    #                          axis=0)
    #
    # return rangeDoppler

    def statistics(self, Data):
        Data = np.frombuffer(Data, dtype='uint8')
        idX = 0
        word32 = [1, 2 ** 8, 2 ** 16, 2 ** 24]
        nterFrameProcessingTime = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        transmitOutputTime = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        interFrameProcessingMargin = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        interChirpProcessingMargin = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        activeFrameCPULoad = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        interFrameCPULoad = np.matmul(Data[idX:idX + 4], word32)
        idX += 4
        statics = {'nterFrameProcessingTime': float(nterFrameProcessingTime),
                   'transmitOutputTime': float(transmitOutputTime),
                   'interFrameProcessingMargin': float(interFrameProcessingMargin),
                   'interChirpProcessingMargin': float(interChirpProcessingMargin),
                   'activeFrameCPULoad': float(activeFrameCPULoad), 'interFrameCPULoad': float(interFrameCPULoad)}
        return statics

    def sideInfoForDetectedPoints(self, Data, numDetectedObj):
        Data = np.frombuffer(Data, dtype='uint8')
        snr = np.zeros(numDetectedObj, dtype=np.float32)
        noise = np.zeros(numDetectedObj, dtype=np.float32)
        """The values for snr and noise are measured in multiples of 0.1dB."""
        idX = 0
        word = [1, 2 ** 8]
        for i in range(numDetectedObj):
            temp = np.matmul(Data[idX:idX + 2], word)
            if temp > 32767:
                temp = temp - 65536
            snr[i] = temp * 0.1
            temp = np.matmul(Data[idX + 2:idX + 4], word)
            if temp > 32767:
                temp = temp - 2 ** 65536
            noise[i] = temp * 0.1
            idX += 4
        sideinfo = {'snr': snr, 'noise': noise}
        return sideinfo

    def azimuthElevationStaticHeatmap(self, Data, tlvLength):

        # Data = np.frombuffer(Data, dtype='uint8')
        # Data = Data.view(dtype = np.float32)
        # Data = Data.astype('float')

        Data = np.frombuffer(Data, dtype='uint8')
        word = [1, 2 ** 8]
        idX = 0
        data = []
        while idX < tlvLength:
            data.append(np.matmul(Data[idX:idX + 2], word))
            idX += 2
        return data

        # return Data

    def temperatureStatistics(self, Data):
        Data = np.frombuffer(Data, dtype='uint8')
        idX = 0
        word32 = [1, 2 ** 8, 2 ** 16, 2 ** 24]
        word16 = [1, 2 ** 8]
        tempReportValid = toInt32(np.matmul(Data[idX:idX + 4], word32))  # (used to know if values are valid)	uint32_t	4
        idX += 4
        time = toInt32(
            np.matmul(Data[idX:idX + 4], word32))  # (radarSS local Time from device powerup) [1LSB = 1ms]	uint32_t	4
        idX += 4
        tmpRx0Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpRx1Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpRx2Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpRx3Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpTx0Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpTx1Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpTx2Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpPmSens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpDig0Sens = toInt16(np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C]	uint16_t	2
        idX += 2
        tmpDig1Sens = toInt16(
            np.matmul(Data[idX:idX + 2], word16))  # [1 LSB = 1 deg C] (Not valid for devices without DSP)	uint16_t	2

        temperatureStatistics = {'tempReportValid': tempReportValid, 'time': time,
                                 'tmpRx0Sens': tmpRx0Sens, 'tmpRx1Sens': tmpRx1Sens, 'tmpRx2Sens': tmpRx2Sens,
                                 'tmpRx3Sens': tmpRx3Sens,
                                 'tmpTx0Sens': tmpTx0Sens, 'tmpTx1Sens': tmpTx1Sens, 'tmpTx2Sens': tmpTx2Sens,
                                 'tmpPmSens': tmpPmSens, 'tmpDig0Sens': tmpDig0Sens, 'tmpDig1Sens': tmpDig1Sens}

        return temperatureStatistics

    def writeJson(self, data):
        # json_object = json.dumps(data)

        # Writing to sample.json
        with open("sample.json", "a+") as outfile:
            json.dump(data, outfile)
        # with open("sample.json", "a") as outfile:
        #     outfile.write(json_object, cls = NpEncoder)

    def tlvHeader(self, data):
        while data:
            headerLength = 36
            try:
                magic, version, length, platform, frameNum, cpuCycles, numObj, numTLVs = struct.unpack('Q7I', data[
                                                                                                              :headerLength])
            except:
                print("Improper TLV structure found: ")
                # print(data)
                break
            print("*********************************")
            print("Packet ID:\t%d " % (frameNum))
            print("length:\t%d " % (length))
            print("Version:\t%x " % (version))
            print("TLV:\t\t%d " % (numTLVs))
            print("Detect Obj:\t%d " % (numObj))
            print("Platform:\t%X " % (platform))
            self.frameNum = frameNum
            print("length Data")
            print(len(data))
            # print(data)
            # if version > 0x01000005:
            json_data = {}
            json_data["Packet ID"] = frameNum
            json_data["length"] = length
            json_data["TLV"] = numTLVs
            json_data["number of detected object"] = numObj
            if len(data) > length:
                subFrameNum = struct.unpack('I', data[36:40])[0]
                self.subFrameNum = subFrameNum
                json_data["subFrameNum"] = subFrameNum
                headerLength = 40
                print("Subframe:\t%d " % (subFrameNum))
                pendingBytes = length - headerLength
                data = data[headerLength:]
                for i in range(numTLVs):
                    tlvType, tlvLength = self.tlvHeaderDecode(data[:8])
                    print("tlvtype: {}\ntlvLength: {}".format(tlvType, tlvLength))
                    data = data[8:]
                    IdX = 0
                    if tlvType == 1:
                        # print(tlvType)
                        json_data['detectPoint'] = self.detectPoint(data[:tlvLength], numObj)  # Dictionary
                        # IdX += tlvLength
                    elif tlvType == 2:
                        # print(tlvType)
                        json_data['rangePoint'] = self.rangeProfile(data[:tlvLength], tlvLength)  # Data
                    elif tlvType == 3:
                        json_data['noisData'] = self.noiseProfile(data[:tlvLength], tlvLength)  # Data
                    elif tlvType == 4:
                        json_data['azimData'] = self.azimuthStaticHeatmap_raw(data[:tlvLength], tlvLength)  # Data
                        # plt.imshow(json_data['azimData'])
                        # plt.savefig('Azim_data'+str(random.random())+'.jpg')
                    elif tlvType == 5:
                        json_data['dopplerdata'] = self.rangeDopplerHeatmap(data[:tlvLength], tlvLength)  # Data
                    elif tlvType == 6:
                        json_data['Statistics'] = self.statistics(data[:tlvLength])  # Dictionary
                    elif tlvType == 7:
                        json_data['sideInfooDetectedPoint'] = self.sideInfoForDetectedPoints(data[:tlvLength],
                                                                                             numObj)  # Dictionary
                    elif tlvType == 8:
                        json_data['azimElevData'] = self.azimuthElevationStaticHeatmap(data[:tlvLength], tlvLength)
                    elif tlvType == 9:
                        json_data['tempStat'] = self.temperatureStatistics(data[:tlvLength])  # Dictionary
                    else:
                        print(f"Unidentified tlv type {tlvType}")
                    data = data[tlvLength:]
                    pendingBytes -= (8 + tlvLength)
                # break
                data = data[pendingBytes:]
                yield length, frameNum, json_data
            else:
                break


if __name__ == "__main__":
    configFileName = 'your_cfg_file.cfg'
    csv_writer = CSVWrite(csvPath=None, csvFileName='name_of_csv_file.csv')
    data_extractor = AWR2944DataExtractor(configFileName)
    fileName = 'your_radar_dat_file.dat'
    rawDataFile = open(fileName, "rb")
    rawData = rawDataFile.read()
    rawDataFile.close()
    magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'
    offset = rawData.find(magic)
    rawData = rawData[offset:]
    for length, frameNum, json_data in data_extractor.tlvHeader(rawData):
        print(json_data)
        csv_writer.write_data(json_data)
