"""
TODO:
    1. Phrase Config File
"""
import os
import math
import numpy


class AWRConfigparser:
    def __init__(self, configFileName):
        self.configFileName = configFileName  # configuration file Name
        self.config = None
        self.maxNumSubframes = 4
        self.subFrameNumInvalid = -1
        self.platform = 'AWR294X'
        self.sdkVersionUint16 = 0x0402
        self.profileCfgCounter = 0
        self.chirpCfgCounter = 0
        self.gProcChain = 'TDM'
        self.numTxAzimAnt = 0
        self.totalSubframes = 0

    # Check presence of configuration file
    def getConfigParameters(self):
        return self.P

    def checkConfigFile(self):
        if os.path.isfile(self.configFileName):
            # Check Configuration file extension
            if os.path.splitext(self.configFileName)[1] == '.cfg':
                print('Configuration file present')
                return True
            else:
                print('Configuration file extention not in proper format')
                return False
        else:
            print('Check the configuration file or configuration file not present')
            return False

    # Read Config File
    def readConfigFile(self):
        if self.checkConfigFile():  # check config file present or not
            config = [line.rstrip('\r\n') for line in open(self.configFileName)]
            return config
        else:
            print('Check the configuration file or configuration file not present')
            return None

    def checkSubFrameIdx(self, subFrameNum, command, dynamicFlg=None):
        error = ""
        R_Type = False
        if self.P['dfeDataOutputMode']['mode'] == 1:
            # /* legacy frame config*/
            if subFrameNum != self.subFrameNumInvalid:
                error = str(command) + \
                            " SubFrameIdx must be set to -1 (i.e. N/A)."
                R_Type = False
            else:
                R_Type = True
        elif self.P['dfeDataOutputMode']['mode'] == 3:
            if (subFrameNum >= self.maxNumSubframes) or (subFrameNum < -1):
                error = str(command) + " SubFrameIdx is invalid."
                R_Type = False
            else:
                R_Type = True
        else:
             error = "Make sure dfeDataOutputMode has been configured before " + \
                 str(command) + ". dfeDataOutputMode must be set to either 1 or 3."
             R_Type = False
        return R_Type, error

    # Phrase Config Information
    def phrase_config_file(self):
        self.P = {
            'channelCfg': {}, 'dataPath': {}, 'profileCfg': {}, 'frameCfg': {}, 'guiMonitor': {},
            'dfeDataOutputMode': {}, 'advFrameCfg': {'frameData': {}}, 'subFrameCfg': {}, 'chirpCfg': {},
            'subFrameInfo': {}, 'log2linScale': {}, 'platform': self.platform, 'cmdReceivedFlag': {},
            'numDetectedObj': {}, 'dspFftScaleComp2D_lin': {}, 'dspFftScaleComp2D_log': {}, 'dspFftScaleComp1D_lin': {},
            'dspFftScaleComp1D_log': {}, 'dspFftScaleCompAll_lin': {}, 'dspFftScaleCompAll_log': {},
            'interFrameProcessingTime': {}, 'transmitOutputTime': {}, 'interFrameProcessingMargin': {},
            'interChirpProcessingMargin': {}, 'activeFrameCPULoad': {}, 'interFrameCPULoad': {},
            'compRxChanCfg': {}, 'measureRxChanCfg': {}, 'bpmCfg': {}, 'nearFieldCfg': {}, 'aoaFovCfg': {},
            'cfarRangeFov': {}, 'cfarDopplerFov': {}, 'extendedMaxVelocity': {}
            }
        # /*initialize variables*/
        for i in range(self.maxNumSubframes):
            # /*data path*/
            self.P['dataPath'][i] = {
                'numTxAzimAnt': 0,
                'numTxElevAnt': 0,
                'numRxAnt': 0,
                'azimuthResolution': 0,
                'numChirpsPerFrame': 0,
                'numDopplerBins': 0,
                'numRangeBins': 0,
                'rangeResolutionMeters': 0,
                'rangeMeters': 0,
                'velocityMps': 0,
                'dopplerResolutionMps': 0,
                'numDopplerChirps': 0
                }

            # /*log2lin*/
            self.P['log2linScale'][i] = 0

            # /*max vel*/
            self.P['extendedMaxVelocity'][i] = {
                'enable': 0
                }

            # /*gui monitor*/
            self.P['guiMonitor'][i] = {
                'subFrameIdx': 0,
                'detectedObjects': 0,
                'logMagRange': 0,
                'noiseProfile': 0,
                'rangeAzimuthHeatMap': 0,
                'rangeDopplerHeatMap': 0,
                'statsInfo': 0
                }

        self.P['dfeDataOutputMode']['mode'] = 0
        self.P['configErrorFlag'] = 0
        lines = self.readConfigFile()
        # print(lines)
        
        if lines is None:
            # print(lines)
            return None
        # print(lines)
        self.validateCfg(lines, False)
        
        self.P['subFrameToPlot'] = self.subframeNumberToPlot()
        self.P['detectedObjectsToPlot'] = self.checkDetectedObjectsSetting()

        if (self.P['dfeDataOutputMode']['mode'] == 1):
            self.totalSubframes = 1
        elif (self.P['dfeDataOutputMode']['mode'] == 3):
            self.totalSubframes = self.P['advFrameCfg']['numOfSubFrames']
        # print(self.P)
        for idx in range(self.totalSubframes):
            profileCfgIdx = self.getProfileIdx(idx)
            self.P['subFrameInfo'][idx] = {'profileCfgIndex': profileCfgIdx}

            if (profileCfgIdx == -1):
                self.P['configErrorFlag'] = 1
                return

            if (self.getAntCfg(idx) == -1):
                self.P['configErrorFlag'] = 1
                return
            self.P['dataPath'][idx]['numTxAnt'] = self.P['dataPath'][idx]['numTxElevAnt'] + \
                self.P['dataPath'][idx]['numTxAzimAnt']
            if ((self.P['dataPath'][idx]['numTxAnt'] == 1) and (self.P['dataPath'][idx]['numRxAnt'] == 1) and
                (self.P['guiMonitor'][idx]['rangeAzimuthHeatMap'] == 1)):
                self.P['configErrorFlag'] = 1
                return

            if ((self.P['dataPath'][idx]['numRxAnt'] * self.P['dataPath'][idx]['numTxAzimAnt'] < 2)):
                self.P['dataPath'][idx]['azimuthResolution'] = 'None'
            else:
                self.P['dataPath'][idx]['azimuthResolution'] = round(math.asin(
                    2 / (self.P['dataPath'][idx]['numRxAnt'] * self.P['dataPath'][idx]['numTxAzimAnt'])) * 180 / 3.1415926, 1)

            if (self.P['dfeDataOutputMode']['mode'] == 1):
                    self.P['dataPath'][idx]['numChirpsPerFrame'] = (
                        self.P['frameCfg']['chirpEndIdx'] - self.P['frameCfg']['chirpStartIdx'] + 1) * self.P['frameCfg']['numLoops']
            else:
                self.P['dataPath'][idx]['numChirpsPerFrame'] = self.P['subFrameCfg'][idx]['numOfChirps'] * \
                    self.P['subFrameCfg'][idx]['numLoops']
            # print("\n\n**************************\n\n")
            # print(self.P['dataPath'])
            # print("\n\n**************************\n\n")
            if (self.gProcChain == 'DDM'):
                self.P['dataPath'][idx]['numDopplerChirps'] = self.P['dataPath'][idx]['numChirpsPerFrame']
            else:
                self.P['dataPath'][idx]['numDopplerChirps'] = self.P['dataPath'][idx]['numChirpsPerFrame'] / \
                    self.P['dataPath'][idx]['numTxAnt']
            # /*For 68xx (SDK 3.1) and 18xx demos (SDK 3.1 onwards)  when number of doppler chirps <= 4, the number of doppler bins is hardcoded
            #   to 8 by the demo. See Jira MMWSDK-1565.*/
            if ((self.platform == 'AWR294X') and (self.sdkVersionUint16 >= 0x0301) and (self.P['dataPath'][idx]['numDopplerChirps'] <= 4)):
                self.P['dataPath'][idx]['numDopplerBins'] = 8
            else:
                if (self.gProcChain == 'DDM'):
                    self.P['dataPath'][idx]['numDopplerBins'] = self.getValidFFTSize(
                        self.P['profileCfg'][profileCfgIdx]['numDopplerChirps'])
                else:
                    self.P['dataPath'][idx]['numDopplerBins'] = 1 << math.ceil(
                        math.log2(self.P['dataPath'][idx]['numDopplerChirps']))
            if (self.gProcChain == 'DDM'):
                self.P['dataPath'][idx]['numRangeBins'] = self.getValidFFTSize(
                    self.P['profileCfg'][profileCfgIdx]['numAdcSamples'])
            else:
                self.P['dataPath'][idx]['numRangeBins'] = 1 << math.ceil(
                    math.log2(self.P['profileCfg'][profileCfgIdx]['numAdcSamples']))

            if (self.platform == 'AWR294X'):
                self.P['dataPath'][idx]['numRangeBins'] = self.P['dataPath'][idx]['numRangeBins']/2

            if ((self.platform == 'AWR294X') and (self.sdkVersionUint16 >= 0x0301) and ((self.P['dataPath'][idx]['numTxAnt'] * self.P['dataPath'][idx]['numRxAnt']) == 12) and (self.P['dataPath'][idx]['numRangeBins'] == 1024)):
                self.P['dataPath'][idx]['numRangeBins'] = 1022

            if (self.P['profileCfg'][profileCfgIdx]['startFreq'] >= 76):
                CLI_FREQ_SCALE_FACTOR = 3.6  # //77GHz
            else:
                CLI_FREQ_SCALE_FACTOR = 2.7  # //60GHz

            mmwFreqSlopeConst = math.trunc(
                self.P['profileCfg'][profileCfgIdx]['freqSlopeConst'] * (1 << 26) / CLI_FREQ_SCALE_FACTOR)

            self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual'] = mmwFreqSlopeConst * \
                (CLI_FREQ_SCALE_FACTOR / (1 << 26))

            startFreqConst = math.trunc(
                self.P['profileCfg'][profileCfgIdx]['startFreq'] * (1 << 26) / CLI_FREQ_SCALE_FACTOR)

            self.P['profileCfg'][profileCfgIdx]['startFreq_actual'] = (
                startFreqConst * CLI_FREQ_SCALE_FACTOR / (1 << 26))

            #// center freq = start freq post adcstart time + 1/2 bandwidth#
            self.P['profileCfg'][profileCfgIdx]['centerFreq_actual'] = (self.P['profileCfg'][profileCfgIdx]['startFreq_actual'] + 0.5*((self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual'] * self.P['profileCfg'][profileCfgIdx]['numAdcSamples'])/(
                self.P['profileCfg'][profileCfgIdx]['digOutSampleRate'])) + (self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual'] * (self.P['profileCfg'][profileCfgIdx]['adcStartTimeConst'] * 10 * 1e-9)))

            self.P['dataPath'][idx]['rangeResolutionMeters'] = 300 * self.P['profileCfg'][profileCfgIdx]['digOutSampleRate'] / \
                (2 * self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual']
                  * 1e3 * self.P['profileCfg'][profileCfgIdx]['numAdcSamples'])

            # // self.P['dataPath'][idx].rangeIdxToMeters = 3e8 * self.P['profileCfg'][profileCfgIdx].digOutSampleRate * 1e3 /
            # //              (2 * Math.abs(self.P['profileCfg'][profileCfgIdx].freqSlopeConst_actual)* 1e12 * (1 << Math.ceil(Math.log2(self.P['profileCfg'][profileCfgIdx].numAdcSamples))))
            if (self.gProcChain == 'DDM'):
                self.P['dataPath'][idx]['rangeIdxToMeters'] = 3e8 * self.P['profileCfg'][profileCfgIdx]['digOutSampleRate'] * 1e3 / (2 * abs(
                    self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual']) * 1e12 * self.getValidFFTSize(self.P['profileCfg'][profileCfgIdx]['numAdcSamples']))
            else:
                self.P['dataPath'][idx]['rangeIdxToMeters'] = 3e8 * self.P['profileCfg'][profileCfgIdx]['digOutSampleRate'] * 1e3 / (2 * abs(
                    self.P['profileCfg'][profileCfgIdx]['freqSlopeConst_actual']) * 1e12 * (1 << math.ceil(math.log2(self.P['profileCfg'][profileCfgIdx]['numAdcSamples']))))
            self.P['dataPath'][idx]['rangeMeters'] = 300 * 0.8 * self.P['profileCfg'][profileCfgIdx]['digOutSampleRate'] / \
                (2 * self.P['profileCfg'][profileCfgIdx]
                  ['freqSlopeConst_actual'] * 1e3)

            if (self.platform == 'AWR294X'):
                # /* To account for real samples */
                self.P['dataPath'][idx]['rangeMeters'] = self.P['dataPath'][idx]['rangeMeters'] / \
                    (0.8 * 2)

            self.P['dataPath'][idx]['velocityMps'] = 3e8 / (4 * self.P['profileCfg'][profileCfgIdx]['centerFreq_actual'] * 1e9 * (
                self.P['profileCfg'][profileCfgIdx]['idleTime'] + self.P['profileCfg'][profileCfgIdx]['rampEndTime']) * 1e-6 * self.P['dataPath'][idx]['numTxAnt'])

            self.P['dataPath'][idx]['dopplerResolutionMps'] = 3e8 / (2 * self.P['profileCfg'][profileCfgIdx]['centerFreq_actual'] * 1e9 * (
                self.P['profileCfg'][profileCfgIdx]['idleTime'] + self.P['profileCfg'][profileCfgIdx]['rampEndTime'])*1e-6 * self.P['dataPath'][idx]['numDopplerBins'] * self.P['dataPath'][idx]['numTxAnt'])

            NumVirtAnt = self.P['dataPath'][idx]['numTxAnt'] * \
                self.P['dataPath'][idx]['numRxAnt']

            if (((self.platform == 'xWR18xx') or (self.platform == 'xWR68xx')) and (self.sdkVersionUint16 < 0x0302)):
                self.P['log2linScale'][idx] = (
                    1 / 512) * (math.pow(2, math.ceil(math.log2(NumVirtAnt))) / NumVirtAnt)
            else:
                self.P['log2linScale'][idx] = (
                    1 / 256) * (math.pow(2, math.ceil(math.log2(NumVirtAnt))) / NumVirtAnt)
            sumTxScaleFac = 1

            if (NumVirtAnt == 16):
                sumTxScaleFac = 8/6
            else:
                sumTxScaleFac = 1

            rangeFFTScale = 8
            dopplerFFTScale = 0
            FFTScalingCorrection = rangeFFTScale + dopplerFFTScale

            if (self.gProcChain == 'DDM'):
                if (self.platform == 'AWR294X'):
                    self.P['log2linScale'][idx] = 20 * \
                        ((sumTxScaleFac/(math.pow(2, 11)*math.log2(10))))
                else:
                    sumTxScaleFac = 4/4
                    rangeFFTScale = 8
                    dopplerFFTScale = 0
                    FFTScalingCorrection = rangeFFTScale + dopplerFFTScale
                    self.P['log2linScale'][idx] = 20 * \
                        ((sumTxScaleFac/(math.pow(2, 11)*math.log2(10))))

            self.P['toDB'] = 20 * math.log10(2)
            self.P['rangeAzimuthHeatMapGrid_points'] = 100
            
            # print("\n\n\n******************************\n\n")
            # print(self.P)
            # print("\n\n\n******************************\n\n")
            # self.P['stats'] = { 'activeFrameCPULoad': {}, interFrameCPULoad: [], sizeLimit: 100 }
            # for (var i = 0 i < P.stats.sizeLimit i++) {
            #     P.stats.activeFrameCPULoad.push(0)
            #     P.stats.interFrameCPULoad.push(0)
            # }
            # P.scatter_data = {x_coord: [], y_coord: [], z_coord: [], det_obj_list: [], frameNumList: []}
            # for (var i=0 i<P.scatter_data.sizeLimit i++) {
            # P.scatter_data.det_obj_list.push(0)
            # P.scatter_data.frameNumList.push(0)
            # }
            # /*Must depend on range and doppler DPU scaling. Therefore
            #   each demo must have a scalling factor for 1D or 2D depending if the corresponding
            #   DPU is DSP or HWA version.*/
            if (self.platform == 'xWR16xx'):
				# //Range DSP DPU
                # P.dspFftScaleComp1D_lin[idx] = dspFftScalComp1(64, self.P['dataPath'][idx].numRangeBins)
				# P.dspFftScaleComp1D_log[idx] = 20 * Math.log10(P.dspFftScaleComp1D_lin[idx])
                # //Doppler DSP DPU
				# P.dspFftScaleComp2D_lin[idx] = dspFftScalComp2(16, self.P['dataPath'][idx].numDopplerBins)
				# P.dspFftScaleComp2D_log[idx] = 20 * Math.log10(P.dspFftScaleComp2D_lin[idx])
                pass
            elif ((self.platform == 'xWR18xx') or (self.platform == 'AM273X') or (self.platform == 'AWR294X') or (self.platform == 'xWR18xx_AOP')):
				# //Range HWA DPU
                self.P['dspFftScaleComp1D_lin'][idx] = self.dspFftScalComp2(
                    32, self.P['dataPath'][idx]['numRangeBins'])
                self.P['dspFftScaleComp1D_log'][idx] = 20 * \
                    math.log10(self.P['dspFftScaleComp1D_lin'][idx])
                # //Doppler HWA DPU
                self.P['dspFftScaleComp2D_lin'][idx] = 1
                self.P['dspFftScaleComp2D_log'][idx] = 0
            elif ((self.platform == 'xWR64xx')):
                # //Range HWA DPU
                # P.dspFftScaleComp1D_lin[idx] = dspFftScalComp2(32, self.P['dataPath'][idx].numRangeBins)
                # P.dspFftScaleComp1D_log[idx] = 20 * Math.log10(P.dspFftScaleComp1D_lin[idx])
                # //Doppler HWA DPU
                # P.dspFftScaleComp2D_lin[idx] = 1
                # P.dspFftScaleComp2D_log[idx] = 0
                pass
            elif ((self.platform == 'xWR68xx')):
                # //Range HWA DPU
                # P.dspFftScaleComp1D_lin[idx] = dspFftScalComp2(32, self.P['dataPath'][idx].numRangeBins)
                # P.dspFftScaleComp1D_log[idx] = 20 * Math.log10(P.dspFftScaleComp1D_lin[idx])
                # if(sdkVersionUint16 < 0x0302)
                # {
                #     //Doppler HWA DPU
                #     P.dspFftScaleComp2D_lin[idx] = 1
                #     P.dspFftScaleComp2D_log[idx] = 0
                # }
                # else
                # {
                #     //Doppler DSP DPU
                # 	P.dspFftScaleComp2D_lin[idx] = dspFftScalComp2(16, self.P['dataPath'][idx].numDopplerBins)
                # 	P.dspFftScaleComp2D_log[idx] = 20 * Math.log10(P.dspFftScaleComp2D_lin[idx])
                # }
                # To DO
                pass
            elif ((self.platform == 'xWR68xx_AOP')):
                # //Range HWA DPU
                # P.dspFftScaleComp1D_lin[idx] = dspFftScalComp2(32, self.P['dataPath'][idx].numRangeBins)
                # P.dspFftScaleComp1D_log[idx] = 20 * Math.log10(P.dspFftScaleComp1D_lin[idx])
                # //Doppler HWA DPU
                # P.dspFftScaleComp2D_lin[idx] = 1
                # P.dspFftScaleComp2D_log[idx] = 0
                pass
                # To DO
            self.P['dspFftScaleCompAll_lin'][idx] = self.P['dspFftScaleComp2D_lin'][idx] * self.P['dspFftScaleComp1D_lin'][idx]
            self.P['dspFftScaleCompAll_log'][idx] = self.P['dspFftScaleComp2D_log'][idx] + self.P['dspFftScaleComp1D_log'][idx]
            if (self.gProcChain == 'DDM'):
                self.P['dspFftScaleCompAll_log'][idx] = FFTScalingCorrection
                self.P['dspFftScaleCompAll_log'][idx] = 20 * math.log10(FFTScalingCorrection)
        return self.P
    def getProfileIdx(self, subFrameNum):
        if (self.P['dfeDataOutputMode']['mode'] == 1):
            firstChirp = self.P['frameCfg']['chirpStartIdx']
        elif (self.P['dfeDataOutputMode']['mode'] == 3):
            firstChirp = self.P['subFrameCfg'][subFrameNum]['chirpStartIdx']
        profileId = -1
        for i in range(self.chirpCfgCounter):
            if ((firstChirp >= self.P['chirpCfg'][i]['startIdx']) and (firstChirp <= self.P['chirpCfg'][i]['endIdx'])):
                profileId = self.P['chirpCfg'][i]['profileId']
        if (profileId == -1):
            return -1
        for i in range(self.profileCfgCounter):
            if (self.P['profileCfg'][i]['profileId'] == profileId):
                return i
        return -1

    def getAntCfg(self, subFrameNum):
        if (self.P['dfeDataOutputMode']['mode'] == 1):
            if (self.P['chirpCfg'][0]['numTxAzimAnt'] == 1):
                self.P['dataPath'][0]['numTxAzimAnt'] = 1
            else:
                self.P['dataPath'][0]['numTxAzimAnt'] = self.P['channelCfg']['numTxAzimAnt']
            self.P['dataPath'][0]['numTxElevAnt'] = self.P['channelCfg']['numTxElevAnt']
            self.P['dataPath'][0]['numRxAnt'] = self.P['channelCfg']['numRxAnt']
        elif (self.P['dfeDataOutputMode']['mode'] == 3):
            foundFlag = False
            chirp = self.P['subFrameCfg'][subFrameNum]['chirpStartIdx']
            for i in range(self.chirpCfgCounter):
                if ((chirp >= self.P['chirpCfg'][i]['startIdx']) and (chirp <= self.P['chirpCfg'][i]['endIdx'])):
                    foundFlag = True
                    break

            if (foundFlag == False):
                return -1

            if (self.P['chirpCfg'][i]['numTxAzimAnt'] == 1):
                 self.P['dataPath'][subFrameNum]['numTxAzimAnt'] = 1
            else:
                self.P['dataPath'][subFrameNum]['numTxAzimAnt'] = self.P['channelCfg']['numTxAzimAnt']
            self.P['dataPath'][subFrameNum]['numTxElevAnt'] = self.P['channelCfg']['numTxElevAnt']
            self.P['dataPath'][subFrameNum]['numRxAnt'] = self.P['channelCfg']['numRxAnt']
        else:
            return -1
        return 0

    def pow2roundup(self, x):
        power = 1
        while (power < x):
            power = power * 2
        return power

    def getValidFFTSize(self, numSamples):
        fftSize = 0
        numZeros_rad2_only = self.pow2roundup(numSamples) - numSamples
        numZeros_rad3_and_rad2 = 3 * self.pow2roundup(numSamples/3) - numSamples

        if (numZeros_rad2_only <= numZeros_rad3_and_rad2):
            fftSize = self.pow2roundup(numSamples)
        else:
            fftSize = 3 * self.pow2roundup(numSamples/3)
        return fftSize

    def dspFftScalComp2(self, fftMinSize, fftSize):
        sLin = fftMinSize / fftSize
        # //sLog = 20*log10(sLin)
        return sLin

    def dspFftScalComp1(self, fftMinSize, fftSize):
        smin = (math.pow(
            (math.ceil(math.log2(fftMinSize) / math.log2(4) - 1)), 2)) / (fftMinSize)
        sLin = (
            math.pow((math.ceil(math.log2(fftSize) / math.log2(4) - 1)), 2)) / (fftSize)
        sLin = sLin / smin
        # //sLog = 20*log10(sLin)
        return sLin

    def validateCfg(self, lines, dynamicFlg, isrealTimeFlg=False):
        # self.config = self.Read_Config_File()
        if lines is not None:
            for i in lines:
                splitWords = i.split(" ")  # Split the line
                if (splitWords[0] == 'channelCfg'):
                    self.validateChannelCfg(splitWords)
                elif (splitWords[0] == 'profileCfg'):
                    self.validateProfileCfg(splitWords)
                elif (splitWords[0] == 'chirpCfg'):
                    self.validateChirpCfg(splitWords)
                elif (splitWords[0] == 'frameCfg'):
                    self.validateFrameCfg(splitWords)
                elif (splitWords[0] == 'extendedMaxVelocity'):
                    self.validateExtendedMaxVelocity(splitWords)
                elif (splitWords[0] == 'guiMonitor'):
                    self.validateguiMonitor(splitWords)
                elif (splitWords[0] == 'dfeDataOutputMode'):
                    self.validatedfeDataOutputMode(splitWords)
                elif (splitWords[0] == 'advFrameCfg'):
                    self.validateadvFrameCfg(splitWords)
                elif (splitWords[0] == 'subFrameCfg'):
                    self.validatesubFrameCfg(splitWords)
                elif (splitWords[0] == 'cfarCfg'):
                    pass
                    # validateCfarCfg(P, platform, mmwInput, tokens, sdkVersionUint16, dynamicFlg)
                elif (splitWords[0] == 'compRangeBiasAndRxChanPhase'):
                    # validatecompRangeBiasAndRxChanPhase(P, platform, tokens, mmwInput, dynamicFlg)
                    pass
                elif (splitWords[0] == 'measureRangeBiasAndRxChanPhase'):
                    self.validatemeasureRangeBiasAndRxChanPhase(
                        splitWords)
                elif (splitWords[0] == 'CQRxSatMonitor'):
                    # self.validateCQRxSatMonitor(tokens, dynamicFlg)
                    pass
                elif (splitWords[0] == 'CQSigImgMonitor'):
                    # self.validateCQSigImgMonitor(tokens,dynamicFlg)
                    pass
                elif (splitWords[0] == 'analogMonitor'):
                    # self.validateanalogMonitor(dynamicFlg,tokens)
                    pass
                elif (splitWords[0] == 'multiObjBeamForming'):
                    self.validatemultiObjBeamForming(dynamicFlg, splitWords)
                elif (splitWords[0] == 'calibDcRangeSig'):
                    # self.validatecalibDcRangeSig(P, platform, splitWords, mmwInput, sdkVersionUint16, dynamicFlg)
                    pass
                elif (splitWords[0] == 'adcbufCfg'):
                    self.validateadcbufCfg(splitWords, dynamicFlg)
                elif (splitWords[0] == 'adcCfg'):
                    self.validateadcCfg(splitWords)
                elif (splitWords[0] == 'clutterRemoval'):
                    self.validateClutterRemoval(splitWords)
                elif (splitWords[0] == 'bpmCfg'):
                    # self.validatebpmCfg(dynamicFlg, splitWords)
                    pass
                elif (splitWords[0] == 'lvdsStreamCfg'):
                    # self.validatelvdsStreamCfg(P, platform, splitWords, mmwInput, sdkVersionUint16, dynamicFlg)
                    pass
                elif (splitWords[0] == 'nearFieldCfg'):
                    # self.validatenearFieldCfg(P, platform, dynamicFlg, splitWords, sdkVersionUint16, mmwInput)
                    pass
                elif (splitWords[0] == 'lowPower'):
                    self.validatelowPower(dynamicFlg, splitWords)
                elif (splitWords[0] == 'cfarFovCfg'):
                    self.validatecfarFovCfg(
                        dynamicFlg, splitWords, isrealTimeFlg)
                elif (splitWords[0] == 'aoaFovCfg'):
                    self.validateaoaFovCfg(
                        dynamicFlg, splitWords, isrealTimeFlg)
                elif (splitWords[0] == 'dataPathClkCfg'):
                    self.validatedataPathClkCfg(dynamicFlg, splitWords)
                elif (splitWords[0] == 'hsiClockConfig'):
                    # self.validateHsiClkCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                    pass
                elif (splitWords[0] == 'hsiLaneConfig'):
                    # self.validateHsiLaneCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                    pass
                elif (splitWords[0] == 'dataFormatConfig'):
                    pass
                    # self.validateDataFmtCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                elif (splitWords[0] == 'dataPathConfig'):
                    pass
                    # self.validateDataPathCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                elif (splitWords[0] == 'compressionCfg'):
                    pass
                    # self.validateCompressionCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                elif (splitWords[0] == 'localMaxCfg'):
                    pass
                    # self.validateLocalMaxCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                elif (splitWords[0] == 'antennaCalibParams'):
                    # self.validateAntennaCalibParamsCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                    pass
                elif (splitWords[0] == 'intfMitigCfg'):
                    # self.validateIntfMitigCfg(P, platform, dynamicFlg, mmwInput, sdkVersionUint16, splitWords)
                    pass
                elif (splitWords[0] == 'calibData'):
                    self.validateCalibDataCfg(splitWords)
            if (self.P['configErrorFlag'] == 1):
                raise ValueError('Error in configuration file')
        else:
            raise AttributeError('Configuration file not present')

    def subframeNumberToPlot(self):
        if (self.P['dfeDataOutputMode']['mode'] == 3):
            for i in range(self.maxNumSubframes):
                if ((self.P['guiMonitor'][i]['logMagRange'] == 1) or (self.P['guiMonitor'][i]['noiseProfile'] == 1) or (self.P['guiMonitor'][i]['rangeAzimuthHeatMap'] == 1) or (self.P['guiMonitor'][i]['rangeDopplerHeatMap'] == 1) or (self.P['guiMonitor'][i]['statsInfo'] == 1)):
                    return i
        return 0
    # /* Validate channel Cfg*/

    def checkDetectedObjectsSetting(self):
        if (self.P['dfeDataOutputMode']['mode'] == 3):
             for i in range(self.maxNumSubframes):
                if (self.P['guiMonitor'][i]['detectedObjects'] > 0):
                    return 1  # //enabled
        else:
            if (self.P['guiMonitor'][0]['detectedObjects'] > 0):
                return 1
            else:
                return 0
        return 0



    def validateChannelCfg(self, tokens):
        print(tokens)
        self.P['channelCfg']['txChannelEn'] = eval(tokens[2])
        self.P['channelCfg']['rxChannelEn'] = eval(tokens[1])
        self.P['channelCfg']['numTxAzimAnt'] = ((self.P['channelCfg']['txChannelEn'] << 0) & 1) + (
            (self.P['channelCfg']['txChannelEn'] >> 2) & 1) + ((self.P['channelCfg']['txChannelEn'] >> 3) & 1)
        self.P['channelCfg']['numTxElevAnt'] = (
            (self.P['channelCfg']['txChannelEn'] >> 1) & 1)
        self.P['channelCfg']['numRxAnt'] = ((self.P['channelCfg']['rxChannelEn'] << 0) & 1) + ((self.P['channelCfg']['rxChannelEn'] >> 1) & 1)+(
            (self.P['channelCfg']['rxChannelEn'] >> 2) & 1) + ((self.P['channelCfg']['rxChannelEn'] >> 3) & 1)
        print('\n\n')
        print('*****************************************')
        print(self.P['channelCfg'])
        print('\n\n')
        print('*****************************************')

    # Validate Profile cfg
    def validateProfileCfg(self, tokens):
        self.P['profileCfg'][self.profileCfgCounter] = {}
        self.P['profileCfg'][self.profileCfgCounter]['profileId'] = eval(
            tokens[1])
        self.P['profileCfg'][self.profileCfgCounter]['startFreq'] = eval(
            tokens[2])
        self.P['profileCfg'][self.profileCfgCounter]['idleTime'] = eval(
            tokens[3])
        self.P['profileCfg'][self.profileCfgCounter]['adcStartTimeConst'] = eval(
            tokens[4])
        self.P['profileCfg'][self.profileCfgCounter]['rampEndTime'] = eval(
            tokens[5])
        self.P['profileCfg'][self.profileCfgCounter]['freqSlopeConst'] = eval(
            tokens[8])
        self.P['profileCfg'][self.profileCfgCounter]['numAdcSamples'] = eval(
            tokens[10])
        self.P['profileCfg'][self.profileCfgCounter]['digOutSampleRate'] = eval(
            tokens[11])
        self.profileCfgCounter += 1

    # /* Validate Chirp Cfg*/
    def validateChirpCfg(self, tokens):
        self.P['chirpCfg'][self.chirpCfgCounter] = {}
        self.P['chirpCfg'][self.chirpCfgCounter]['startIdx'] = eval(tokens[1])
        self.P['chirpCfg'][self.chirpCfgCounter]['endIdx'] = eval(tokens[2])
        self.P['chirpCfg'][self.chirpCfgCounter]['profileId'] = eval(tokens[3])
        self.P['chirpCfg'][self.chirpCfgCounter]['txEnable'] = eval(tokens[8])
        self.P['chirpCfg'][self.chirpCfgCounter]['numTxAzimAnt'] = 0
        self.chirpCfgCounter += 1

    # /* Validate Frame Cfg*/
    def validateFrameCfg(self, tokens):
        if self.P['dfeDataOutputMode']['mode'] != 1:
            self.P['configErrorFlag'] = 1
        self.P['frameCfg']['chirpStartIdx'] = eval(tokens[1])
        self.P['frameCfg']['chirpEndIdx'] = eval(tokens[2])
        self.P['frameCfg']['numLoops'] = eval(tokens[3])
        self.P['frameCfg']['numFrames'] = eval(tokens[4])
        self.P['frameCfg']['numAdcSamples'] = eval(tokens[5])
        self.P['frameCfg']['framePeriodicity'] = eval(tokens[6])

    # /* Validate Extended Velocity */
    def validateExtendedMaxVelocity(self, tokens):
        R_Type, error = self.checkSubFrameIdx( eval(tokens[1]), "extendedMaxVelocity")
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
        if len(tokens) != 3:
            self.P['configErrorFlag'] = 1
            raise ValueError("extendedMaxVelocity invalid number of arguments")
        subFrameMaxVel = eval(tokens[1])
        if subFrameMaxVel == -1:
            # /*This is a 'broadcast to all subframes' configuration*/
            for maxVelIdx in range(self.maxNumSubframes):
                self.P['extendedMaxVelocity'][maxVelIdx]['enable'] = eval(
                    tokens[2])
        else:
            self.P['extendedMaxVelocity'][subFrameMaxVel]['enable'] = eval(
                tokens[2])

    def validateguiMonitor(self, tokens):
        # /* Validate guiMonitor*/
        if len(tokens) != 8:
            self.P['configErrorFlag'] = 1
            raise ValueError("guiMonitor invalid number of arguments")
        """/*GUI monitor for subframe N is stored in array positon N. If GUI monitor command is sent with subframe -1, configuration is copied in all subframes 0-maxNumSubframes*/"""
        guiMonIdx = eval(tokens[1])
        R_Type, error = self.checkSubFrameIdx(guiMonIdx, "guiMonitor")
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
            raise ValueError(error)
        if (guiMonIdx == -1):
            # /*This is a 'broadcast to all subframes' configuration*/
            for guiIdx in range(self.maxNumSubframes):
                self.P['guiMonitor'][guiIdx]['subFrameIdx'] = eval(tokens[1])
                self.P['guiMonitor'][guiIdx]['detectedObjects'] = eval(
                    tokens[2])
                self.P['guiMonitor'][guiIdx]['logMagRange'] = eval(tokens[3])
                self.P['guiMonitor'][guiIdx]['noiseProfile'] = eval(tokens[4])
                self.P['guiMonitor'][guiIdx]['rangeAzimuthHeatMap'] = eval(
                    tokens[5])
                self.P['guiMonitor'][guiIdx]['rangeDopplerHeatMap'] = eval(
                    tokens[6])
                self.P['guiMonitor'][guiIdx]['statsInfo'] = eval(tokens[7])
        else:
            self.P['guiMonitor'][guiMonIdx]['subFrameIdx'] = eval(tokens[1])
            self.P['guiMonitor'][guiMonIdx]['detectedObjects'] = eval(
                tokens[2])
            self.P['guiMonitor'][guiMonIdx]['logMagRange'] = eval(tokens[3])
            self.P['guiMonitor'][guiMonIdx]['noiseProfile'] = eval(tokens[4])
            self.P['guiMonitor'][guiMonIdx]['rangeAzimuthHeatMap'] = eval(
                tokens[5])
            self.P['guiMonitor'][guiMonIdx]['rangeDopplerHeatMap'] = eval(
                tokens[6])
            self.P['guiMonitor'][guiMonIdx]['statsInfo'] = eval(tokens[7])

    # /* Validate dfeDataOutputMode*/
    def validatedfeDataOutputMode(self, tokens):
        if len(tokens) != 2:
            self.P['configErrorFlag'] = 1
            raise ValueError("dfeDataOutputMode invalid number of arguments")
            # configParameters['configErrorFlag'] = 1
        self.P['dfeDataOutputMode']['mode'] = eval(tokens[1])

    # /*Validate AdvFrame Cfg*/
    def validateadvFrameCfg(self, tokens):
        if len(tokens) != 7:
            self.P['configErrorFlag'] = 1
            raise ValueError("advFrameCfg invalid number of arguments")
            # configParameters['configErrorFlag'] = 1
        if self.P['dfeDataOutputMode']['mode'] != 3:
            self.P['configErrorFlag'] = 1
            raise ValueError("advFrameCfg must use dfeDataOutputMode 3")
            # configParameters['configErrorFlag'] = 1
        self.P['advFrameCfg']['numOfSubFrames'] = eval(tokens[1])
        self.P['advFrameCfg']['forceProfile'] = eval(tokens[2])
        self.P['advFrameCfg']['numFrames'] = eval(tokens[3])
        self.P['advFrameCfg']['triggerSelect'] = eval(tokens[4])
        self.P['advFrameCfg']['frameTrigDelay'] = eval(tokens[5])
        self.P['advFrameCfg']['frameData']['numSubFrames'] = eval(tokens[6])
        if self.P['advFrameCfg']['numOfSubFrames'] > self.maxNumSubframes:
            self.P['configErrorFlag'] = 1
            raise ValueError("advFrameCfg: Maximum number of subframes is 4")

    # /* Validate subFrame Cfg*/ # Not used in demo visualizer
    def validatesubFrameCfg(self, tokens):
        if len(tokens) != 11:
            self.P['configErrorFlag'] = 1
            raise ValueError("subFrameCfg invalid number of arguments")
        if self.P['dfeDataOutputMode']['mode'] != 3:
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "subFrameCfg is allowed only in advFrameCfg mode and must use dfeDataOutputMode 3")
        subFrameNumLocal = eval(tokens[1])
        if (subFrameNumLocal >= self.maxNumSubframes):
            self.P['configErrorFlag'] = 1
            raise ValueError("Bad subframe config:Invalid subframe number")
        self.P['subFrameCfg'][subFrameNumLocal] = {}
        self.P['subFrameCfg'][subFrameNumLocal]['forceProfileIdx'] = eval(
            tokens[2])
        self.P['subFrameCfg'][subFrameNumLocal]['chirpStartIdx'] = eval(
            tokens[3])
        self.P['subFrameCfg'][subFrameNumLocal]['numOfChirps'] = eval(
            tokens[4])
        self.P['subFrameCfg'][subFrameNumLocal]['numLoops'] = eval(tokens[5])
        self.P['subFrameCfg'][subFrameNumLocal]['burstPeriodicity'] = eval(
            tokens[6])
        self.P['subFrameCfg'][subFrameNumLocal]['chirpStartIdxOffset'] = eval(
            tokens[7])
        self.P['subFrameCfg'][subFrameNumLocal]['numOfBurst'] = eval(tokens[8])
        self.P['subFrameCfg'][subFrameNumLocal]['numOfBurstLoops'] = eval(
            tokens[9])
        self.P['subFrameCfg'][subFrameNumLocal]['subFramePeriodicity'] = eval(
            tokens[10])
        if (self.P['subFrameCfg'][subFrameNumLocal]['numOfBurst'] != 1):
            self.P['configErrorFlag'] = 1
            raise ValueError("Bad subframe config: numOfBurst must be 1")
        if (self.P['subFrameCfg'][subFrameNumLocal]['numOfBurstLoops'] != 1):
            self.P['configErrorFlag'] = 1
            raise ValueError("Bad subframe config: numOfBurstLoops must be 1")

    # 	/* Validate Cfar Cfg*/
    """def validateCfarCfg(self, tokens):
        localSubframe = eval(tokens[1])
        threshold = eval(tokens[8])
        peakGroupingEn = eval(tokens[9])
        if ((peakGroupingEn != 0) and (peakGroupingEn != 1)):
            self.P['configErrorFlag'] = 1
            raise ValueError("cfarCfg invalid peakGroupingEn value.")
        if (threshold > 100.0):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "cfarCfg invalid thresholdScale. This parameter must be given in dB and it must be in the range [0-100].")
        if (len(tokens) != 10):
            self.P['configErrorFlag'] = 1
            raise ValueError("cfarCfg invalid number of arguments")
        R_Type, error = self.checkSubFrameIdx(self.P, localSubframe, "cfarCfg")
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
            raise ValueError(error)
    # /* Validate compRangeBiasAndRxChanPhase*/
    def validatecompRangeBiasAndRxChanPhase(self, tokens, dynamicFlg):
        checkTokenLength=34
        if (len(tokens) != checkTokenLength):
            self.P['configErrorFlag']=1
            raise ValueError(
                "compRangeBiasAndRxChanPhase invalid number of arguments")
        self.P['compRxChanCfg']['rangeBias'] = eval(tokens[1])
        """

    # /* Validate measureRangeBiasAndRxChanPhase*/
    def validatemeasureRangeBiasAndRxChanPhase(self, tokens):
        if (len(tokens) != 4):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "measureRangeBiasAndRxChanPhase invalid number of arguments")
        self.P['measureRxChanCfg']['enabled'] = int(
            eval(tokens[1]))  # //0 - compensation 1- measurement

    # /* Validate multiObjBeamForming*/
    def validatemultiObjBeamForming(self, dynamicFlg, tokens):
        localSubframe = int(eval(tokens[1]))
        if (len(tokens) != 4):
            self.P['configErrorFlag'] = 1
            raise ValueError("multiObjBeamForming invalid number of arguments")

        R_Type, error = self.checkSubFrameIdx(
            localSubframe, "multiObjBeamForming", dynamicFlg)
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
            raise ValueError(error)

    # /* Validate adcbufCfg*/
    def validateadcbufCfg(self, tokens, dynamicFlg):
        pass

    # /* Validate adcCfg*/
    def validateadcCfg(self, tokens):
        pass

    # /* Validate clutterRemoval*/
    def validateClutterRemoval(self, tokens):
        pass

    # /* Validate calibData */
    def validateCalibDataCfg(self, tokens):
        if ((int(eval(tokens[1])) + int(eval(tokens[2]))) > 1):
            self.P['configErrorFlag'] = 1
            raise ValueError("calibData invalid configuration ")

    # /* Validate lowPower*/
    def validatelowPower(self, dynamicFlg, tokens):
        # if (len(tokens) != 3):
        #     self.P['configErrorFlag'] = 1
        #     raise ValueError("lowPower invalid configuration ")
        pass
    # /* Validate cfar FOV*/

    def validatecfarFovCfg(self, dynamicFlg, tokens, isrealTimeFlg):
        if (len(tokens) != 5):
            self.P['configErrorFlag'] = 1
            raise ValueError("cfarFovCfg invalid configuration")
        cfarFovSubframeIdx = int(tokens[1])
        cfarFovDir = int(eval(tokens[2]))
        cfarFovMin = float(eval(tokens[3]))
        cfarFovMax = float(eval(tokens[4]))
        R_Type, error = self.checkSubFrameIdx(
            cfarFovSubframeIdx, "cfarFovCfg", dynamicFlg)
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
            raise ValueError(error)
        if (cfarFovMin > cfarFovMax):
            raise ValueError(
                "cfarFovCfg invalid min/max configuration: Min > Max. ")

        if (cfarFovDir == 0):
            if (cfarFovMin < 0):
                raise ValueError(
                    "cfarFovCfg minimum range invalid. Minimum value allowed is 0.")

            if (cfarFovSubframeIdx == -1):
                for cfarFovIdx in range(self.maxNumSubframes):
                   self.P['cfarRangeFov'][cfarFovIdx] = {
                       'min': cfarFovMin, 'max': cfarFovMax}
            else:
                self.P['cfarRangeFov'][cfarFovSubframeIdx] = {
                    'min': cfarFovMin, 'max': cfarFovMax}
        elif (cfarFovDir == 1):
            if (cfarFovSubframeIdx == -1):
                for cfarFovIdx in range(self.maxNumSubframes):
                   self.P['cfarDopplerFov'][cfarFovIdx] = {
                       'min': cfarFovMin, 'max': cfarFovMax}
            else:
                self.P['cfarDopplerFov'][cfarFovSubframeIdx] = {
                    'min': cfarFovMin, 'max': cfarFovMax}
        else:
            self.P['configErrorFlag'] = 1
            raise ValueError("cfarFovCfg invalid procDirection")

    # /*Validate AoA FOV*/
    def validateaoaFovCfg(self, dynamicFlg, tokens, isrealTimeFlg):
        if (len(tokens) != 6):
            self.P['configErrorFlag'] = 1
            raise ValueError("aoaFovCfg invalid procDirection")
        aoaFovSubframeIdx = int(eval(tokens[1]))
        aoaFovMinAzim = float(eval(tokens[2]))
        aoaFovMaxAzim = float(eval(tokens[3]))
        aoaFovMinElev = float(eval(tokens[4]))
        aoaFovMaxElev = float(eval(tokens[5]))
        if (aoaFovMinAzim > aoaFovMaxAzim):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid azimuth min/max configuration: Min > Max.")

        if (aoaFovMinElev > aoaFovMaxElev):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid elevation min/max configuration: Min > Max. ")

        if (aoaFovMinAzim < -90):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid min azimuth angle. Minimum allowed value is -90.")

        if (aoaFovMaxAzim > 90):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid min elevation angle. Minimum allowed value is -90.")

        if (aoaFovMinElev < -90):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid max azimuth angle. Maximum allowed value is 90.")

        if (aoaFovMaxElev > 90):
            self.P['configErrorFlag'] = 1
            raise ValueError(
                "aoaFovCfg invalid max elevation angle. Maximum allowed value is 90.")

        R_Type, error = self.checkSubFrameIdx(
            aoaFovSubframeIdx, "aoaFovCfg", dynamicFlg)
        if not R_Type:
            # /*return error*/
            self.P['configErrorFlag'] = 1
            raise ValueError(error)

        if (aoaFovSubframeIdx == -1):
            for aoaFovIdx in range(self.maxNumSubframes):
                self.P['aoaFovCfg'][aoaFovIdx] = {
                    'minAzim': aoaFovMinAzim,
                    'maxAzim': aoaFovMaxAzim,
                    'minElev': aoaFovMinElev,
                    'manElev': aoaFovMaxElev
                    }
        else:
            self.P['aoaFovCfg'][aoaFovSubframeIdx] = {
                    'minAzim': aoaFovMinAzim,
                    'maxAzim': aoaFovMaxAzim,
                    'minElev': aoaFovMinElev,
                    'manElev': aoaFovMaxElev
                    }
        """Validating Commands arguments and syntax
        isRealTime Flag is false by default, it will remain false from Send Config, Load Config, Advacned Command Window.
        when action performed in the Real Time Window on FOV commands it becomes true, to display error messages(if any) in Real Time tab for FOV commands*/"""

    # def PhraseConfigFile(self):
    #     self.config = self.Read_Config_File()
    #     if self.config is not None:
    #         configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    #         for i in self.config:
    #             splitWords = i.split(" ") # Split the line
    #             # Get the information about the profile configuration
    #             if "profileCfg" in splitWords[0]:
    #                 startFreq = int(float(splitWords[2]))
    #                 idleTime = int(splitWords[3])
    #                 rampEndTime = float(splitWords[5])
    #                 freqSlopeConst = float(splitWords[8])
    #                 numAdcSamples = int(splitWords[10])
    #                 numAdcSamplesRoundTo2 = 1
    #                 while numAdcSamples > numAdcSamplesRoundTo2:
    #                     numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
    #                 digOutSampleRate = int(splitWords[11])
    #             # Get the information about the frame configuration
    #             elif "frameCfg" in splitWords[0]:
    #                 chirpStartIdx = int(splitWords[1])
    #                 chirpEndIdx = int(splitWords[2])
    #                 numLoops = int(splitWords[3])
    #                 numFrames = int(splitWords[4])
    #                 framePeriodicity = float(splitWords[5])
    #         # For specific parameter setting
    #         configParameters['numTxAnt'] = 2
    #         # Combine the read data to obtain the configuration parameters
    #         numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    #         configParameters["numDopplerBins"] = numChirpsPerFrame /  configParameters['numTxAnt']
    #         configParameters["numRangeBins"] = numAdcSamplesRoundTo2 //2 # for 2944
    #         configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    #         configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    #         configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] *  configParameters['numTxAnt'])
    #         configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    #         configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters['numTxAnt'])
    #         return configParameters
    #     else:
    #         return None
if __name__=="__main__":
    pass