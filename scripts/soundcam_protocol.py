from enum import Enum
import dataclasses, struct, socket
from dataclasses import dataclass, fields
from collections import namedtuple
from bitarray import bitarray
from bitarray.util import int2ba, ba2int
from operator import itemgetter
from typing import List
import numpy as np

Features = namedtuple('Features', 'LED Ultrasound Microcontroller Battery IR TachoTrigger')
MDDataMessage = namedtuple('MDDataMessage', 'Command InvokeId Reserved DataLength ObjectCount')
Status = namedtuple('Status', 'isListenerCreated isConnectedHost isBitfileLoaded isBitfileRunning isTransferActive')
MDLeakRateData = namedtuple('MDLeakRateData', 'TimeStamp LeakRate State Reserved1 Reserved2')
class CommandCodes(Enum):
    ResetReq = 0x80
    ResetRes = 0x00
    IdentificationReq = 0x84
    IdentificationRes = 0x04
    UDPIdentificationRes = 0x05
    PrepareStateReq = 0x86
    PrepareStateRes = 0x06
    FinishStateReq = 0x87
    FinishStateRes = 0x07
    StopProcedureReq = 0x89
    StopProcedureRes = 0x09
    StartProcedureReq = 0x8A
    StartProcedureRes = 0x0A
    ReadDataObjectReq = 0x92
    ReadDataObjectRes = 0x12
    WriteDataObjectReq = 0x93
    WriteDataObjectRes = 0x13

class DataMessages(object):
    class CommandCodes(Enum):
        DataMessage = 0x41

    class Status(Enum): #bitset for the status bitarray
        PrepareStateRes = 0
        FinishStateRes = 1
        StartProcRes = 2
        StopProcRes = 3
        MAX_SIZE = 4

class Device(object):
    class DeviceIDs(Enum):
        CAESoundCam_48_8_kHz = 0xCAEACA01
        CAESoundCam_200_kHz = 0xCAEACA02
        CAESoundCamUltra_200_kHz = 0xCAEACA03
        CAESoundCamSensor_48_8_kHz = 0xCAEACA11
        CAESoundCamSensor_200_kHz = 0xCAEACA12
        CAESoundCamUltraSensor_200_kHz = 0xCAEACA13 #this
        CAESoundCam_dFE_64_I2S_48_8_kHz = 0xCAEDFE01
        CAESoundCamNewBionic_112_48_8_kHz = 0xCAEDBA02
    
    class DeviceStates(Enum):
        Idle = 0
        Measuring = 1
        Streaming = 2
        Service_1 = 3
        Ultrasonic = 4
        Ultrasonic_Plus = 5
        Unknown = 6
    class SubStates(Enum):
        Idle = 0
        Running = 1
        Preparing = 16
        Finished = 128
        Aborted = 129
        Locked = 204
    
    class ErrorCodes(Enum):
        NoError = 0x0000                        #0 No error occurred
        DeviceIdle = 0x0001                     #1 A request requires that the device is not idle.
        DeviceNotIdle = 0x0002                  #2 A request requires that the device is idle.
        InvalidState = 0x0003                   #3 A PrepareState command contains an unknown state
        StateNotPrepared = 0x0004               #4 StartProcedure without previous PrepareState
        SubStateIdle = 0x0005                   #5 A request requires that the SubState is not idle.
        SubStateNotIdle = 0x0006                #6 A request requires that the SubState is idle.
        SubStateNotRunning = 0x0007             #7 A request requires that the SubState is running.
        WriteMemoryInconsistent = 0x0008        #8 The address in the WriteMemoryReq does not match the amount of data transferred so far.
        WriteMemoryWriteFlagError = 0x0009      #9 The WriteFlag is not supported (not in the range [0..3]).
        WriteMemoryChecksumError = 0x000A       #10 The check sum of at least one file from the upload does not match the check sum list (Included in the
                                                    #upload data set).
        WriteMemoryFWFileNotFound = 0x000B      #11 The FW start file specified in the upload data record was not found after unpacking.
        WriteMemoryCreateZipError = 0x000C      #12 FW zip file could not be created
        WriteMemoryWriteZipError = 0x000D       #13 Error while writing to the FW zip file
        WriteMemoryCloseZipError = 0x000E       #14 Error while closing FW fip file
        WriteMemoryExtractZipError = 0x000F     #15 Error while extracting the FW zip file
        WriteMemoryConfigFileError = 0x0010     #16 Error while changing the config file (Entering the new firmware Start VIs)
        WriteMemoryInvalidFWVersion = 0x0011    #17 The version of the firmware to be uploaded is outside the valid range
        NewArray = 0x0020                       #32 The microphone array selected with "ActiveArray" does not exist. A new 
                                                    #one is created and must be configured (at least microphone positions, optionally weights and
                                                    #alignment) before it can be used.
        ArrayNotAvailable = 0x0021              #33 The microphone array selected with "ActiveArray" does not exist and "StartProcedure" was sent before it
                                                    #was configured. "StartProcedure" is not executed.
        ArrayLocked = 0x0022                    #34 An attempt was made to change the configuration of a protected
                                                    #microphone array (CAE standard array).
        InvalidArrayName = 0x0023               #35 The name sent with "ActiveArray" contains illegal characters.
        ArrayNotFound = 0x0024                  #36 The array specified with "RemoveArray“ does not exist and therefore cannot be deleted.
        InvalidArraySN = 0x0025                 #37 The serial number sent contains invalid characters.
        HighTemperature = 0x0080                #128 Temperature of at least one sensor of the SOM very high
        CriticalTemperature = 0x0081            #129 Temperature of at least one sensor of the SOM critically high
        RequestNotSuccessful = 0x00E0           #224 Request could not be executed
        RequestNotSupported = 0x00E1            #225 A request is not supported by the FW.
        RequestNotUpportedBootLoader = 0x00E2   #226 The SOUNDCAM is in BootLoader mode and the sent request cannot be processed there (e.g. frequency range).
        I2CError = 0x00E3                       #227 Not all written register values could be read back correctly.
        NoVideoFrames = 0x00E4                  #228 No video images are transmitted from the FPGA.
        IRCameraInitializationError = 0x00E5    #229 The IR camera could not be initialised.
        NotLicensed = 0x00EE                    #238 No valid license found
        NoLicenseFile = 0x0EF                   #239 No license file found
        NoSerialNumber = 0x0F0                  #240 No serial number readable
        WrongLicenseKey = 0x0F1                 #241 The license key does not match the serial number
        InvalidLicenseKey = 0x0F2               #242 The license key is invalid.
        DataObjectNotSupported = 0x00FD         #253 The data object ID is not supported here (e.g. ReadDataObjectReq SpectrumData).
        DataObjectUnknown = 0x00FE              #254 The data object ID is unknown.
        UnknownCommand = 0x00FF                 #255 Unknown request was received
        FatalError = 0x0400                     #1024 Severe unspecified error. The Device must be restarted

class DataObjects(object):
    class Ids(Enum):
        '''R = ReadDataObjects, W = WriteDataObjects, Data = DataMessage'''
        CommonStatus = 0x0001                   #R + Data
        Distance = 0x0002                       #R + W
        FrequencyRange = 0x0003                 #R + W
        CameraResolution = 0x0004               #R + W
        CameraBrightness = 0x0005               #R + W
        AcousticFrameRate = 0x0006              #R + W
        VideoFrameRate = 0x0007                 #R + W
        CameraLighting = 0x0008                 #R + W
        MicrophonePosition = 0x0009             #R + W (Production only)
        MicrophoneFactor = 0x000A               #R + W (Production only)
        MicrophoneWeighting = 0x000B            #R + W (Production only)
        Alignment = 0x000C                      #R + W (Production only)
        VideoData = 0x000D                      #Data
        AcousticVideoData = 0x000E              #Data
        SpectrumData = 0x000F                   #Data
        AudioData = 0x0010                      #Data
        CameraStatus = 0x0011                   #R + Data
        SelftestStatus = 0x0012                 #R + Data
        LocalSoundTargetCoordinates = 0x0013    #R + W
        MicrophoneIndex = 0x0014                #R + W
        RawData = 0x0019                        #Data
        ProductionData = 0x001A                 #R + W (Production only)
        MicrophoneStatus = 0x001E               #R + Data
        DataToSend = 0x0020                     #R + W
        ActiveArray = 0x0021                    #R + W
        ArraysAvailable = 0x0022                #R + Data
        ArraySerialNumber = 0x0023              #R + W
        AcousticAveraging = 0x0106              #R + W
        ThermalVideoData = 0x0A01               #Data
        CurrentDateTime = 0xC000                #R + W
        MicLevels = 0xC001                      #R + Data
        RawDataStream = 0xC002                  #Data
        NoOfStreamingSamples = 0xC008           #R + W
        LeakRate = 0x0B00                       #R
    
    '''Device Type Ids'''
    class TypeIds(Enum):
        #[DataType]         [TypeId] [DataLength]
        Bool                = 0x0001      #1
        Int8                = 0x0002      #1
        UInt8               = 0x0003      #1
        Int16               = 0x0004      #2
        UInt16              = 0x0005      #2
        Int32               = 0x0006      #4
        UInt32              = 0x0007      #4
        Int64               = 0x0008      #8
        UInt64              = 0x0009      #8
        Float               = 0x000A      #4
        Double              = 0x000B      #8
        String              = 0x000C      #dynamic
        Date                = 0x000D      #4
        DateTime            = 0x000E      #8
        CommonStatus        = 0x000F      #8
        MACAddress          = 0x0010      #12
        ArrayBool           = 0x0011      #dynamic
        ArrayInt8           = 0x0012      #dynamic
        ArrayUInt8          = 0x0013      #dynamic
        ArrayInt16          = 0x0014      #dynamic
        ArrayUInt16         = 0x0015      #dynamic
        ArrayInt32          = 0x0016      #dynamic
        ArrayUInt32         = 0x0017      #dynamic
        ArrayInt64          = 0x0018      #dynamic
        ArrayUInt64         = 0x0019      #dynamic
        ArrayFloat          = 0x001A      #dynamic
        ArrayDouble         = 0x001B      #dynamic
        Fxp24_24            = 0x001C      #3
        FrequencyRange      = 0x0100      #4
        Resolution          = 0x0200      #4
        LocalSoundTarget    = 0x0201      #4
        Position            = 0x0300      #16
        MicrophoneWeighting = 0x0400      #dynamic
        MicrophoneAlignment = 0x0500      #64
        VideoData           = 0x0600      #dynamic
        AcousticVideoData   = 0x0700      #dynamic
        SpectrumData        = 0x0800      #dynamic
        RawData             = 0x0801      #dynamic
        AudioData           = 0x0900      #dynamic
        MicrophoneIndex     = 0x0A00      #4
        ProductionData      = 0x0C00      #36
        CameraStatus        = 0x0C02      #dynamic
        MicLevels           = 0xC001      #dynamic

        '''
        Date -> UInt32 | 4s
            Year UInt16
            Month UInt8
            Day UInt8

        DateTime -> UInt64 | 8s
            Year UInt16
            Month UInt8
            Day UInt8
            Hour UInt8
            Min UInt8
            Sec UInt8
            Kind (0=UTC, 1=Local) UInt8

        String
            CountOfCharacters UInt32
            Characters (Unicode) UInt16

        CommonStatus
            DeviceState UInt32
            SubState UInt32

        MACAddress
            MAC Address hex 12 Char
        '''
    
    '''Thermal Stream Data types'''
    class ThermalDatatype(Enum):
        TelemetryDisabledRaw14 = 0x01 #Bytes per Pixel 2
        TelemetryDisabledRGB888 = 0x02 #Bytes per Pixel 3
        TelemetryEnabledTLinearDisabledRaw14 = 0x03 #Bytes per Pixel 2
        TelemetryEnabledTLinearEnabledRaw14Res0_1K = 0x04 #Bytes per Pixel 2
        TelemetryEnabledTLinearEnabledRaw14Res0_01K = 0x05 #Bytes per Pixel 2
    

    SDCommonStatus = namedtuple('SDCommonStatus', 'ObjetID ObjectType Length ErrorCode DeviceState SubState')
    SDCameraStatus = namedtuple('SDCameraStatus', 'ObjetID Revision Length CurrentDateTime TemperatureCPU TemperatureTop TemperatureBottom \
                                TotalMemory FreeMemory TotalDisk FreeDisk CPULoad_1 CPULoad_2 DeviceOK MicrophonesOK CameraOK IROK \
                                NumberOfMicGroups MicStat1 MicStat2 MicStat3 MicStat4 MicStat5 MicStat6 MicStat7 MicStat8 MicStat9')
    PDistance = namedtuple('PDistance', 'ObjetID ObjectType Value') #Valid range: 100 mm to 3500 mm - Fixed Length: 8
    PFrequencyRange = namedtuple('PFrequencyRange', 'ObjetID Revision FrequencyMin FrequencyMax') #Valid range: Device Dependent - Fixed Length: 8

    '''Valid range: 640 x 480 and 320 x 240 (1280 x 960 for VM12 camera only)
        The 320 x 240 resolution is the standard resolution at which the highest frame rate is
        achieved
    '''
    PCameraResolution = namedtuple('PCameraResolution', 'ObjetID ObjectType HResolution VResolution') #Fixed Length: 8
    PCameraBrightness = namedtuple('PCameraBrightness', 'ObjetID ObjectType Brightness') #Valid range: 0 to 100 - Fixed Length: 8
    PAcousticFrameRate = namedtuple('PAcousticFrameRate', 'ObjetID ObjectType Framerate') #Valid range: 1 to 200 - Fixed Length: 8
    PAcousticAveraging = namedtuple('PAcousticAveraging', 'ObjetID ObjectType AveragingType') #Valid range: 0= fast, 1 = slow, 2 = impulse - Fixed Length: 8
    PVideoFrameRate = namedtuple('PVideoFrameRate', 'ObjetID ObjectType Framerate') #Valid range: 1 to 100 - Fixed Length: 8
    '''Switches the lighting of the SOUNDCAM on or off'''
    PCameraLighting = namedtuple('PCameraLighting', 'ObjetID ObjectType On') #Valid range: 0, 1 - Fixed Length: 8
    '''
        Sets the target coordinates for the local sound. The coordinates are
        given as the position in the acoustic image. 0, 0 is in the center, -32767, -32797 is bottom
        left, and 32767, 32797 is top right.
    '''
    PLocalSoundTargetCoordinates = namedtuple('PLocalSoundTargetCoordinates', 'ObjetID ObjectType Horizontal Vertical') #Valid range:-32767to 32767 - Fixed Length: 8
    '''Selects the microphone for transmitting raw time data and spectra'''
    PMicrophoneIndex = namedtuple('PMicrophoneIndex', 'ObjetID ObjectType Index Filtered') #Valid range: 0, 1 - Fixed Length: 8 NB: param Filtered is obsolete
    PActiveArray = namedtuple('PActiveArray', 'ObjetID Revision DataLength ArrayName') #Valid range: 0, 1 - Fixed Length: 8 NB: for Chassis & Bionic devices
    '''Returns a list of the array configurations stored on the unit'''
    PArraysAvailable = namedtuple('PArraysAvailable', 'ObjetID Revision LengthTotal NumArrays') #Dynamic Length
    PArrays = namedtuple('PArrays', 'ArrayName')
    '''Sets or reads the serial number of the active array'''
    PArraySerialNumber = namedtuple('PArraySerialNumber', 'ObjetID Revision DataLength ArraySerialNumber') #Valid range: 0, 1 - Dynamic Length
    '''Sets the maximum number of samples per channel that are transmitted during raw data streaming.'''
    PNoOfStreamingSamples = namedtuple('PNoOfStreamingSamples', 'ObjetID ObjectType NoOfStreamingSamples') #Valid range: 0, 1 - Fixed Length: 8

    #TODO: Implement Production Parameters: MicrophonePosition MicrophoneWeighting Alignment ProductionData

    '''
        Sets the data to be sent. LocalSound, levels and rawdata may be sent filtered
        PicturesSpectraMic = [AcousticPictures VideoPictures ThermalPictures LocalSound MicLevels MicRawData Spectra SingleMicData]
            NB: The ThermalPictures option is only available for 0xCAEACA23
        FiltersAndGain = [LocalSoundFiltered LevelFiltered RawDataFiltered IR_Gain Reserved Reserved Reserved Reserved]
        TachoAndHWTrigger = [Tacho HWTrigger Reserved Reserved Reserved Reserved Reserved Reserved]
    '''
    MDDataToSend = namedtuple('MDDataToSend', 'ObjetID Revision PicturesSpectraMic Reserved1 FiltersAndGain TachoAndHWTrigger \
                               Reserved2 Reserved3 Reserved4 Reserved5') #Fixed Length: 12
    '''The grayscale video image is transmitted line by line with one byte per pixel from left to right and top to bottom'''
    MDVideoData = namedtuple('MDVideoData', 'TimeStamp HResolution VResolution')
    '''The acoustic image is transmitted line by line with a 32 bit float value per pixel from left to
        right and from top to bottom. The 32 bit value represents the value in dB determined for the
        pixel.
    '''
    MDAcousticImageData = namedtuple('MDAcousticImageData', 'TimeStamp FrequencyMin FrequencyMax Distance \
                                     Reserved1 Reserved2 Reserved3')
    '''
        Data set with a sound pressure spectrum. The first 4 bytes (float)
        represent the frequency resolution of the spectrum in Hz. The following 1024 float values
        represent the spectral lines in dB(A).
    '''
    MDSpectrumData1 = namedtuple('MDSpectrumData1', 'TimeStamp Delta_f Filtered FrequencyMin FrequencyMax')
    MDSpectrumData2 = namedtuple('MDSpectrumData2', 'Level_global Level_local Level_global_band Level_local_band')
    '''
        Data set with 2048 samples of the focused sound signal
        SoundCam 1.0/2.0 and Digital Frontend with 64 channels
        LSB is 5.268 10-8 Pa (3.372 10-6 Pa / 64)
        SoundCam Ultra / Ultra Sensor
        LSB is 4.683 10-8 Pa (3.372 10-6 Pa / 72)
        Bionic arrays
        LSB is 3.01 10-8 Pa (3.372 10-6 Pa / 112)
        Digital Frontend with 128 channels
        LSB is 2.634 10-8 Pa (3.372 10-6 Pa / 128)

        As of FW version 2.8.0.0, the LocalSound signal is normalised with the number of
        microphones. This means that the scaling factor is 3.372 10-6 Pa, independently of the
        device.
    '''
    MDAudioData1 = namedtuple('MDAudioData1', 'TimeStamp Horizontal Vertical')
    MDAudioData2 = namedtuple('MDAudioData2', 'dt Filtered FrequencyMin FrequencyMax')

    MDRawdata = namedtuple('MDRawdata', 'TimeStamp MicrophoneIndex RawData dt')
    MDThermalVideoData = namedtuple('MDThermalVideoData', 'TimeStamp HResolution VResolution DataType')

    #TODO: Implement Service Parameter: MicLevels RawDataStream
    
    SPCurrentDateTime = namedtuple('SPCurrentDateTime', 'ObjetID ObjectType Length CurrentDateTime')

    #Other named tuples
    DataObjPk = namedtuple('DataObjPk', 'Id Struct')
    DataObjHeader = namedtuple('DataObjHeader', 'Id Type Length')


@dataclass
class CameraProtocol(object):
    """Class keeping track of device states and protocols"""
    IdResponse = namedtuple('IdResponse', 'Command InvokeId Reserved Status1 DataLength DeviceProtocolVersion \
                            DeviceId DeviceMode DeviceError DeviceSerialNumber SiliconSerialNumber HardwareVersion \
                            BootloaderVersion FirmwareVersion FPGAVersion ProductionDate CalibrationDate MACAddress \
                            IPAddress PCBSerialNumber Features1 Features2 Features3 Status2 ConnectIP DeviceState \
                            SubState Error Step0 Step_1 TemperatureCPU TemperatureTop TemperatureBottom \
                            CPULoad_1 CPULoad_2')
    UdpResponse = namedtuple('UdpResponse', 'Command InvokeId Reserved Status1 DataLength DeviceProtocolVersion \
                            DeviceId DeviceMode DeviceError DeviceSerialNumber SiliconSerialNumber HardwareVersion \
                            BootloaderVersion FirmwareVersion FPGAVersion ProductionDate CalibrationDate MACAddress \
                            IPAddress PCBSerialNumber')
    GenericResponse = namedtuple('GenericResponse', 'Command InvokeId Reserved Status DataLength')
    ReadDataObjectResponse = namedtuple('ReadDataObjectResponse', 'Command InvokeId Reserved Status DataLength DataObjectCount')

    def __init__(self, protocol:int=3, debug=False):
        self.pcProtocolVersion = protocol
        self.debug = debug

        self.prepareStateResponse = Device.DeviceStates.Idle
        self.DeviceState = Device.DeviceStates.Idle
        self.SubState = Device.SubStates.Idle
        self.isConfigured = False
        self.hasInitialStatus = False

        self.stateProcStatus = bitarray(DataMessages.Status.MAX_SIZE.value)
        self.stateProcStatus.setall(0)

    
    def __post_init__(self):
        print('Running post init')
        for field in fields(self):
            # If there is a default and the value of the field is none we can assign a value
            if not isinstance(field.default, dataclasses._MISSING_TYPE) and getattr(self, field.name) is None:
                setattr(self, field.name, field.default)
    
    def __str__(self):
        summary = '''ProtocolVersion: {0} \n\t DeviceId: {1} \n\t DeviceIP: {2} \n\t DeviceMAC: {3} 
                    \n\t Features: {4}, \n\t Status[2]: {5} '''.format(self.DeviceProtocolVersion, self.DeviceId, self.IPAddress, 
                                                  self.MACAddress, self.Features1, self.Status2)
        return summary
    
    '''
            PARAMETER GETTERS
    '''
    def p_isConfigured(self):
        return self.isConfigured
    def p_hasInitialStatus(self):
        return self.hasInitialStatus

    '''
            OTHER METHODS
    '''
    def setInitialStatus(self, val):
        self.hasInitialStatus = val

    def getDeviceState(self):
        try:
            return (self.DeviceState, self.SubState)
        except Exception as ex:
            return None
    
    def unpackDecodeResponse(self, response:bytes)->None:
        if(response):
            # if(len(response) < 1460):
            #     print(response, ' Length:', len(response))
            if(self.pcProtocolVersion == 3):
                unpacked:List[CameraProtocol | DataObjects] = list()
                cmd = int.from_bytes(response[:1], byteorder='little')
                if(cmd == CommandCodes.IdentificationRes.value):
                    unpacked.append(CameraProtocol.IdResponse._make(struct.unpack('<2BH7LQ4L4s4s12s2L4B4L2H3f2L', response[:128])))
                elif(cmd == CommandCodes.UDPIdentificationRes.value):
                    if(len(response) > 84): #Extended message
                        unpacked.append(CameraProtocol.IdResponse._make(struct.unpack('<2BH7LQ4L4s4s12s2L4B4L2H3f2L', response[:128])))
                    else: #Normal Broadcast response
                        unpacked.append(CameraProtocol.UdpResponse._make(struct.unpack('<2BH7LQ4L4s4s12s2L', response[:84])))
                elif(cmd == CommandCodes.ReadDataObjectRes.value): #Read Response
                    dstr = '<2BH3L'
                    arr_idx = struct.Struct(dstr).size
                    rdobj_unpacked = CameraProtocol.ReadDataObjectResponse._make(struct.unpack(dstr, response[:arr_idx]))
                    for i in range(rdobj_unpacked.DataObjectCount):
                        dtobjId = int.from_bytes(response[arr_idx:arr_idx+2], byteorder='little')
                        if(dtobjId == DataObjects.Ids.CommonStatus.value): #CommonStatus  -------------------------------Status Data
                            dstr = '<2H4L'
                            unpacked.append(DataObjects.SDCommonStatus._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += (unpacked[-1].Length + 8) # dtobjLen->(value) + dtobjType->2 + dtobId->2 + dtobjLen->4  
                        elif(dtobjId == DataObjects.Ids.CameraStatus.value): #CameraStatus
                            dstr = '<2HL8s3f6L4BL9B'
                            unpacked.append(DataObjects.SDCameraStatus._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += (unpacked[-1].Length + 8) # dtobjLen->(value) + dtobjType->2 + dtobId->2 + dtobjLen->4
                        elif(dtobjId == DataObjects.Ids.Distance.value): #Distance    -------------------------------Parameters
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PDistance._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.FrequencyRange.value): #FrequencyRange
                            dstr = '<2H2L'
                            unpacked.append(DataObjects.PFrequencyRange._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 12  # FixedLength
                        elif(dtobjId == DataObjects.Ids.CameraResolution.value): #CameraResolution
                            dstr = '<4H'
                            unpacked.append(DataObjects.PCameraResolution._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.CameraBrightness.value): #CameraBrightness
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PCameraBrightness._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.AcousticFrameRate.value): #AcousticFrameRate
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PAcousticFrameRate._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.AcousticAveraging.value): #AcousticAveraging
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PAcousticAveraging._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.VideoFrameRate.value): #VideoFrameRate
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PVideoFrameRate._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.CameraLighting.value): #CameraLighting
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PCameraLighting._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.LocalSoundTargetCoordinates.value): #LocalSoundTargetCoordinates
                            dstr = '<2H2h'
                            unpacked.append(DataObjects.PLocalSoundTargetCoordinates._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.MicrophoneIndex.value): #MicrophoneIndex
                            dstr = '<4H'
                            unpacked.append(DataObjects.PMicrophoneIndex._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength
                        elif(dtobjId == DataObjects.Ids.ArraysAvailable.value): #ArraysAvailable
                            dstr = '<2HLH'
                            dssize = struct.Struct(dstr).size
                            tmp_unpack = DataObjects.PArraysAvailable._make(struct.unpack(dstr, response[arr_idx:(arr_idx + dssize)]))
                            arr_idx += dssize
                            if(self.debug):
                                print('Idx after partial read: ', arr_idx) #@ 26 (16 + 10)
                            for x in range(tmp_unpack.NumArrays):
                                arr_len = response[arr_idx] #13
                                dstr = '<' + str(arr_len) + 's'
                                dssize = struct.Struct(dstr).size
                                unpacked.append(DataObjects.PArrays._make(struct.unpack(dstr, response[(arr_idx + 1):((arr_idx + 1) + dssize)]))) # +1 to skip '\r' in Name
                                arr_idx +=  (1 + dssize) # sizeof(len value) + dssize
                        elif(dtobjId == DataObjects.Ids.ArraySerialNumber.value): #ArraySerialNumber
                            dta_len = int.from_bytes(response[arr_idx + 4: arr_idx + 8], byteorder='little')
                            dstr = '<2HL' + str(dta_len) + 's'
                            dssize = struct.Struct(dstr).size
                            unpacked.append(DataObjects.PArraySerialNumber._make(struct.unpack(dstr, response[arr_idx:(arr_idx + dssize)])))
                            arr_idx += dssize  # DynamicLength
                        elif(dtobjId == DataObjects.Ids.NoOfStreamingSamples.value): #NoOfStreamingSamples
                            dstr = '<2HL'
                            unpacked.append(DataObjects.PNoOfStreamingSamples._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 8  # FixedLength

                        elif(dtobjId == DataObjects.Ids.DataToSend.value): #DataToSend    -------------------------------Measurement Data
                            dstr = '<2H8B'
                            unpacked.append(DataObjects.MDDataToSend._make(struct.unpack(dstr, response[arr_idx:(arr_idx+struct.Struct(dstr).size)])))
                            arr_idx += 12  # FixedLength
                        elif(dtobjId == DataObjects.Ids.VideoData.value): #VideoData
                            result = self.unpackDecodeVideoData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result
                        elif(dtobjId == DataObjects.Ids.AcousticVideoData.value): #AcousticVideoData
                            result = self.unpackDecodeAcousticVideoData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result
                        elif(dtobjId == DataObjects.Ids.SpectrumData.value): #SpectrumData
                            result = self.unpackDecodeSpectrumData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result
                        elif(dtobjId == DataObjects.Ids.AudioData.value): #AudioData
                            result = self.unpackDecodeAudioData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result
                        elif(dtobjId == DataObjects.Ids.RawData.value): #RawData
                            result = self.unpackDecodeRawData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result
                        elif(dtobjId == DataObjects.Ids.ThermalVideoData.value): #ThermalVideoData
                            result = self.unpackDecodeThermalVideoData(response=response[arr_idx:])
                            if(result is not None):
                                arr_idx += result

                        if(self.debug):
                            print('Next Idx(Len): ', arr_idx)
                            return None


                elif((cmd == CommandCodes.ResetRes.value) or 
                    (cmd == CommandCodes.StartProcedureRes.value) or 
                    (cmd == CommandCodes.StopProcedureRes.value) or
                    (cmd == CommandCodes.PrepareStateRes.value) or 
                    (cmd == CommandCodes.FinishStateRes.value) or 
                    (cmd == CommandCodes.WriteDataObjectRes.value)):
                    unpacked.append(CameraProtocol.GenericResponse._make(struct.unpack('<2BH2L', response)))

                if(self.debug and (unpacked is not None)):
                    #print('Decoding protocol version %i message' % self.pcProtocolVersion)
                    if(self.debug):
                        print('Unpacked: ', unpacked)
            self.decode(unpackedLs=unpacked)
    
    def decode(self, unpackedLs:list)->None:
        if(len(unpackedLs) == 0):
            if(self.debug):
                print('Nothing to decode')
            return None
        else: #iterative decoding and storage
            for unpacked in unpackedLs:
                data = None
                if(type(unpacked) is CameraProtocol.IdResponse):
                    devId = hex(unpacked.DeviceId).upper()
                    sserialNum = hex(unpacked.SiliconSerialNumber).upper()
                    bootLoaderVer = hex(unpacked.BootloaderVersion).upper()
                    firmwareVer = hex(unpacked.FirmwareVersion).upper()
                    prodDate = self.decodeDate(unpacked.ProductionDate)
                    calibDate = self.decodeDate(unpacked.CalibrationDate)
                    MAC = ':'.join(unpacked.MACAddress.decode('utf-8')[i:i+2] for i in range(0,12,2))
                    connectIP = socket.inet_ntoa(struct.pack('!L', unpacked.ConnectIP))
                    devState = Device.DeviceStates(unpacked.DeviceState)
                    subState = Device.SubStates(unpacked.SubState)
                    deviceIP = socket.inet_ntoa(struct.pack('!L', unpacked.IPAddress))
                    features1 = Features._make(int2ba(unpacked.Features1, 6, endian='little'))
                    status2 = Status._make(int2ba(unpacked.Status2, 5, endian='little'))
                    data = unpacked._replace(DeviceId = devId, SiliconSerialNumber = sserialNum,
                        BootloaderVersion = bootLoaderVer, FirmwareVersion = firmwareVer,              
                        IPAddress = deviceIP, ConnectIP = connectIP, MACAddress = MAC, 
                        ProductionDate = prodDate, CalibrationDate = calibDate, 
                        Features1 = features1, Status2 = status2, DeviceState = devState, SubState = subState)

                    if(unpacked.Status1 == 0):
                        if(self.debug):
                            print('IdResponse decoded: Camera OK!')
                    else:
                        print('IdResponse decoded: Camera ERROR!!')
                        self.processError(unpacked.Status1)
                elif(type(unpacked) is CameraProtocol.GenericResponse):
                    if(self.debug):
                        print('GenericResponse - [Reset/State/Procedure]')
                    data = unpacked
                    if(unpacked.Status == 0):
                        if(self.debug):
                            print('Command successful!')
                        if(data.Command == CommandCodes.StartProcedureRes.value):
                            self.stateProcStatus[DataMessages.Status.StartProcRes.value] = 1
                            self.stateProcStatus[DataMessages.Status.StopProcRes.value] = 0
                        elif(data.Command == CommandCodes.StopProcedureRes.value):
                            #print('Procedure stopped!')
                            self.stateProcStatus[DataMessages.Status.StopProcRes.value] = 1
                            self.stateProcStatus[DataMessages.Status.StartProcRes.value] = 0
                        elif(data.Command == CommandCodes.PrepareStateRes.value):
                            #print('PrepareState response!')
                            self.isConfigured = True
                            self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] = 1
                            self.stateProcStatus[DataMessages.Status.FinishStateRes.value] = 0
                        elif(data.Command == CommandCodes.FinishStateRes.value):
                            #print('State finished!')
                            self.stateProcStatus[DataMessages.Status.FinishStateRes.value] = 1
                            self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] = 0
                    else:
                        if(self.debug):
                            print('Command failed!\n')
                        if(data.Command == CommandCodes.StartProcedureRes.value):
                            self.stateProcStatus[DataMessages.Status.StartProcRes.value] = 0
                        elif(data.Command == CommandCodes.StopProcedureRes.value):
                            self.stateProcStatus[DataMessages.Status.StopProcRes.value] = 0
                        elif(data.Command == CommandCodes.PrepareStateRes.value):
                            self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] = 0
                            self.isConfigured = False
                        elif(data.Command == CommandCodes.FinishStateRes.value):
                            self.stateProcStatus[DataMessages.Status.FinishStateRes.value] = 0
                        self.processError(unpacked)
                elif(type(unpacked) is DataObjects.SDCameraStatus):
                    pass #Decode CurrentDateTime & MicStat[*]
                elif((type(unpacked) is CameraProtocol.UdpResponse)):
                    devId = hex(unpacked.DeviceId).upper()
                    sserialNum = hex(unpacked.SiliconSerialNumber).upper()
                    bootLoaderVer = hex(unpacked.BootloaderVersion).upper()
                    firmwareVer = hex(unpacked.FirmwareVersion).upper()
                    prodDate = self.decodeDate(unpacked.ProductionDate)
                    calibDate = self.decodeDate(unpacked.CalibrationDate)
                    deviceIP = socket.inet_ntoa(struct.pack('!L', unpacked.IPAddress))
                    data = unpacked._replace(DeviceId = devId, SiliconSerialNumber = sserialNum,
                        BootloaderVersion = bootLoaderVer, FirmwareVersion = firmwareVer,              
                        IPAddress = deviceIP, 
                        ProductionDate = prodDate, CalibrationDate = calibDate)
                
                self.extractData(data)
    
    def extractData(self, input)->None:
        if(type(input) is CameraProtocol.GenericResponse):
            self.Command, self.InvokeId, self.Reserved, self.Status1, self.DataLength = \
                itemgetter('Command', 'InvokeId', 'Reserved', 'Status', 'DataLength')(input._asdict())
        elif(type(input) is CameraProtocol.IdResponse):
            self.Command, self.InvokeId, self.Reserved, self.Status1, self.DataLength, self.DeviceProtocolVersion, \
            self.DeviceId, self.DeviceMode, self.DeviceError, self.DeviceSerialNumber, \
            self.SiliconSerialNumber, self.HardwareVersion, self.BootloaderVersion, self.FirmwareVersion, \
            self.FPGAVersion, self.ProductionDate, self.CalibrationDate, self.MACAddress, self.IPAddress, \
            self.PCBSerialNumber, self.Features1, self.Features2, self.Features3, self.Status2, self.ConnectIP, \
            self.DeviceState, self.SubState, self.Error, self._Step0, self._Step_1, self.TemperatureCPU, \
            self.TemperatureTop, self.TemperatureBottom, self.CPULoad_1, self.CPULoad_2 = \
                itemgetter('Command', 'InvokeId', 'Reserved', 'Status1', 'DataLength', 'DeviceProtocolVersion' \
                            ,'DeviceId', 'DeviceMode', 'DeviceError', 'DeviceSerialNumber', 'SiliconSerialNumber', 'HardwareVersion', \
                            'BootloaderVersion', 'FirmwareVersion', 'FPGAVersion', 'ProductionDate', 'CalibrationDate', 'MACAddress', \
                            'IPAddress', 'PCBSerialNumber', 'Features1', 'Features2', 'Features3', 'Status2', 'ConnectIP', 'DeviceState', \
                            'SubState', 'Error', 'Step0', 'Step_1', 'TemperatureCPU', 'TemperatureTop', 'TemperatureBottom', \
                            'CPULoad_1', 'CPULoad_2')(input._asdict())
            self.hasInitialStatus = True
        elif(type(input) is CameraProtocol.UdpResponse):
            self.Command, self.InvokeId, self.Reserved, self.Status1, self.DataLength, self.DeviceProtocolVersion, \
            self.DeviceId, self.DeviceMode, self.DeviceError, self.DeviceSerialNumber, \
            self.SiliconSerialNumber, self.HardwareVersion, self.BootloaderVersion, self.FirmwareVersion, \
            self.FPGAVersion, self.ProductionDate, self.CalibrationDate, self.MACAddress, self.IPAddress, \
            self.PCBSerialNumber = \
                itemgetter('Command', 'InvokeId', 'Reserved', 'Status1', 'DataLength', 'DeviceProtocolVersion' \
                            ,'DeviceId', 'DeviceMode', 'DeviceError', 'DeviceSerialNumber', 'SiliconSerialNumber', 'HardwareVersion', \
                            'BootloaderVersion', 'FirmwareVersion', 'FPGAVersion', 'ProductionDate', 'CalibrationDate', 'MACAddress', \
                            'IPAddress', 'PCBSerialNumber')(input._asdict())
            self.hasInitialStatus = False
        elif(type(input) is DataObjects.SDCommonStatus):
            _, _, _, self.Error, DeviceState, SubState = \
                itemgetter('ObjetID', 'ObjectType' ,'Length', 'ErrorCode',\
                            'DeviceState', 'SubState')(input._asdict())
            self.DeviceState = Device.DeviceStates(DeviceState)
            self.SubState = Device.SubStates(SubState)

    def decodeDate(self, input:bytes)-> str:
        if(len(input) == 4):#date
            return '{0}-{1}-{2}'.format(int.from_bytes(input[:2], 'little'), input[2], input[3])
        else: #datetime
            return '{0}-{1}-{2} {3}:{4}:{5} {6}'.format(int.from_bytes(input[:2], 'little'), 
                                                input[2], input[3], input[4], input[5], input[6])
    
    '''Returns the time in seconds since the FPGA start'''
    def decodeTimeStamp(self, input:int)-> float:
        return input/48828.125

    def getDeviceStatus(self):
        if(self.p_hasInitialStatus()):
            return dict({'deviceId': str(self.DeviceId),
                        'protocolVersion': self.DeviceProtocolVersion,
                        'hardwareVersion': str(self.HardwareVersion), 
                        'firmwareVersion': str(self.FirmwareVersion),
                        'deviceError': self.DeviceError,
                        'cpuTemperature': self.TemperatureCPU,
                        'temperatureTop': self.TemperatureTop,
                        'temperatureBottom': self.TemperatureBottom,
                        'deviceState': self.DeviceState,
                        'subState': self.SubState,
                        'deviceIP': self.IPAddress,
                        'deviceMAC': self.MACAddress, 
                        'Features': self.Features1,
                        'Status': self.Status2})
        return None

    def processError(self, data:GenericResponse):
        for e in Device.ErrorCodes:
            if(e.value == data.Status):
                if(self.debug):
                    print('Error reported is %s. \nCode: %i !!' % (e.name, e.value))
    
    def isStreaming(self)->bool:
        if(self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] and 
           self.stateProcStatus[DataMessages.Status.StartProcRes.value]):
            return True
        return False
    
    def setStreamFlags(self):
        self.stateProcStatus[DataMessages.Status.FinishStateRes.value] = 0
        self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] = 1
        self.stateProcStatus[DataMessages.Status.StopProcRes.value] = 0
        self.stateProcStatus[DataMessages.Status.StartProcRes.value] = 1

    def unsetStreamFlags(self):
        self.stateProcStatus[DataMessages.Status.FinishStateRes.value] = 1
        self.stateProcStatus[DataMessages.Status.PrepareStateRes.value] = 0
        self.stateProcStatus[DataMessages.Status.StopProcRes.value] = 1
        self.stateProcStatus[DataMessages.Status.StartProcRes.value] = 0
    
    def unpackDataMessage(self, response:bytes)->MDDataMessage:
        dstr = '<2BH2L'
        try:
            return MDDataMessage._make(struct.unpack(dstr, response))
        except Exception as e:
            print(e)
            return None
    
    def unpackDataObjectHeader(self, response:bytes)->DataObjects.DataObjHeader:
        dstr = '<2HL'
        try:
            #return DataObjects.DataObjHeader._make(struct.unpack(dstr, response))
            return struct.unpack(dstr, response)
        except Exception as e:
            print(e)
            return None
    
    ''' Unpacks, Decodes the received video bytes '''
    def unpackDecodeVideoData(self, data:bytes):
        try:
            dstr = '<Q2H'
            dssize = struct.calcsize(dstr)
            unpack = struct.unpack(dstr, data[:dssize])
            # vid = DataObjects.MDVideoData._make()
            # datasize = vid.VResolution * vid.HResolution
            datasize = unpack[1] * unpack[2]
            vidframe:np.array = np.array(
                        struct.unpack('<' + str(datasize) +'B', data[dssize:]), dtype=np.uint8).reshape(unpack[2], unpack[1])
            return (unpack, vidframe)
        except Exception as ex:
            print('Error in Video decode: ', ex)
            return (None, None)
    
    ''' Unpacks, Decodes the received acoustic video bytes '''
    def unpackDecodeAcousticVideoData(self, data:bytes):
        try:
            dstr = '<Q6L'
            dssize = struct.calcsize(dstr)
            unpack = struct.unpack(dstr, data[:dssize])
            #acvid = DataObjects.MDAcousticImageData._make()
            #data.seek(dssize)
            frameData:np.array = np.array(
                struct.unpack('<3072f', data[dssize:]), dtype=np.float32).reshape(48, 64)
            return (unpack, frameData)
        except Exception as ex:
            print(ex)
            return (None, None)
    
    ''' Unpacks, Decodes the received LeakRate bytes '''
    def unpackDecodeLeakRateData(self, data:bytes)->MDLeakRateData:
        dstr = '<Qf2BH'
        try:
            return MDLeakRateData._make(struct.unpack(dstr, data))
        except Exception as ex:
            print(ex)
            return None
    
    ''' Unpacks, Decodes the received spectrum bytes '''
    def unpackDecodeSpectrumData(self, data:bytes):
        if(data is None):
            return None
        try:
            dstr = '<QfH2L'
            dssize = struct.calcsize(dstr)
            spc1 = struct.unpack(dstr, data[:dssize])
            
            data_global:np.array = np.array(
                struct.unpack('<1024f', data[dssize: dssize + 4096]), dtype=np.float32)
            
            dssize += 4096
            data_local:np.array = np.array(
                struct.unpack('<1024f', data[dssize: dssize + 4096]), dtype=np.float32)
            #data_local[np.where(np.isinf(data_local))] = 0.0
            dstr = '4f'
            dssize += 4096
            spc2 = struct.unpack(dstr, data[dssize:])
            return (spc1, spc2, data_global[1:], data_local[1:])
        except Exception as ex:
            print('Error in Spectrum decode: ', ex)
            return (None, None, None, None)
    
    ''' Unpacks, Decodes the received audio bytes '''
    def unpackDecodeAudioData(self, data:bytes):
        try:
            dstr = '<Q2h'
            dssize = struct.calcsize(dstr)
            aud1 = struct.unpack(dstr, data[:dssize])
            data_plus = np.array(struct.unpack('<2048i', data[dssize: dssize + 8192]), dtype=np.int32)
            dssize += 8192
            data_minus = np.array(struct.unpack('<2048i', data[dssize: dssize + 8192]), dtype=np.int32)
            dstr = '<dH2L'
            dssize += 8192
            aud2 = struct.unpack(dstr, data[dssize:])
            #audioData = data_plus + data_minus
            return (aud1, aud2, data_plus, data_minus)
        except Exception as ex:
            print(ex)
            return (None, None, None)
    
    ''' Unpacks, Decodes the received raw bytes '''
    def unpackDecodeRawData(self, response:bytes):
        #TODO: Write the decoding process well
        if(response is None):
            return
        dta_len = int.from_bytes(response[4:8], byteorder='little')
        rawdt_len = (dta_len - 20)/4
        dstr = '<2HLQL' + str(rawdt_len) + 'ld'
        dssize = struct.Struct(dstr).size
        data = DataObjects.MDRawdata._make(struct.unpack(dstr, response))
    
    ''' Unpacks, Decodes the received thermal video bytes 
        Format 0x04:
        640 byte header + 38400 byte thermal image data.
        Header: TBD (ask CAE for details)
        Thermal image data: One U16 per pixel, line by line from left to right and top to bottom,
        each U16 value indicates the temperature in 0.1K.

        Format 0x05:
        640 byte header + 38400 byte thermal image data.
        Header: TBD (ask CAE for details)
        Thermal image data: One U16 per pixel, line by line from left to right and top to bottom,
        each U16 value indicates the temperature in 0.01K.
    '''
    def unpackDecodeThermalVideoData(self, data:bytes):
        try:
            dstr = '<Q3H'
            dssize = struct.calcsize(dstr)
            #hdr = DataObjects.MDThermalVideoData._make(struct.unpack(dstr, data.read(dssize)))
            hdr = struct.unpack(dstr, data[:dssize])
            datasize = hdr[2] * hdr[1]
            dssize += 640 #skipping header TODO: Get Header info from CAE
            vidframe:np.array = np.array(
                         struct.unpack('<' + str(datasize) +'H', data[dssize:]), dtype=np.uint16).reshape(hdr[2], hdr[1])
            if(hdr[3] == DataObjects.ThermalDatatype.TelemetryEnabledTLinearEnabledRaw14Res0_1K.value):
                vidframe = (vidframe/ 10.0)
                vidframe -= 273.15
            elif(hdr[3] == DataObjects.ThermalDatatype.TelemetryEnabledTLinearEnabledRaw14Res0_01K.value):
                vidframe = (vidframe/ 100.0)
                vidframe -= 273.15
            return (hdr, vidframe)
        except Exception as ex:
            print(ex)
            return (None, None)

    ''' Methods to write data to the device '''
    def writeDistance(self, invokeId, distance:int)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.Distance, '<2HL'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.Distance.value, 
                                            DataObjects.TypeIds.UInt32.value, distance))
        return query

    def writeFrequencyRange(self, invokeId, freqRange:tuple)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.FrequencyRange, '<2H2L'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.FrequencyRange.value, 
                                                    2, *(freqRange)))
        return query
    
    def writeCamResolution(self, invokeId, resolution:tuple)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.CameraResolution, '<4H'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.CameraResolution.value, 
                                        DataObjects.TypeIds.Resolution.value, *(resolution)))
        return query
    
    def writeVidFPS(self, invokeId, rate:int)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.VideoFrameRate, '<2HL'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.VideoFrameRate.value, 
                                                    DataObjects.TypeIds.UInt32.value, rate))
        return query
    
    def writeAcFPS(self, invokeId, rate:int)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.AcousticFrameRate, '<2HL'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', DataMessages.CommandCodes.DataMessage.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.AcousticFrameRate.value, 
                                                    DataObjects.TypeIds.UInt32.value, rate))
        return query
    
    def writeCameraLighting(self, invokeId: int, val:int)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.CameraLighting, '<2HL'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.CameraLighting.value, 
                                                    DataObjects.TypeIds.UInt32.value, val))
        return query
    
    ''' Methods to read data from the device '''
    '''
    RequestHeader ( ): Command (UInt8) | InvokeId (UInt8) | Reserved (UInt16) | DataLength (UInt32) | PCProtocolVersion (UInt32)
    RequestHeader (DataObjects /w Response): Command (UInt8) | InvokeId (UInt8) | Reserved (UInt16) | DataLength (UInt32) | DataObjectCount (UInt32) \
                    DataObject1 (ID+Type(+Length)+data of data object 1) ... DataObjectN (ID+Type(+Length)+data of data object N)
    RequestHeader (DataObjects w/o Response): Command (UInt8) | InvokeId (UInt8) | Reserved (UInt16) | DataLength (UInt32) | DataObjectCount (UInt32) \
                    DataObject1 (ID+Type(+Length)+data of data object 1) ... DataObjectN (ID+Type(+Length)+data of data object N)
    '''
    def generateReadRequest(self, command:CommandCodes, invokeId, dataLs:object=None):
        query = None
        if(command == CommandCodes.IdentificationReq):
            dataLength = 4
            query = struct.pack('<2BH2L', command.value, invokeId, 0, dataLength, self.pcProtocolVersion)
        elif(command == CommandCodes.ResetReq):
            '''Approx. 100ms delay before Device resets'''
            dataLength = 0
            query = struct.pack('<2BH2L', command.value, invokeId, 0, dataLength, self.pcProtocolVersion)
        elif(command == CommandCodes.ReadDataObjectReq):
            dataLength = 4 + (len(dataLs) * 2)
            query = struct.pack('<2BH2L', command.value, invokeId, 0, dataLength, len(dataLs))
            for dto in dataLs:
                query += struct.pack('<H', dto.value)
        else:
            print('Uknown Request!')
        return query

    '''
    This method sends the required configuration parameters to the SoundCam to begin 
    continuous measurement and data transmission
    Config:
        Distance - sDis
        Frequency range - sFre
        Camera resolution - sVRe
        Framerate acoustics - sAFr
        Acoustic Averaging - sAAvg
    '''
    def ConfigureCamera(self, invokeId, distance:int, freqRange:tuple, resolution:tuple, 
                        acousticsRate:int, acousticAvg:int)->bytes:
        dataLs=[DataObjects.DataObjPk._make((DataObjects.Ids.Distance, '<2HL')), 
                DataObjects.DataObjPk._make((DataObjects.Ids.FrequencyRange, '<2H2L')),
                DataObjects.DataObjPk._make((DataObjects.Ids.CameraResolution, '<4H')),
                DataObjects.DataObjPk._make((DataObjects.Ids.AcousticFrameRate, '<2HL')),
                DataObjects.DataObjPk._make((DataObjects.Ids.AcousticAveraging, '<2HL'))]
        dataLength = 4
        dataQuery = bytes()
        for dt in dataLs:
            dssize = struct.Struct(dt.Struct).size
            if(dt.Id == DataObjects.Ids.Distance):
                dataQuery += struct.pack(dt.Struct, *(DataObjects.Ids.Distance.value, 
                                                    DataObjects.TypeIds.UInt32.value, distance))
                dataLength += dssize
            elif(dt.Id == DataObjects.Ids.FrequencyRange):
                dataQuery += struct.pack(dt.Struct, *(DataObjects.Ids.FrequencyRange.value, 
                                                    2, *(freqRange)))
                dataLength += dssize
            elif(dt.Id == DataObjects.Ids.CameraResolution):
                dataQuery += struct.pack(dt.Struct, *(DataObjects.Ids.CameraResolution.value, 
                                                    DataObjects.TypeIds.Resolution.value, *(resolution)))
                dataLength += dssize
            elif(dt.Id == DataObjects.Ids.AcousticFrameRate):
                dataQuery += struct.pack(dt.Struct, *(DataObjects.Ids.AcousticFrameRate.value, 
                                                    DataObjects.TypeIds.UInt32.value, acousticsRate))
                dataLength += dssize
            elif(dt.Id == DataObjects.Ids.AcousticAveraging):
                dataQuery += struct.pack(dt.Struct, *(DataObjects.Ids.AcousticAveraging.value, 
                                                    DataObjects.TypeIds.UInt32.value, acousticAvg))
                dataLength += dssize
        
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, len(dataLs))
        query += dataQuery
        return query
    
    def dataToSendConfig(self, invokeId, dataToSend1:int = 0, dataToSend2:int = 0, dataToSend3:int = 0)->bytes:
        pkg = DataObjects.DataObjPk._make((DataObjects.Ids.DataToSend, '<2H8B'))
        dataLength = 4 + struct.Struct(pkg.Struct).size
        query = struct.pack('<BBHLL', CommandCodes.WriteDataObjectReq.value, invokeId, 0, dataLength, 1)
        if(dataToSend1 == 0):
            #AcVid|Vid|Thermal|Audio|MicLvl|MicRaw|Spectra|SingleMicData
            dataToSend1 = ba2int(bitarray('11010010', endian='little'))
        query += struct.pack(pkg.Struct, *(DataObjects.Ids.DataToSend.value, 
                                            3, dataToSend1, dataToSend2, dataToSend3, 0, 0, 0, 0, 0))
        return query
    
    '''
        *Sets the Device to the desired state. The device performs all necessary initializations and
        waits for the StartProcedureReq to start the actual measurement

        *Stops the measurement and transmission of all data from device to host.
        The device returns to the DeviceIdle state. A running procedure is aborted.
    '''
    def setState(self, invokeId:int, 
                 state:Device.DeviceStates= Device.DeviceStates.Unknown)->bytes:
        if(state != Device.DeviceStates.Unknown):
            #print(state, '   ', int(state.value))
            return struct.pack('<BBHLL', CommandCodes.PrepareStateReq.value, invokeId, 0, 4, state.value)
        else: #send FinishState
            return struct.pack('<BBHL', CommandCodes.FinishStateReq.value, invokeId, 0, 0)

    '''
        Starts/Stops the measurement and transmission of data from device to host
    '''
    def startStopProcedure(self, invokeId:int, startProc:bool = True)->bytes:
        if(startProc):
            return struct.pack('<BBHL', CommandCodes.StartProcedureReq.value, invokeId, 0, 0)
        else:
            return struct.pack('<BBHL', CommandCodes.StopProcedureReq.value, invokeId, 0, 0)