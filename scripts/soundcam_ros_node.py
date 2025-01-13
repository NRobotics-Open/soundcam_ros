#! /usr/bin/env python3

import rospy, time, signal
from config import cfgContext
import matplotlib
if(not cfgContext['system_run']):
    matplotlib.use('Agg')
from soundcam_connector import SoundCamConnector
from jsk_recognition_msgs.msg import Spectrum
from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from soundcam_ros.msg import SoundcamStatus, SoundcamDetection, \
    SoundcamAction, SoundcamFeedback, SoundcamGoal, SoundcamResult, \
    Preset, Blob
from soundcam_ros.srv import SoundcamService, SoundcamServiceRequest, \
    SoundcamServiceResponse
from sensor_msgs.msg import CompressedImage, CameraInfo
from diagnostic_msgs.msg import KeyValue
import actionlib, threading, numpy as np
from cv_bridge import CvBridge
from soundcam_protocol import Features, Status, MDLeakRateData
from utils_ROS import ROSLayerUtils, MissionData
from utils import SignalInfo, BlobInfo
from datetime import datetime
import cv2
import pyfakewebcam as pf
from threading import Event, Thread, Lock
from typing import List

class SoundcamROS(object):
    def __init__(self) -> None:
        self.canRun = Event()
        self.canRun.set()
        self.cfg = cfgContext
        self.debug = self.cfg['ros_debug']
        self.camera = SoundCamConnector(debug=self.cfg['driver_debug'], cfgObj=self.cfg)
        self.utils = ROSLayerUtils(debug=self.debug)
        
        #operational parameters
        self.curPreset = Preset(presetName='DEFAULT',
                                scalingMode=self.cfg['scalingMode'],
                                crest=self.cfg['crest'],
                                distance=self.cfg['distance'],
                                maximum=self.cfg['maximum'],
                                dynamic=self.cfg['dynamic'],
                                maxFrequency=self.cfg['maxFrequency'],
                                minFrequency=self.cfg['minFrequency'])
        self.curMediaType = [int(x) for x in '0|1|2|3|4|5'.split('|')] #all
        self._f_mul = self.cfg['frame_multiplier']
        if(self._f_mul <= 0.0):
            self._f_mul = 1.0
        self.curCaptureTime = self.cfg['max_record_time'] * self._f_mul #how long to record onAutoDetection
        self.autoDetect = self.cfg['auto_detect']
        self._isStartup = True

        # Stream parameters
        self._manual_frames_ls = list()
        self._manual_audio_frames_ls = list()
        self._auto_bw_frames_ls = list()
        self._auto_tm_frames_ls = list()
        self._auto_overlay_frames_ls = list()
        self._auto_audio_frames_ls = list()
        self.streamType = SoundcamServiceRequest.OVERLAY_STREAM
        self.manualTrigger = False
        self.recordTrigger = False
        self.pauseContinuous = False
        self.devStr = list()
        self.pubDevStream = self.cfg['publish_dev']
        self.prevUUID = ''
        self.missionData = MissionData('unknown-none-nothing-nada', 0, 'unset', None)
        self.curPose = [0.0, 0.0, 90.0]

        self.past_sig_i:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        self.signalInfo:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        self.tileInfo:ROSLayerUtils.TileInfo = ROSLayerUtils.TileInfo(0, 0)
        self.signalLock = Lock()
        self.tileLock = Lock()
        self.runCmp = Event()
        self.runCmp.clear()
        
        #static image caches
        self.overlayed_img = None
        self.bw_img = None
        self.ac_img = None
        self.thm_img = None

        #operational threads
        self.trigger_reset = False
        self.threads = list()
        self.threads.append(Thread(target=self.restartCamera, daemon=True))
        self.threads.append(Thread(target=self.captureDetections, daemon=True))
        #start threads
        for th in self.threads:
            th.start()

    def bringUpInterfaces(self):
        rospy.loginfo('Bringing up interfaces ...')
        if(self.pubDevStream):
            rospy.loginfo('Instantiating dev streams ...')
            self.devStr.append(pf.FakeWebcam(self.cfg['bw_virt_device'], 640, 480))

        if(not cfgContext['system_run']): #start publishers, service servers and action servers
            self.status_pub = rospy.Publisher(self.cfg['frame'] + '/status', SoundcamStatus, queue_size=3, latch=True)
            self.statusPublish(self.status_pub, self.camera.getStatus)
            self.bridge = CvBridge()
            self.detection_pub = rospy.Publisher(self.cfg['frame'] + '/detection', SoundcamDetection, queue_size=5)
            self.preset_pub = rospy.Publisher(self.cfg['frame'] + '/preset', Preset, queue_size=5)
            self.capture_pub = rospy.Publisher(self.cfg['capture_feedback_topic'], Bool, queue_size=1)
            self.caminfo_pub = rospy.Publisher(self.cfg['frame'] + '/CameraInfo', CameraInfo, queue_size=1, latch=True)
            
            thread_grp = list()
            if(self.cfg['processVideo']):
                self.vidbw_pub = rospy.Publisher(self.cfg['frame'] + '/video/bw/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidbw_pub, SoundcamServiceRequest.VIDEO_STREAM, 
                                                self.devStr[0]], 
                                        daemon=True))
            if(self.cfg['processThermal']):
                self.vidthm_pub = rospy.Publisher(self.cfg['frame'] + '/video/thermal/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidthm_pub, SoundcamServiceRequest.THERMAL_STREAM], 
                                        daemon=True))
            if(self.cfg['processAcVideo']):
                self.vidac_pub = rospy.Publisher(self.cfg['frame'] + '/video/acoustic/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidac_pub, SoundcamServiceRequest.ACOUSTIC_STREAM], 
                                        daemon=True))
            if(self.cfg['processAcVideo'] and self.cfg['processVideo']):
                self.vidoly_pub = rospy.Publisher(self.cfg['frame'] + '/video/overlay/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidoly_pub, SoundcamServiceRequest.OVERLAY_STREAM], 
                                        daemon=True))
            if(self.cfg['processAudio']):
                self.audio_info_pub = rospy.Publisher(self.cfg['frame'] + '/audio/info', AudioInfo, queue_size=3, latch=True)
                self.audio_pub = rospy.Publisher(self.cfg['frame'] + '/audio', AudioDataStamped, queue_size=15)
                thread_grp.append(threading.Thread(target=self.audioPublishing, 
                                        args=[self.audio_pub, self.audio_info_pub, 
                                                self.camera.getAudio, self.camera.getAudioInfo], 
                                        daemon=True))
            
            if(self.cfg['processSpectrum']):
                self.spectrum_pub = rospy.Publisher(self.cfg['frame'] + '/spectrum', Spectrum, queue_size=15)
                thread_grp.append(threading.Thread(target=self.spectrumPublishing, 
                                        args=[self.spectrum_pub, self.camera.getSpectrum], 
                                        daemon=True))
            
            rospy.Subscriber(self.cfg['pose_topic'], Pose, self.subRobotPose)
            rospy.Service('SoundCameraServiceServer', SoundcamService, self.serviceCB)
            self.act_srvr = actionlib.SimpleActionServer("SoundCameraActionServer", SoundcamAction, 
                                                      execute_cb=self.executeCB, auto_start=False)
            self.act_feedbk = SoundcamFeedback()
            self.act_result = SoundcamResult()
            self.act_srvr.start()

            #Send camera info
            self.publishCameraInfo()

            for th in thread_grp:
                th.start()
            rospy.loginfo("ROS Interfaces running ...")
        else:
            print("Using System Interfaces ...")
    
    def publishCameraInfo(self):
        msg = CameraInfo()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cfg['frame']
        msg.width = 640
        msg.height = 480
        msg.distortion_model = 'plumb_bob'
        msg.K = [425.0,    0. , 320.0,
                0.  , 400.0, 240.0,
                0.  ,    0. ,   1. ]
        msg.D = [0.0, 0.0, 0.0, 0.0]
        msg.R = [1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.]
        msg.P = [1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.]
        
        self.caminfo_pub.publish(msg)
    
    def disconnect(self):
        self.canRun.clear()
        self.camera.disconnect()
        print('Exiting ...')
        raise SystemExit()

    def signal_handler(self, sig, frame):
        self.canRun.clear()
        self.camera.release_shared()
        self.camera.disconnect()
        print('Exiting ...')
        raise SystemExit()
    
    '''
    -------------------------PUBLISHER METHODS
    '''

    def statusPublish(self, pub:rospy.Publisher, fn):
        meta:dict = fn()
        msg = SoundcamStatus()
        msg.header.frame_id = self.cfg['frame']
        msg.header.stamp = rospy.Time.now()
        msg.deviceId.data = meta['deviceId']
        msg.protocolVersion = meta['protocolVersion']
        msg.hardwareVersion.data = meta['hardwareVersion']
        msg.firmwareVersion.data = meta['firmwareVersion']
        msg.deviceError = meta['deviceError']
        msg.cpuTemperature.data = meta['cpuTemperature']
        msg.temperatureTop.data = meta['temperatureTop']
        msg.temperatureBottom.data = meta['temperatureBottom']
        msg.deviceState.data = meta['deviceState'].name
        msg.subState.data = meta['subState'].name
        msg.deviceIP.data = meta['deviceIP']
        msg.deviceMAC.data = meta['deviceMAC']
        featureObj:Features = meta['Features']
        for i,field in enumerate(featureObj._fields):
            msg.features.append(KeyValue(field, str(featureObj.__getitem__(i))))
        statusObj:Status = meta['Status']
        for i,field in enumerate(statusObj._fields):
            msg.features.append(KeyValue(field, str(statusObj.__getitem__(i))))
        pub.publish(msg)
        if(self.debug):
            rospy.loginfo_throttle(3, 'SC| Sent status msg')

    def videoPublishing(self, pub:rospy.Publisher, streamType, devStrObj:pf.FakeWebcam=None):
        pub_rate = 30.0
        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
            pub_rate = self.cfg['videoFPS']
        elif(streamType == SoundcamServiceRequest.ACOUSTIC_STREAM):
            pub_rate = self.cfg['acimageFPS']
        rate = rospy.Rate(pub_rate)
        while(not rospy.is_shutdown() and self.canRun.is_set()):
            if(not self.pauseContinuous):
                frame = self._getFrame(streamType=streamType)
                if(frame is not None):
                    # Publish to DEV
                    if(self.pubDevStream and devStrObj is not None and 
                       (streamType == SoundcamServiceRequest.VIDEO_STREAM)):
                        processed_frame = self.utils.publishDevStream(frame)
                        devStrObj.schedule_frame(processed_frame)

                    # MANUAL stream recording
                    if(self.manualTrigger and (self.streamType == streamType)):
                        self._manual_frames_ls.append(frame)
                        self.utils.limitMemUsage(self._manual_frames_ls, self.cfg['max_megabytes'])
                    
                    # # AUTO stream recording
                    if(self.recordTrigger):
                        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                            self._auto_bw_frames_ls.append(frame)
                            self.utils.limitMemUsage(self._auto_bw_frames_ls, self.cfg['max_megabytes'])
                        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                            self._auto_tm_frames_ls.append(frame)
                            self.utils.limitMemUsage(self._auto_tm_frames_ls, self.cfg['max_megabytes'])
                        elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                            self._auto_overlay_frames_ls.append(frame)
                            self.utils.limitMemUsage(self._auto_overlay_frames_ls, self.cfg['max_megabytes'])

                    if(pub.get_num_connections() > 0): # topic publishing
                        # if(self.debug):
                        #     rospy.loginfo_once('SC| Video Streaming on Stream type -> {0}'.format(streamType))
                        self.convertPublishCompressedImage(pub=pub, cv_image=frame)
            rate.sleep()
    
    def audioPublishing(self, pub:rospy.Publisher, pubinfo:rospy.Publisher, fn, fninfo):
        rate = rospy.Rate(40)
        msg = AudioDataStamped()
        msg.header.frame_id = self.cfg['frame']
        msginfo = AudioInfo()
        start_t = rospy.Time.now().secs
        info_published = False
        while(not rospy.is_shutdown() and self.canRun.is_set()):
            aud_arr:np.array = fn()

            if(self.manualTrigger):
                self._manual_audio_frames_ls.append(aud_arr)
                self.utils.limitMemUsage(self._manual_audio_frames_ls, self.cfg['max_megabytes'])

            if(self.recordTrigger):
                self._auto_audio_frames_ls.append(aud_arr)
            
            if(pub.get_num_connections() > 0):
                msg.header.stamp = rospy.Time.now()
                if(aud_arr is not None):
                    aud_arr = aud_arr/ np.max(aud_arr)
                    aud_arr = 255 * aud_arr
                    aud_arr = aud_arr.astype(np.uint8)
                    msg.audio.data = aud_arr.flatten().tolist()
                    pub.publish(msg)
                    if(self.debug):
                        rospy.loginfo_throttle(3, 'SC| Streaming audio')
            if((pubinfo.get_num_connections() > 0) and 
               ((rospy.Time.now().secs - start_t) > 1.0) and 
               not info_published): # publish audio info
                meta = fninfo()
                if(meta):
                    msginfo.channels = meta['channels']
                    msginfo.sample_rate = meta['sample_rate']
                    msginfo.bitrate = meta['bitrate']
                    pubinfo.publish(msginfo)
                    start_t = rospy.Time.now().secs
                    info_published = True
            else:
                info_published = False
            rate.sleep()

    def spectrumPublishing(self, pub:rospy.Publisher, fn):
        rate = rospy.Rate(25)
        msg = Spectrum()
        msg.header.frame_id = self.cfg['frame']
        while(not rospy.is_shutdown() and self.canRun.is_set()):
            if(pub.get_num_connections() > 0):
                msg.header.stamp = rospy.Time.now()
                (frequency, amplitude) = fn()
                msg.frequency = frequency.flatten().tolist()
                msg.amplitude = amplitude.flatten().tolist()
                pub.publish(msg)
                if(self.debug):
                    rospy.loginfo_throttle(5, 'SC| Streaming spectrum data')
            rate.sleep()
    
    ''' Sends alert to other nodes about current detections '''
    def publishDetection(self, sigInfo:SignalInfo, blobCoords:List[BlobInfo]):
        if(self.detection_pub.get_num_connections() > 0):
            msg = SoundcamDetection()
            msg.header.frame_id = self.cfg['frame']
            msg.header.stamp = rospy.Time.now()
            msg.preset = self.curPreset
            msg.meanEnergy.data = sigInfo.mean_energy
            msg.stddev.data = sigInfo.std_dev
            msg.highEnergyThreshold.data = sigInfo.hi_thresh
            msg.currentEnergy.data = sigInfo.current_energy
            msg.lowEnergyThreshold.data = sigInfo.lo_thresh
            msg.snr.data = sigInfo.snr
            msg.acousticEnergy.data = sigInfo.acoustic_energy
            msg.preActivation.data = sigInfo.pre_activation
            msg.detection.data = sigInfo.detection
            if(len(blobCoords) > 0):
                for dt in blobCoords:
                    blob:Blob = Blob(**dt._asdict())
                    msg.blobs.append(blob)
            self.detection_pub.publish(msg)
            if(self.debug):
                rospy.loginfo_throttle(3, 'SC| Published detection status')
    
    ''' Publishes current camera preset '''
    def publishPreset(self):
        if(self.preset_pub.get_num_connections() > 0):
            self.preset_pub.publish(self.curPreset)

    def publishCaptureFeedback(self, pub:rospy.Publisher):
        pub.publish(Bool(True))
        if(self.debug):
            rospy.loginfo_throttle(3, 'SC| Published capture feedback')
    
    def convertPublishCompressedImage(self, pub:rospy.Publisher, cv_image):
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = rospy.Time.now()
        compressed_image_msg.header.frame_id = self.cfg['frame']
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()

        # Publish the compressed image
        pub.publish(compressed_image_msg)

    '''
    -------------------------other METHODS
    '''
    def restartCamera(self):
        while(self.canRun.is_set()):
            if(self.trigger_reset):
                if(self.camera.restartCamera()):
                    self.setPreset(self.curPreset) #reset preset
                self.trigger_reset = False
            time.sleep(1.0)
    
    def setPreset(self, preset:Preset):
        if(preset.presetName == self.curPreset.presetName):
            return True
        return self.camera.updatePreset(mode=int(preset.scalingMode),
                                        distance=int(preset.distance),
                                        minFreq=int(preset.minFrequency),
                                        maxFreq=int(preset.maxFrequency),
                                        dynamic=float(preset.dynamic),
                                        crest=float(preset.crest),
                    maximum=float(preset.maximum) if (float(preset.maximum) > 0.0) else None)
                
    def subRobotPose(self, msg:Pose):
        try:
            orientation_euler = self.utils.quaternionToEulerDegrees(msg.orientation.w, 
                                                            msg.orientation.x,
                                                            msg.orientation.y,
                                                            msg.orientation.z)
            self.curPose = [msg.position.x, msg.position.y, orientation_euler[2]]
        except Exception as e:
            rospy.logerr(e)

        
    '''
    -------------------------SERVICE SERVER
    '''

    ''' Returns a single frame using the given function '''
    def _getFrame(self, streamType=SoundcamServiceRequest.OVERLAY_STREAM):
        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
            img = self.camera.getBWVideo()
            if(img is not None):
                self.bw_img = img
                return self.bw_img
            else:
                return self.bw_img
        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
            img = self.camera.getTMVideo()
            if(img is not None):
                self.thm_img = img
                return self.thm_img
            else:
                return self.thm_img
        elif(streamType == SoundcamServiceRequest.ACOUSTIC_STREAM):
            img = self.camera.getACVideo()
            if(img is not None):
                self.ac_img = img
                return self.ac_img
            else:
                return self.ac_img
        elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
            bw_img = self.camera.getBWVideo()
            ac_img = self.camera.getACVideo()
            if((bw_img is not None) and (ac_img is not None)):
                self.bw_img = bw_img
                self.ac_img = ac_img
                self.overlayed_img = self.camera.drawRect(frame=self.utils.imageOverlay(self.bw_img, self.ac_img))
                return self.overlayed_img
            else:
                return self.overlayed_img
        


    ''' Takes/Saves a snapshot of the given stream and returns True/False
        0 -> BW Image
        3 -> Thermal Image
        5 -> Overlayed Image
        6 -> [BW Image, Thermal Image, Overlayed Image]
    '''
    def _takeSnapshot(self, streamType=SoundcamServiceRequest.OVERLAY_STREAM, 
                      wpInfo:ROSLayerUtils.WaypointInfo=None, sigInfo:SignalInfo=None, 
                      tileInfo:ROSLayerUtils.TileInfo=None):
        isActPoint = True
        media = list()
        frame_data = list()
        if(tileInfo is None):
            tileInfo = ROSLayerUtils.TileInfo(0, 0)

        if(streamType == SoundcamServiceRequest.ALL):
            try:
                sfx = ''.join(['BW_', str(tileInfo.id)])
                filename = self.utils.getUniqueName(suffix=sfx)
                frame = self._getFrame(streamType=SoundcamServiceRequest.VIDEO_STREAM)
                if(frame is not None):
                    frame_data.append((frame, filename))

                sfx = ''.join(['THM_', str(tileInfo.id)])
                filename = self.utils.getUniqueName(suffix=sfx)
                frame = self._getFrame(streamType=SoundcamServiceRequest.THERMAL_STREAM)
                if(frame is not None):
                    frame_data.append((frame, filename))

                sfx = ''.join(['OV_', str(tileInfo.id)])
                filename = self.utils.getUniqueName(suffix=sfx)
                frame = self._getFrame(streamType=SoundcamServiceRequest.OVERLAY_STREAM)
                if(frame is not None):
                    frame_data.append((frame, filename))
                
                if(len(frame_data) == 0):
                    return False
                
                for fobj in frame_data: #write snapshots
                    self.utils.createSnapshotFromFrame(fobj[0], filename=fobj[1])
                    media.append(fobj[1])
            except Exception as e:
                rospy.logerr('SC| Error taking snapshots [ALL]: ', e)
                return False
        else: # selective image save
            if (isinstance(streamType, list)): #if streamType is a list
                def _streamTypeIter(strmLs:list, mediaLs:list, tile:ROSLayerUtils.TileInfo):
                    for stream in strmLs:
                        if(stream == SoundcamServiceRequest.VIDEO_STREAM):
                            frame = self._getFrame(streamType=SoundcamServiceRequest.VIDEO_STREAM)
                            if(frame is not None):
                                sfx = ''.join(['BW_', str(tileInfo.id)])
                                filename = self.utils.getUniqueName(suffix=sfx)
                                frame_data.append((frame, filename))
                        elif(stream == SoundcamServiceRequest.THERMAL_STREAM):
                            frame = self._getFrame(streamType=SoundcamServiceRequest.THERMAL_STREAM)
                            if(frame is not None):
                                sfx = ''.join(['THM_', str(tileInfo.id)])
                                filename = self.utils.getUniqueName(suffix=sfx)
                                frame_data.append((frame, filename))
                        elif(stream == SoundcamServiceRequest.OVERLAY_STREAM):
                            frame = self._getFrame(streamType=SoundcamServiceRequest.OVERLAY_STREAM)
                            if(frame is not None):
                                sfx = ''.join(['OV_', str(tileInfo.id)])
                                filename = self.utils.getUniqueName(suffix=sfx)
                                frame_data.append((frame, filename))
                        
                        if(len(frame_data) == 0):
                            return False
                        for fobj in frame_data: #write snapshots
                            self.utils.createSnapshotFromFrame(fobj[0], filename=fobj[1])
                            mediaLs.append(fobj[1])
                try:
                    _streamTypeIter(streamType, mediaLs=media, tile=tileInfo)
                except Exception as e:
                    rospy.logerr('SC| Error taking snapshots [List]: ', e)
                    return False
            else: # if streamType is a single type
                try:
                    if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                        frame = self._getFrame(streamType=SoundcamServiceRequest.VIDEO_STREAM)
                        if(frame is None):
                            return False
                        else:
                            sfx = ''.join(['BW_', str(tileInfo.id)])
                            filename = self.utils.getUniqueName(suffix=sfx)
                    elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                        frame = self._getFrame(streamType=SoundcamServiceRequest.THERMAL_STREAM)
                        if(frame is None):
                            return False
                        else:
                            sfx = ''.join(['THM_', str(tileInfo.id)])
                            filename = self.utils.getUniqueName(suffix=sfx)
                    elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                        frame = self._getFrame(streamType=SoundcamServiceRequest.OVERLAY_STREAM)
                        if(frame is None):
                            return False
                        else:
                            sfx = ''.join(['OV_', str(tileInfo.id)])
                            filename = self.utils.getUniqueName(suffix=sfx)
                except Exception as e:
                    rospy.logerr('SC| Error taking BW/THM/OV snapshot: ', e)
                    return False
        
        if((wpInfo is not None) and (len(media) > 0)):
            self.utils.addMetaData(
                wpInfo=ROSLayerUtils.WaypointInfo(*wpInfo),
                media=media, 
                sigInfo=SignalInfo(*sigInfo),
                isActionPoint=isActPoint,
                relevantIdx=tileInfo.relId,
                preset=self.curPreset,
                loop=self.curLoop,
                useMsnPath=True)
        self.publishCaptureFeedback(self.capture_pub)
        if(self.debug):
            rospy.loginfo_throttle(1, 'SC| Snapshot success!')
        return True
    
    ''' Save recorded stream and returns True/False'''
    def _saveRecording(self, auto=False, start_t=time.time(), 
                       streamType=SoundcamServiceRequest.ALL,
                       sigInfo:SignalInfo=None,
                       wpInfo:ROSLayerUtils.WaypointInfo=None,
                       tileInfo:ROSLayerUtils.TileInfo=None,
                       isActPoint=False):
        if(not auto): # manual recording save
            try:
                self.manualTrigger = False
                self.utils.createVideoFromFrames(self._manual_frames_ls)
                self.utils.createAudioFromFrames(self._manual_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'])
                self.publishCaptureFeedback(self.capture_pub)
                if(self.debug):
                    rospy.loginfo_throttle(1, 'SC| Manual Recording success!')
                return True
            except Exception as e:
                rospy.logerr("SC| Error saving manual recording, ", e)
                self._manual_audio_frames_ls.clear()
                self._manual_frames_ls.clear()
                if(self.debug):
                    rospy.logerr_throttle(1, 'SC| Manual Recording failure!')
                return False
        else: # auto recording save
            self.recordTrigger = False
            try:
                auto_elapsed_t = time.time() - start_t
                if(auto_elapsed_t >= self.cfg['min_record_time']):
                    timestamp = datetime.now().strftime("%H_%M_%S")
                    media = list()
                    if(tileInfo is None):
                        tileInfo = ROSLayerUtils.TileInfo(0, 0)
                    # if(wpInfo is None):
                    #     wpInfo = ROSLayerUtils.WaypointInfo(0, *self.curPose)
                    
                    if(streamType == SoundcamServiceRequest.ALL):
                        sfx = ''.join(['BW_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, filename)
                        media.append(filename)

                        sfx = ''.join(['THM_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, filename)
                        media.append(filename)

                        sfx = ''.join(['OV_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, filename)
                        media.append(filename)

                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                        sfx = ''.join(['BW_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                        sfx = ''.join(['THM_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                        sfx = ''.join(['OV_', str(tileInfo.id)])
                        filename = self.utils.getUniqueName(isImg=False, suffix=sfx)
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)
                    
                    if((wpInfo is not None) and (sigInfo is not None) and (len(media) > 0)):
                        self.utils.addMetaData(
                            wpInfo=ROSLayerUtils.WaypointInfo(*wpInfo),
                            media=media, 
                            sigInfo=SignalInfo(*sigInfo),
                            isActionPoint=isActPoint,
                            relevantIdx=tileInfo.relId,
                            useMsnPath=True)
                    self.publishCaptureFeedback(self.capture_pub)
                else:
                    rospy.loginfo('SC| Recording time %fs is less the minimum specified. Discarding ...' % auto_elapsed_t)
                    self._auto_overlay_frames_ls.clear()
                    self._auto_bw_frames_ls.clear()
                    self._auto_tm_frames_ls.clear()
                    self._auto_audio_frames_ls.clear()
            except Exception as e:
                self._auto_overlay_frames_ls.clear()
                self._auto_bw_frames_ls.clear()
                self._auto_tm_frames_ls.clear()
                self._auto_audio_frames_ls.clear()
                print('SC| Error AUTOMATICALLY saving streams: ', e)
                if(self.debug):
                    rospy.logerr('SC| Auto Recording failure!')
                return False
            if(self.debug):
                rospy.loginfo('SC| Auto Recording success!')
            return True

    
    ''' Service callback for Configuration and Operation '''
    def _handle_reset_camera(self, req:SoundcamServiceRequest):
        if not self.camera.restartCamera():
            return SoundcamServiceResponse.FAIL
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_start_record(self, req:SoundcamServiceRequest):
        stream_type = req.op_command1
        if stream_type not in [SoundcamServiceRequest.VIDEO_STREAM,
                            SoundcamServiceRequest.ACOUSTIC_STREAM,
                            SoundcamServiceRequest.THERMAL_STREAM,
                            SoundcamServiceRequest.OVERLAY_STREAM]:
            if(self.debug):
                rospy.logerr(f"Unknown stream type: {stream_type}")
            return SoundcamServiceResponse.FAIL
        self.streamType = stream_type
        self.manualTrigger = True
        if(self.debug):
            rospy.loginfo(f"Recording started: Type -> {stream_type}")
        return SoundcamServiceResponse.SUCCESS

    def _handle_stop_record(self, req:SoundcamServiceRequest):
        if not self._saveRecording():
            return SoundcamServiceResponse.FAIL
        if(self.debug):
            rospy.loginfo("Recording saved")
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_cancel_record(self, req:SoundcamServiceRequest):
        self.manualTrigger = False
        self._manual_frames_ls.clear()
        if(self.debug):
            rospy.loginfo('Recording cancelled!')
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_snapshot(self, req:SoundcamServiceRequest):
        if(not self._takeSnapshot(int(req.op_command1))):
            return SoundcamServiceResponse.FAIL
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_set_dynamic_crest(self, req:SoundcamServiceRequest):
        (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
        dynamic = req.op_command1
        crest = req.op_command2
        if(not self.camera.setScalingMode(mode=mode, dynamic=dynamic, 
                                            crest=crest, max=maximum)):
            return SoundcamServiceResponse.FAIL
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_set_scaling_mode(self, req:SoundcamServiceRequest):
        (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
        mode = int(req.op_command1)
        if(mode == 0):
            maximum = req.op_command2
        else:
            dynamic = req.op_command2
            crest = req.op_command3
        if(not self.camera.setScalingMode(mode=mode, dynamic=dynamic, 
                            max=None, crest=crest)):
            return SoundcamServiceResponse.FAIL
        return SoundcamServiceResponse.SUCCESS

    def _handle_scan_detect(self, req:SoundcamServiceRequest):
        self.autoDetect = True if (req.op_command2 == 1) else False
        return SoundcamServiceResponse.SUCCESS
    
    def _handle_extra_parameters(self, req:SoundcamServiceRequest):
        if(len(req.extras) > 0):
            for param in req.extras:
                if(param.key == 'uuid'):
                    self.missionData.uuid = param.value
                if(param.key == 'missionId'):
                    self.missionData.id = int(param.value)
                if(param.key == 'missionName'):
                    self.missionData.name = param.value
                if(param.key == 'currentLoop'):
                    self.curLoop = int(param.value)
                if(param.key == 'resultDirectory'):
                    self.missionData.result_dir = param.value
    
    def _handle_preset_change(self, req:SoundcamServiceRequest):
        if(req.preset.presetName != self.curPreset.presetName):
            if(self.setPreset(req.preset)):
                rospy.loginfo('Preset sent!')
                start_t = time.time()
                while(not self.camera.isDetectionReady()):
                    rospy.loginfo_throttle(5, "Awaiting Detection algo ...")
                    if((time.time() - start_t) >= 10.0):
                        rospy.logwarn("Camera stream taking longer to resume \
                                    \nCamera might be in Error!")
                        rospy.loginfo("Camera will be restarted ...")
                        self.trigger_reset = True
                        break
            else:
                rospy.logerr('Preset change failed!')
                if(not self.camera.isMeasuring()):
                    rospy.loginfo("Camera will be restarted ...")
                    self.trigger_reset = True
                return SoundcamServiceResponse.FAIL
        else:
            if(self.debug):
                rospy.loginfo("%s is already set!" % self.curPreset.presetName)
        return SoundcamServiceResponse.SUCCESS


    def _get_command_dispatcher(self):
        return {
            SoundcamServiceRequest.RESET_CAMERA: self._handle_reset_camera,
            SoundcamServiceRequest.START_RECORD: self._handle_start_record,
            SoundcamServiceRequest.STOP_RECORD: self._handle_stop_record,
            SoundcamServiceRequest.CANCEL_RECORD: self._handle_cancel_record,
            SoundcamServiceRequest.SNAPSHOT: self._handle_snapshot,
            SoundcamServiceRequest.SET_DYNAMIC_CREST: self._handle_set_dynamic_crest,
            SoundcamServiceRequest.SET_SCALING_MODE: self._handle_set_scaling_mode,
            SoundcamServiceRequest.SCAN_DETECT: self._handle_scan_detect
        }
    
    def serviceCB(self, req:SoundcamServiceRequest):
        if(self.debug):
            rospy.loginfo("Incoming Service Call ...")
            rospy.loginfo("\tCommand Type: %s" % 
                    ("CONFIG" if (req.command_type == SoundcamServiceRequest.CMD_TYPE_CONFIG) else "OP"))
            rospy.loginfo("\t CaptureTime: %f" % req.captureTime)
            if(req.preset.hasPreset):
                rospy.loginfo("\t Preset: ")
                rospy.loginfo("\t\t Scaling Mode: %f" % req.preset.scalingMode)
                rospy.loginfo("\t\t Crest: %f" % req.preset.crest)
                rospy.loginfo("\t\t Distance: %f" % req.preset.distance)
                rospy.loginfo("\t\t Maximum: %f" % req.preset.maximum)
                rospy.loginfo("\t\t Dynamic: %f" % req.preset.dynamic)
                rospy.loginfo("\t\t Max. Frequency: %i" % req.preset.maxFrequency)
                rospy.loginfo("\t\t Min. Frequency: %i" % req.preset.minFrequency)
            rospy.loginfo("\t Media Type: %s" % req.mediaType)
            rospy.loginfo("\t Command: %i" % req.command)
            rospy.loginfo("\t\t Op. Cmd-1: %i" % req.op_command1)
            rospy.loginfo("\t\t Op. Cmd-2: %i" % req.op_command2)
            rospy.loginfo("\t\t Op. Cmd-3: %i" % req.op_command3)
            try:
                if(len(req.extras) > 0):
                    rospy.loginfo("\t\t Extras: ")
                    for kv in req.extras:
                        dt:KeyValue = kv
                        rospy.loginfo("\t\t\t Key: {}, Value: {}".format(dt.key, dt.value))
            except Exception:
                pass
        
        resp = SoundcamServiceResponse()
        resp.results = SoundcamServiceResponse.SUCCESS

        # Get the command dispatcher
        command_dispatcher = self._get_command_dispatcher()

        # Get Extra Parameters
        self._handle_extra_parameters(req=req)

        if(req.command_type == SoundcamServiceRequest.CMD_TYPE_CONFIG):
            try:
                self.curCaptureTime = req.captureTime #if (req.captureTime > 0.0) else self.curCaptureTime
                if(req.mediaType != ''):
                    media = [int(x) for x in req.mediaType.split('|')]
                    if(SoundcamServiceRequest.ALL not in media):
                        self.curMediaType = media
                    else:
                        self.curMediaType = SoundcamServiceRequest.ALL

                #Configure camera preset
                if(req.preset.hasPreset):
                    self._handle_preset_change(req=req)
            except Exception as e:
                rospy.logerr("Error occured! ", e)
                resp.results = SoundcamServiceResponse.FAIL
        elif(req.command_type == SoundcamServiceRequest.CMD_TYPE_OP):
            # Check if the command is in the dispatcher
            if req.command in command_dispatcher:
                try:
                    # Call the appropriate handler
                    result = command_dispatcher[req.command](req)
                    resp.results = result
                except Exception as e:
                    rospy.logerr(f"Error handling command {req.command}: {e}")
                    resp.results = SoundcamServiceResponse.FAIL
            else:
                rospy.logerr(f"Unknown Operation command: {req.command}")
                resp.results = SoundcamServiceResponse.FAIL
        else:
            rospy.logerr(f"Unknown command: {req.command}")
            resp.results = SoundcamServiceResponse.FAIL
                
        if(self.debug):
            if(resp.results == SoundcamServiceResponse.SUCCESS):
                rospy.loginfo("SC| Service Call success!")
            else:
                rospy.logerr("SC| Service Call failure!")
        resp.ack.data = True
        return resp
    
    
    '''
    -------------------------ACTION SERVER
    '''

    def prepareMissionDirectory(self):
        if(self.prevUUID != self.missionData.uuid):
            self.utils.prepareDirectory(str(self.missionData.id), self.missionData.name)
            self.prevUUID = self.missionData.uuid
            if(self.debug):
                rospy.loginfo('UUID - %s' % self.missionData.uuid)
                rospy.loginfo('Newly created path is: %s' % self.utils.getPath(fetchMsnDir=True))

    def captureDetections(self):
        while(self.canRun.is_set()):
            if(self.runCmp.is_set()):
                # Capture detection values
                with self.signalLock:
                    siginfo = self.signalInfo._asdict()
                    prv_siginfo = self.past_sig_i._asdict()

                if(prv_siginfo['mean_energy'] == 0.0 and 
                prv_siginfo['std_dev'] == 0.0 and 
                prv_siginfo['acoustic_energy'] == 0.0 and 
                prv_siginfo['current_energy'] == 0.0):
                    prv_siginfo = siginfo
                
                if((not prv_siginfo['detection']) and siginfo['detection']):
                    prv_siginfo['detection'] = siginfo['detection']

                if(prv_siginfo['current_energy'] < siginfo['current_energy']):
                    prv_siginfo['current_energy'] = siginfo['current_energy']

                if(siginfo['mean_energy'] > prv_siginfo['mean_energy']):
                    prv_siginfo['mean_energy'] = siginfo['mean_energy']
                    prv_siginfo['snr'] = siginfo['snr']
                    prv_siginfo['std_dev'] = siginfo['std_dev']

                if(siginfo['hi_thresh'] > prv_siginfo['hi_thresh']):
                    prv_siginfo['hi_thresh'] = siginfo['hi_thresh']
                if(siginfo['lo_thresh'] > prv_siginfo['lo_thresh']):
                    prv_siginfo['lo_thresh'] = siginfo['lo_thresh']
                if(siginfo['acoustic_energy'] > prv_siginfo['acoustic_energy']):
                    prv_siginfo['acoustic_energy'] = siginfo['acoustic_energy']
                    with self.tileLock:
                        tile_i = self.tileInfo._asdict()
                        tile_i['relId'] = tile_i['id']
                        self.tileInfo = ROSLayerUtils.TileInfo(**tile_i)
            else:
                time.sleep(0.1)
            

    def executeCB(self, goal: SoundcamGoal):
        rospy.loginfo('SC| Received goal!')

        start_t = time.time()
        can_proceed = True
        while(not self.camera.isDetectionReady()):
            rospy.loginfo_throttle(5, "Awaiting detection algorithm to resume ...")
            if((time.time() - start_t) >= 5.0):
                can_proceed = False
                rospy.logerr("SC| Camera in Error!")
                rospy.logerr("SC| Goal will be aborted!")
                break
        
        if(can_proceed):
            #save detection/recording parameters
            tmp_autoDetect = self.autoDetect
            self.autoDetect = False
            self.recordTrigger = False #deactivate record triggers
            
            #extract parameters
            rospy.loginfo('Extracting goal parameters ...')
            streamType = SoundcamServiceRequest.ALL
            for param in goal.parameters:
                if(param.key == 'uuid'):
                    self.missionData.uuid = param.value
                if(param.key == 'delay'):
                    delay = int(param.value)
                if(param.key == 'numCaptures'):
                    numCaptures = int(param.value)
                if(param.key == 'recordTime'):
                    recordTime = float(param.value) * self._f_mul
                if(param.key == 'mediaType'):
                    if(param.value != ''):
                        media = [int(x) for x in param.value.split('|')]
                        if(SoundcamServiceRequest.ALL not in media):
                            streamType = media
                if(param.key == 'missionId'):
                    self.missionData.id = int(param.value)
                if(param.key == 'missionName'):
                    self.missionData.name = param.value
                if(param.key == 'currentLoop'):
                    self.curLoop = int(param.value)
                if(param.key == 'waypointId'):
                    wpId = int(param.value)
                if(param.key == 'waypointX'):
                    wpX = float(param.value)
                if(param.key == 'waypointY'):
                    wpY = float(param.value)
                if(param.key == 'waypointTheta'):
                    wpTheta = float(param.value)
                if(param.key == 'resultDirectory'):
                    self.missionData.result_dir = param.value
                if(param.key == 'imageTileNo'): #default = 0
                    tile_no = int(param.value)
                
            #initialize containers
            wpInfo = ROSLayerUtils.WaypointInfo(wpId, wpX, wpY, wpTheta)
            with self.tileLock:
                self.tileInfo = ROSLayerUtils.TileInfo(tile_no, tile_no)

            #run while loop
            rate = rospy.Rate(15)
            result = False
            cnt = 1

            if(self.tileInfo.id <= 1):
                with self.signalLock:
                    self.past_sig_i:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
            
            #prepare directory
            self.prepareMissionDirectory()
            self.runCmp.set() #activates detectCompare thread
            time.sleep(1.0)
            
            rospy.loginfo('Processing goal ...')
            while(not rospy.is_shutdown()):
                if((recordTime <= self.cfg['min_record_time']) and (numCaptures > 0)): #Take Snapshots
                    with self.signalLock:
                        with self.tileLock:
                            res = self._takeSnapshot(streamType=streamType, 
                                          wpInfo=wpInfo, sigInfo=SignalInfo(*self.past_sig_i),
                                          tileInfo=ROSLayerUtils.TileInfo(*self.tileInfo))
                    if(res):
                        cnt += 1
                        self.act_feedbk.capture_count = cnt
                        self.act_feedbk.currentTime.data = rospy.Time.now()
                        self.act_srvr.publish_feedback(self.act_feedbk)
                        time.sleep(delay)
                    else:
                        rospy.logerr('SC| Snapshot failed!')
                        break

                    if(cnt > numCaptures):
                        result = True
                        break
                elif((recordTime > self.cfg['min_record_time']) and (numCaptures > 0)): #Make Video Recordings
                    if(not self.recordTrigger): #start recording
                        self.recordTrigger = True
                        record_start_t = time.time()
                    else: #while recording
                        if((time.time()-record_start_t) >= recordTime): #save recording
                            rospy.loginfo('SC| Saving recording ...')
                            with self.signalLock:
                                with self.tileLock:
                                    res = self._saveRecording(auto=True, isActPoint=True, 
                                                streamType=streamType,
                                                start_t=record_start_t, 
                                                wpInfo=wpInfo,
                                                sigInfo=SignalInfo(*self.past_sig_i),
                                                tileInfo=ROSLayerUtils.TileInfo(*self.tileInfo))
                            if(res):
                                cnt += 1
                                with self.signalLock:
                                    self.past_sig_i:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
                                time.sleep(delay)
                        
                        if(cnt > numCaptures):
                            result = True
                            break
                        else:
                            rospy.loginfo_throttle(3, 'SC| Recording [%i] in progress ...' % cnt)
                            self.act_feedbk.capture_count = cnt
                            self.act_feedbk.currentTime.data = rospy.Time.now()
                            self.act_srvr.publish_feedback(self.act_feedbk)

                else:
                    rospy.logerr('SC| Snapshot nichts zu tun!')
                    result = False
                    break
                rate.sleep()

        #on loop exit
        self.act_result.result = result
        if(not result):
            self.act_srvr.set_aborted(self.act_result)
        else:
            self.act_srvr.set_succeeded(self.act_result)
        #restore detection settings and reset rest period
        self.restDist = time.time()
        self.autoDetect = tmp_autoDetect
        self.runCmp.clear()


    '''
        Ping, Connect & Configure Camera
    '''
    def attemptConnection(self):
        rate = rospy.Rate(1.0)
        conn_counter = 1
        max_attempts = 5
        while(not rospy.is_shutdown() and self.canRun):
            if(self.trigger_reset):
                rospy.loginfo_throttle(3.0, 'Waiting for Camera Restart to finish ...')
                rate.sleep()
                continue

            if(self.camera.pingCamera(deviceIP=self.cfg['ip'])):
                if(self.camera.AKAMsAssemble()):
                    if(conn_counter >= max_attempts):
                        print('Max connect attempts surpassed: will trigger RESTART')
                        self.trigger_reset = True
                        conn_counter = 1
                        continue
                    
                    status:Status = self.camera.getStatus()['Status']
                    print('Camera Status (attemptconnection): ', status)

                    if((status.isConnectedHost == 0)):
                        if(self.camera.reconnect(max_retries=2)):
                            rospy.loginfo('Connection established!')
                            while(not self.camera.hasStatus()):
                                rospy.loginfo_throttle(1.5, 'Awaiting hasStatus() ...')
                            continue
                        else:
                            conn_counter += 1
                            self.camera.disconnect()
                            rospy.logwarn('Retrying to connect in 5 seconds ...')
                            rospy.sleep(5.0)
                            continue
                    else:
                        if(self.camera.hasStatus()):
                            rospy.loginfo('Initial Device status received...')
                            rospy.loginfo('Sending Initial configuration and preparing device ...')
                            self.camera.initialConfigure()
                            self.camera.startMeasurement()
                            break
                        else:
                            rospy.loginfo('Awaiting Initial Device status ...')
                else: #on_detectDevice failure
                    print('Waiting for a device to become available ...')
                    rospy.sleep(1.5)
            else:
                rospy.loginfo('Unable to reach SoundCamera on IP:{0} \n \
                              Trying again after {1} seconds'.format(self.cfg['ip'], 5))
                time.sleep(5.0)
            rate.sleep()
        return (self.camera.isConnected())

    def run(self):
        rate = rospy.Rate(40.0)
        has_updated_record_start_t = False
        record_start_t = time.time()
        disconnect_t = time.time()
        disconnect_ev = False
        prevPose = [0.0, 0.0, 0.0]
        firstrun = True

        alive_t = time.time()

        while(not rospy.is_shutdown()):
            if(self._isStartup):
                if(self.attemptConnection()):
                    rospy.loginfo('[X] Camera connected!')
                    if(firstrun):
                        self.bringUpInterfaces()
                        firstrun = False
                    self._isStartup = False
            else: 
                with self.signalLock: #always update signal Info
                    self.signalInfo:SignalInfo = SignalInfo(*self.camera.getSignalInfo())

                if(self.missionData.id != 0): #only perform auto-detection when explicitly required
                    if(self.autoDetect):
                        #Collect highest signal info during recording
                        if(self.recordTrigger):
                            self.runCmp.set()
                        
                        #Begin collecting frames before valid detection is confirmed
                        if(self.signalInfo.pre_activation and 
                           not self.signalInfo.detection and 
                           not self.recordTrigger and
                           (self.utils.calcEuclidDistance(prevPose, self.curPose) >= self.cfg['rest_dist'])):
                            rospy.loginfo('SC| Pre-Starting AUTO recording ...')
                            self.recordTrigger = True
                            has_updated_record_start_t = False
                            prevPose = self.curPose
                            self.past_sig_i:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)

                        if(self.signalInfo.detection):
                            if(not has_updated_record_start_t):
                                record_start_t = time.time()
                                rospy.loginfo('SC| Updated recording start time ...')
                                has_updated_record_start_t = True

                            if(self.recordTrigger and ((time.time()-record_start_t)) >= (self.curCaptureTime * self._f_mul)):
                                rospy.loginfo('SC| Saving AUTO recording [Autodetect| reason: Timeout] ...')
                                self.prepareMissionDirectory()
                                self._saveRecording(auto=True, start_t=record_start_t, 
                                                    wpInfo=ROSLayerUtils.WaypointInfo(0, *prevPose), 
                                                    sigInfo=SignalInfo(*self.past_sig_i))
                                prevPose = self.curPose
                        
                        if(not self.signalInfo.detection and self.recordTrigger):
                            rospy.loginfo('SC| Saving AUTO recording [Autodetect| reason: Deactivation] ...')
                            self.prepareMissionDirectory()
                            self._saveRecording(auto=True, start_t=record_start_t, 
                                                wpInfo=ROSLayerUtils.WaypointInfo(0, *prevPose), 
                                                sigInfo=SignalInfo(*self.past_sig_i))
                            prevPose = self.curPose                  

                self.publishDetection(self.signalInfo, self.camera.getBlobData())

                if((time.time() - alive_t) >= 1.0):
                    if(not self.camera.isAlive()):
                        rospy.logwarn_throttle(1.0, '[X] Camera disconnected!')
                        if(not disconnect_ev):
                            disconnect_t = time.time()
                            disconnect_ev = True
                            rospy.logwarn('Will wait 6.0 seconds for stream to resume ...')
                        else:
                            if((time.time() - disconnect_t) >= 6.0):
                                rospy.logwarn('Stream timeout!')
                                rospy.logwarn('Attempting to re-connect ...')
                                self._isStartup = True
                                disconnect_ev = False
                    else:
                        if(disconnect_ev):
                            disconnect_ev = False
                    alive_t = time.time()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('soundcam_node')
    rosObj = SoundcamROS()
    rospy.core.atexit.register(rosObj.disconnect)
    signal.signal(signal.SIGINT, rosObj.signal_handler)

    rosObj.run()