#! /usr/bin/env python3

import rospy, time, atexit, os, sys, math, signal
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
    Preset
from soundcam_ros.srv import SoundcamService, SoundcamServiceRequest, \
    SoundcamServiceResponse
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import KeyValue
import actionlib, threading, numpy as np
from cv_bridge import CvBridge
from soundcam_protocol import Features
from utils import ROSLayerUtils, MissionData, SignalInfo
from datetime import datetime
import cv2
import pyfakewebcam as pf

class SoundcamROS(object):
    def __init__(self) -> None:
        self.canRun = True
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
        # Stream recording parameters
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
        self.missionData = MissionData('unknown-none-nothing-nada', 0, 'unset')
        self.curPose = [0.0, 0.0, 90.0]
        self.signalInfo:SignalInfo = SignalInfo('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)

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
            
            thread_grp = list()
            if(self.cfg['processVideo']):
                self.vidbw_pub = rospy.Publisher(self.cfg['frame'] + '/video/bw/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidbw_pub, self.camera.getBWVideo, 
                                                SoundcamServiceRequest.VIDEO_STREAM, 
                                                self.devStr[0]], 
                                        daemon=True))
            if(self.cfg['processThermal']):
                self.vidthm_pub = rospy.Publisher(self.cfg['frame'] + '/video/thermal/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidthm_pub, self.camera.getTMVideo, 
                                                SoundcamServiceRequest.THERMAL_STREAM], 
                                        daemon=True))
            if(self.cfg['processAcVideo']):
                self.vidac_pub = rospy.Publisher(self.cfg['frame'] + '/video/acoustic/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                        args=[self.vidac_pub, self.camera.getACVideo, 
                                                SoundcamServiceRequest.ACOUSTIC_STREAM], 
                                        daemon=True))
            if(self.cfg['processAcVideo'] and self.cfg['processVideo']):
                self.vidoly_pub = rospy.Publisher(self.cfg['frame'] + '/video/overlay/compressed', CompressedImage, queue_size=15)
                thread_grp.append(threading.Thread(target=self.videoPublishingPostProc, 
                                        args=[self.vidoly_pub, self.camera.getBWVideo, self.camera.getACVideo, 
                                                SoundcamServiceRequest.OVERLAY_STREAM], 
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
            
            srvr = rospy.Service('SoundCameraServiceServer', SoundcamService, self.serviceCB)
            self.act_srvr = actionlib.SimpleActionServer("SoundCameraActionServer", SoundcamAction, 
                                                      execute_cb=self.executeCB, auto_start=False)
            self.act_feedbk = SoundcamFeedback()
            self.act_result = SoundcamResult()
            self.act_srvr.start()
            
            for th in thread_grp:
                th.start()
            rospy.loginfo("ROS Interfaces running ...")
        else:
            print("Using System Interfaces ...")
    
    def disconnect(self):
        self.canRun = False
        self.camera.disconnect()
        print('Exiting ...')
        raise SystemExit()

    def signal_handler(self, sig, frame):
        self.canRun = False
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
        pub.publish(msg)
        if(self.debug):
            rospy.loginfo_throttle(3, 'SC| Sent status msg')

    def videoPublishing(self, pub:rospy.Publisher, fn, streamType, devStrObj:pf.FakeWebcam=None):
        rate = rospy.Rate(15)
        while(not rospy.is_shutdown() and self.canRun):
            if(not self.pauseContinuous):
                img_arr:np.array = fn()
                #print(img_arr.shape)
                #print(img_arr)
                if(img_arr is not None):
                    # Publish to DEV
                    if(self.pubDevStream and devStrObj is not None):
                        processed_frame = self.utils.publishDevStream(img_arr)
                        devStrObj.schedule_frame(processed_frame)
                        # cv2.imshow('Processed Frame', processed_frame)
                        # if cv2.waitKey(1) & 0xFF == ord('q'):
                        #     break

                    # MANUAL stream recording
                    if(self.manualTrigger and (self.streamType == streamType)):
                        self._manual_frames_ls.append(img_arr)
                        self.utils.limitMemUsage(self._manual_frames_ls, self.cfg['max_megabytes'])
                    
                    # # AUTO stream recording
                    if(self.recordTrigger):
                        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                            self._auto_bw_frames_ls.append(img_arr)
                            self.utils.limitMemUsage(self._auto_bw_frames_ls, self.cfg['max_megabytes'])
                        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                            self._auto_tm_frames_ls.append(img_arr)
                            self.utils.limitMemUsage(self._auto_tm_frames_ls, self.cfg['max_megabytes'])

                    if(pub.get_num_connections() > 0): # topic publishing
                        # if(self.debug):
                        #     rospy.loginfo_once('SC| Video Streaming on Stream type -> {0}'.format(streamType))
                        self.convertPublishCompressedImage(pub=pub, cv_image=img_arr)
            rate.sleep()
    
    def videoPublishingPostProc(self, pub:rospy.Publisher, fn1, fn2, streamType):
        rate = rospy.Rate(15)
        p_img_arr1 = None
        p_img_arr2 = None
        while(not rospy.is_shutdown() and self.canRun):
            if(not self.pauseContinuous):
                img_arr1:np.array = fn1()
                if(img_arr1 is not None):
                    p_img_arr1 = img_arr1
                img_arr2:np.array = fn2()
                if(img_arr2 is not None):
                    p_img_arr2 = img_arr2
                
                if((p_img_arr1 is not None) and (p_img_arr2 is not None)):
                    #Post process
                    img_arr = self.utils.imageOverlay(p_img_arr1, p_img_arr2)
                    # MANUAL stream recording
                    if(self.manualTrigger and (self.streamType == streamType)):
                        self._manual_frames_ls.append(img_arr)
                        self.utils.limitMemUsage(self._manual_frames_ls, self.cfg['max_megabytes'])
                    
                    if(self.recordTrigger):
                        if(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                            self._auto_overlay_frames_ls.append(img_arr)
                            self.utils.limitMemUsage(self._auto_overlay_frames_ls, self.cfg['max_megabytes'])

                    if(pub.get_num_connections() > 0): # topic publishing
                        # if(self.debug):
                        #     rospy.loginfo_throttle(3, 
                        #             'SC| Video Streaming on Stream type -> {0}'.format(streamType))
                        self.convertPublishCompressedImage(pub=pub, cv_image=img_arr)
            rate.sleep()
    
    def audioPublishing(self, pub:rospy.Publisher, pubinfo:rospy.Publisher, fn, fninfo):
        rate = rospy.Rate(30)
        msg = AudioDataStamped()
        msg.header.frame_id = self.cfg['frame']
        msginfo = AudioInfo()
        start_t = rospy.Time.now().secs
        info_published = False
        while(not rospy.is_shutdown() and self.canRun):
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
        rate = rospy.Rate(20)
        msg = Spectrum()
        msg.header.frame_id = self.cfg['frame']
        while(not rospy.is_shutdown() and self.canRun):
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
    def publishDetection(self, sigInfo:SignalInfo, blobCoords:list):
        if(self.detection_pub.get_num_connections() > 0):
            msg = SoundcamDetection()
            msg.header.frame_id = self.cfg['frame']
            msg.header.stamp = rospy.Time.now()
            msg.preset = self.curPreset
            msg.meanEnergy.data = sigInfo.mean
            msg.stddev.data = sigInfo.std_dev
            msg.highEnergyThreshold.data = sigInfo.hi_thresh
            msg.currentEnergy.data = sigInfo.current
            msg.lowEnergyThreshold.data = sigInfo.lo_thresh
            msg.snr.data = sigInfo.snr
            msg.preActivation.data = sigInfo.pre_activation
            msg.detection.data = sigInfo.detection
            msg.blobXY = blobCoords
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
            orientation_eu = self.utils.quaternionToEulerDegrees(msg.orientation.w, 
                                                            msg.orientation.x,
                                                            msg.orientation.y,
                                                            msg.orientation.z)
            self.curPose = [msg.position.x, msg.position.y, orientation_eu[2]]
        except Exception as e:
            rospy.logerr(e)

        
    '''
    -------------------------SERVICE SERVER
    '''

    ''' Returns a single frame using the given function '''
    def _getFrame(self, fn):
        start_t = time.time()
        while(True):
            frame = fn()
            if(frame is not None):
                return frame
            if((time.time()-start_t) > 3.0):
                return None


    ''' Takes/Saves a snapshot of the given stream and returns True/False
        0 -> BW Image
        3 -> Thermal Image
        5 -> Overlayed Image
        6 -> [BW Image, Thermal Image, Overlayed Image]
    '''
    def _takeSnapshot(self, streamType=SoundcamServiceRequest.OVERLAY_STREAM, extras=None):
        isActPoint = True
        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
            frame = self._getFrame(self.camera.getBWVideo)
            if(frame is None):
                return False
        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
            frame = self._getFrame(self.camera.getTMVideo)
            if(frame is None):
                return False
        elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
            p_img_arr1 = self._getFrame(self.camera.getBWVideo)
            p_img_arr2 = self._getFrame(self.camera.getACVideo)
            frame = self.utils.imageOverlay(p_img_arr1, p_img_arr2)
            if(frame is None):
                return False
        elif(streamType == SoundcamServiceRequest.ALL):
            try:
                media = list()
                frame_data = list()
                filename = self.utils.getUniqueName(suffix='BW')
                frame = self._getFrame(self.camera.getBWVideo)
                if(frame is not None):
                    frame_data.append((frame, filename))

                filename = self.utils.getUniqueName(suffix='THM')
                frame = self._getFrame(self.camera.getTMVideo)
                if(frame is not None):
                    frame_data.append((frame, filename))

                filename = self.utils.getUniqueName(suffix='OV')
                p_img_arr1 = self._getFrame(self.camera.getBWVideo)
                p_img_arr2 = self._getFrame(self.camera.getACVideo)
                frame = self.utils.imageOverlay(p_img_arr1, p_img_arr2)
                if(frame is not None):
                    frame_data.append((frame, filename))
                
                for fobj in frame_data: #write snapshots
                    self.utils.createSnapshotFromFrame(fobj[0], filename=fobj[1])
                    media.append(fobj[1])
            except Exception as e:
                rospy.logerr('SC| Error taking snapshots [ALL]: ', e)
                return False
        
        if(streamType != SoundcamServiceRequest.ALL):
            try:
                if(extras is not None):
                    filename=self.utils.getUniqueName()
                    self.utils.createSnapshotFromFrame(frame, filename=filename)
                    media.append(filename)
                else:
                    self.utils.createSnapshotFromFrame(frame)
            except Exception as e:
                rospy.logerr('SC| Error taking BW/THM/OV snapshot: ', e)
                return False
        
        if((extras is not None) and (len(media) > 0)):
            if(not self.utils.addMetaData(media=media,isActionPoint=isActPoint, 
                                   id=extras[0], 
                                   info=(extras[1:4]), 
                                   sigInfo=extras[4],
                                   useMsnPath=True)):
                if(self.debug):
                    rospy.logerr_throttle(1, 'SC| Snapshot failure!')
                return False
        self.publishCaptureFeedback(self.capture_pub)
        if(self.debug):
            rospy.loginfo_throttle(1, 'SC| Snapshot success!')
        return True
    
    ''' Save recorded stream and returns True/False'''
    def _saveRecording(self, auto=False, start_t=time.time(), 
                       streamType=SoundcamServiceRequest.ALL,
                       isActPoint=False, info=None, id=None):
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
                rospy.logerr("SC| Error saving recording, ", e)
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
                    if(streamType == SoundcamServiceRequest.ALL):
                        filename = self.utils.getUniqueName(isImg=False, suffix='BW')
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, filename)
                        media.append(filename)

                        filename = self.utils.getUniqueName(isImg=False, suffix='THM')
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, filename)
                        media.append(filename)

                        filename = self.utils.getUniqueName(isImg=False, suffix='OV')
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                        filename = self.utils.getUniqueName(isImg=False, suffix='BW')
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                        filename = self.utils.getUniqueName(isImg=False, suffix='THM')
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                        filename = self.utils.getUniqueName(isImg=False, suffix='OV')
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, filename)
                        media.append(filename)
                        filename = 'AUD_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                        filename)
                        media.append(filename)
                    
                    if(len(media) > 0):
                        self.utils.addMetaData(isActionPoint=isActPoint, media=media, 
                                               info=info[:-1], sigInfo=info[-1], id=id, 
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
        if(req.command_type == SoundcamServiceRequest.CMD_TYPE_CONFIG):
            try:
                self.curCaptureTime = req.captureTime #if (req.captureTime > 0.0) else self.curCaptureTime
                if(req.mediaType != ''):
                    media = [int(x) for x in req.mediaType.split('|')]
                    if(SoundcamServiceRequest.ALL not in media):
                        self.curMediaType = media
                    else:
                        self.curMediaType = SoundcamServiceRequest.ALL
                # Get Extra Parameters
                if(len(req.extras) > 0):
                    for param in req.extras:
                        if(param.key == 'uuid'):
                            self.missionData.uuid = param.value
                        if(param.key == 'missionId'):
                            self.missionData.id = int(param.value)
                        if(param.key == 'missionName'):
                            self.missionData.name = param.value

                if(req.preset.hasPreset): #Configure camera preset
                    if(req.preset.presetName != self.curPreset.presetName):
                        # if(self.camera.isMeasuring()): #stop measurement if running
                        #     self.camera.stopMeasurement()
                        if(self.setPreset(req.preset)):
                            rospy.loginfo('Preset sent!')
                            # self.curPreset = req.preset
                            # self.camera.startMeasurement() #start measurement
                            # start_t = time.time()
                            # while(not self.camera.isDetectionReady()):
                            #     rospy.loginfo_throttle(3, "Awaiting Detection algo ...")
                            #     if((time.time() - start_t) >= 30.0):
                            #         rospy.logwarn("Camera stream taking longer to resume \
                            #                     \nCamera might be in Error!")
                            #         break
                        else:
                            rospy.logerr('Unable to send preset!')
                            if(not self.camera.isMeasuring()):
                                self.setPreset(self.curPreset)
                                self.camera.startMeasurement()
                            resp.results = SoundcamServiceResponse.FAIL
                    else:
                        if(self.debug):
                            rospy.loginfo("%s is already set!" % self.curPreset.presetName)
            except Exception as e:
                rospy.logerr("Error occured! ", e)
                resp.results = SoundcamServiceResponse.FAIL
        elif(req.command_type == SoundcamServiceRequest.CMD_TYPE_OP):
            try:
                if(req.command == SoundcamServiceRequest.RESET_CAMERA):
                    if(not self.camera.restartCamera()):
                        resp.results = SoundcamServiceResponse.FAIL
                elif((req.command == SoundcamServiceRequest.START_RECORD)):
                    dstrm = int(req.op_command1)
                    if(dstrm not in [SoundcamServiceRequest.VIDEO_STREAM,
                                 SoundcamServiceRequest.ACOUSTIC_STREAM,
                                 SoundcamServiceRequest.THERMAL_STREAM, 
                                 SoundcamServiceRequest.OVERLAY_STREAM]):
                        if(self.debug):
                            rospy.logerr('Stream type unknown: ', dstrm)
                        resp.results = SoundcamServiceResponse.FAIL
                    else:
                        self.streamType = dstrm
                        self.manualTrigger = True
                        if(self.debug):
                            rospy.loginfo('Recording video: Type -> %i' % dstrm)
                elif((req.command == SoundcamServiceRequest.STOP_RECORD)):
                    if(not self._saveRecording()):
                        resp.results = SoundcamServiceResponse.FAIL
                    if(self.debug):
                        rospy.loginfo('Recording saved!')
                elif((req.command == SoundcamServiceRequest.CANCEL_RECORD)):
                    self.manualTrigger = False
                    self._manual_frames_ls.clear()
                    if(self.debug):
                        rospy.loginfo('Recording cancelled!')
                elif(req.command == SoundcamServiceRequest.SNAPSHOT):
                    if(not self._takeSnapshot(int(req.op_command1))):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SET_DYNAMIC_CREST):
                    (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
                    dynamic = req.op_command1
                    crest = req.op_command2
                    if(not self.camera.setScalingMode(mode=mode, dynamic=dynamic, 
                                                      crest=crest, max=maximum)):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SET_SCALING_MODE):
                    (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
                    mode = int(req.op_command1)
                    if(mode == 0):
                        maximum = req.op_command2
                    else:
                        dynamic = req.op_command2
                        crest = req.op_command3
                    if(not self.camera.setScalingMode(mode=mode, dynamic=dynamic, 
                                        max=None, crest=crest)):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SCAN_DETECT):
                    self.autoDetect = True if (req.op_command2 == 1) else False
                else:
                    resp.results = SoundcamServiceResponse.FAIL
            except Exception as e:
                rospy.logerr("Error occured! ", e)
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

    from diagnostic_msgs.msg import KeyValue
    def executeCB(self, goal: SoundcamGoal):
        rospy.loginfo('SC| Received goal!')
        #save detection/recording parameters
        autoDetectVar = self.autoDetect
        self.autoDetect = False
        self.recordTrigger = False #deactivate record triggers
        #extract parameters
        streamType = SoundcamServiceRequest.ALL
        rospy.loginfo('Extracting goal parameters ...')
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
            if(param.key == 'waypointId'):
                wpId = int(param.value)
            if(param.key == 'waypointX'):
                wpX = float(param.value)
            if(param.key == 'waypointY'):
                wpY = float(param.value)
            if(param.key == 'waypointTheta'):
                wpTheta = float(param.value)
        
        #prepare directory
        self.prepareMissionDirectory()

        #run while loop
        rate = rospy.Rate(15)
        result = False
        cnt = 1
        past_sig_i:SignalInfo = SignalInfo('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        rospy.loginfo('Processing goal ...')
        while(not rospy.is_shutdown()):
            if((recordTime <= self.cfg['min_record_time']) and (numCaptures > 0)):
                #Take Snapshots
                if(self._takeSnapshot(streamType=streamType, 
                                   extras=(wpId, wpX, wpY, wpTheta, self.signalInfo))):
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
            elif((recordTime > self.cfg['min_record_time']) and (numCaptures > 0)):
                #Make Recordings
                if(not self.recordTrigger): #start recording
                    self.recordTrigger = True
                    record_start_t = time.time()
                else: #while recording
                    if((time.time()-record_start_t) >= recordTime): #save recording
                        rospy.loginfo('SC| Saving recording ...')
                        if(self._saveRecording(auto=True, isActPoint=True, 
                                            streamType=streamType,
                                            start_t=record_start_t, 
                                            info=(wpX, wpY, wpTheta, past_sig_i), id=wpId)):
                            cnt += 1
                            past_sig_i:SignalInfo = SignalInfo('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
                            time.sleep(delay)
                    else: # report progress
                        # Capture detection values
                        if((not past_sig_i.detection) and self.signalInfo.detection):
                            past_sig_i = self.signalInfo
                        if(self.signalInfo.mean > past_sig_i.mean):
                            past_sig_i._replace(mean=self.signalInfo.mean, snr=self.signalInfo.snr, 
                                                std_dev=self.signalInfo.std_dev)
                        if(self.signalInfo.hi_thresh > past_sig_i.hi_thresh):
                            past_sig_i._replace(hi_thresh=self.signalInfo.hi_thresh)
                        if((past_sig_i.lo_thresh > 0.0) and (self.signalInfo.lo_thresh < past_sig_i.lo_thresh)):
                            past_sig_i._replace(lo_thresh=self.signalInfo.lo_thresh)
                        
                        rospy.loginfo_throttle(3, 'SC| Recording [%i] in progress ...' % cnt)
                        self.act_feedbk.capture_count = cnt
                        self.act_feedbk.currentTime.data = rospy.Time.now()
                        self.act_srvr.publish_feedback(self.act_feedbk)
                    
                    if(cnt > numCaptures):
                        result = True
                        break
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
        self.autoDetect = autoDetectVar


    '''
        Ping, Connect & Configure Camera
    '''
    def attemptConnection(self):
        rate = rospy.Rate(1)
        while(not rospy.is_shutdown() and self.canRun):
            if(self.camera.pingCamera(deviceIP=self.cfg['ip'])):
                if((not self.camera.isConnected()) and self.camera.isAlive()):
                    if(self.camera.connect()):
                        rospy.loginfo('Connection established!')
                    else:
                        rospy.logwarn('Retrying to connect in 3 seconds ...')
                        rospy.sleep(3.0)
                        continue
                if(self.camera.isConnected()):
                    if(self.camera.hasStatus()):
                        rospy.loginfo('Initial Device status received...')
                        rospy.loginfo('Sending Initial configuration and preparing device ...')
                        self.camera.initialConfigure()
                        self.camera.startMeasurement()
                        break
                    else:
                        rospy.loginfo('Awaiting Initial Device status ...')
            else:
                rospy.loginfo('Unable to reach Soundcamera on IP:{0} \n \
                              Trying again after {1} seconds'.format(self.cfg['ip'], 3))
                time.sleep(3.0)
                continue
            rate.sleep()
        return self.camera.isConnected()

    def run(self):
        rate = rospy.Rate(40)
        has_updated_record_start_t = False
        record_start_t = time.time()
        prevPose = [0.0, 0.0, 0.0]
        firstrun = True
        past_sig_i:SignalInfo = SignalInfo('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        while(not rospy.is_shutdown()):
            if(self._isStartup):
                if(self.attemptConnection()):
                    if(firstrun):
                        self.bringUpInterfaces()
                        firstrun = False
                        rospy.loginfo_once('SC| Running ...')
                    else:
                        rospy.loginfo_once('SC| Restarted ...')
                    self._isStartup = False
                    
            else: 
                self.signalInfo:SignalInfo = self.camera.getSignalInfo(self.curPreset.presetName)
                #if(self.debug): 
                    #Mean Energy, Std Dev, High Energy Thresh, Current Energy, Low Energy Thresh, SNR, detection flag
                    #print(self.signalInfo)
                if(self.missionData.id != 0): #only perform auto-detection when explicitly required
                    if(self.autoDetect):
                        #Collect highest signal info during recording
                        if(self.recordTrigger):
                            if(self.signalInfo.mean > past_sig_i.mean):
                                past_sig_i = self.signalInfo
                        
                        #Begin collecting frames before valid detection is confirmed
                        if(self.signalInfo.pre_activation and 
                           not self.signalInfo.detection and 
                           not self.recordTrigger and
                           (self.utils.calcEuclidDistance(prevPose, self.curPose) >= self.cfg['rest_dist'])):
                            rospy.loginfo('SC| Pre-Starting AUTO recording ...')
                            self.recordTrigger = True
                            has_updated_record_start_t = False
                            prevPose = self.curPose
                            past_sig_i:SignalInfo = SignalInfo('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)

                        if(self.signalInfo.detection):
                            if(not has_updated_record_start_t):
                                record_start_t = time.time()
                                rospy.loginfo('SC| Updated recording start time ...')
                                has_updated_record_start_t = True

                            if(self.recordTrigger and ((time.time()-record_start_t)) >= (self.curCaptureTime * self._f_mul)):
                                rospy.loginfo('SC| Saving AUTO recording [Autodetect| Timeout] ...')
                                self.prepareMissionDirectory()
                                self._saveRecording(auto=True, start_t=record_start_t, info=(*self.curPose, past_sig_i))
                                prevPose = self.curPose
                        
                        if(not self.signalInfo.detection and self.recordTrigger):
                            rospy.loginfo('SC| Saving AUTO recording [Autodetect| Deactivation] ...')
                            self.prepareMissionDirectory()
                            self._saveRecording(auto=True, start_t=record_start_t, info=(*self.curPose, past_sig_i))
                            prevPose = self.curPose                  

            self.publishDetection(self.signalInfo, self.camera.getBlobData())
            self.publishPreset()
            if(not self.camera.isAlive()):
                rospy.logwarn('Camera disconnected!')
                rospy.loginfo('Reconnection attempt in 2 seconds ...')
                rospy.sleep(2.0)
                self._isStartup = True
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('soundcam_node')
    rosObj = SoundcamROS()
    #rospy.core.atexit.register(rosObj.release_shared)
    rospy.core.atexit.register(rosObj.disconnect)
    signal.signal(signal.SIGINT, rosObj.signal_handler)

    rosObj.run()