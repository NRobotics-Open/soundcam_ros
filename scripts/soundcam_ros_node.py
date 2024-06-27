#! /usr/bin/env python3

import rospy, time, atexit, os, sys, math
from config import cfgContext
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
from utils import ROSLayerUtils
from datetime import datetime
import cv2

class SoundcamROS(object):
    def __init__(self) -> None:
        self.canRun = True
        self.cfg = cfgContext
        self.camera = SoundCamConnector(debug=False, cfgObj=self.cfg)
        self.utils = ROSLayerUtils()
        
        #operational parameters
        self.curPreset = Preset(scalingMode=self.cfg['scalingMode'],
                                crest=self.cfg['crest'],
                                distance=self.cfg['distance'],
                                maximum=self.cfg['maximum'],
                                dynamic=self.cfg['dynamic'],
                                maxFrequency=self.cfg['maxFrequency'],
                                minFrequency=self.cfg['minFrequency'])
        self.curMediaType = [int(x) for x in '0|1|2|3|4|5'.split('|')] #all
        self.curCaptureTime = self.cfg['max_record_time'] #how long to record onAutoDetection
        self.autoDetect = False
        self._isStartup = True
        # Stream recording parameters
        self._manual_frames_ls = list()
        self._auto_bw_frames_ls = list()
        self._auto_tm_frames_ls = list()
        self._auto_overlay_frames_ls = list()
        self._auto_audio_frames_ls = list()
        self.streamType = SoundcamServiceRequest.OVERLAY_STREAM
        self.manualTrigger = False
        self.recordTrigger = False
        self.pauseContinuous = False

    def bringUpInterfaces(self):
        rospy.loginfo('Bringing up interfaces ...')
        if(not cfgContext['system_run']): #start publishers, service servers and action servers
            self.status_pub = rospy.Publisher(self.cfg['frame'] + '/status', SoundcamStatus, queue_size=3, latch=True)
            self.statusPublish(self.status_pub, self.camera.getStatus)
            self.bridge = CvBridge()
            self.detection_pub = rospy.Publisher(self.cfg['frame'] + '/detection', SoundcamDetection, queue_size=5)
            self.capture_pub = rospy.Publisher(self.cfg['capture_feedback_topic'], Bool, queue_size=1)
            
            thread_grp = list()
            self.vidbw_pub = rospy.Publisher(self.cfg['frame'] + '/video/bw/compressed', CompressedImage, queue_size=15)
            thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                      args=[self.vidbw_pub, self.camera.getBWVideo, 
                                            SoundcamServiceRequest.VIDEO_STREAM], 
                                      daemon=True))
            
            self.vidthm_pub = rospy.Publisher(self.cfg['frame'] + '/video/thermal/compressed', CompressedImage, queue_size=15)
            thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                      args=[self.vidthm_pub, self.camera.getTMVideo, 
                                            SoundcamServiceRequest.THERMAL_STREAM], 
                                      daemon=True))

            self.vidac_pub = rospy.Publisher(self.cfg['frame'] + '/video/acoustic/compressed', CompressedImage, queue_size=15)
            thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                      args=[self.vidac_pub, self.camera.getACVideo, 
                                            SoundcamServiceRequest.ACOUSTIC_STREAM], 
                                      daemon=True))
            
            self.vidoly_pub = rospy.Publisher(self.cfg['frame'] + '/video/overlay/compressed', CompressedImage, queue_size=15)
            thread_grp.append(threading.Thread(target=self.videoPublishing, 
                                      args=[self.vidoly_pub, self.camera.getOverlayVideo, 
                                            SoundcamServiceRequest.OVERLAY_STREAM], 
                                      daemon=True))
            
            self.audio_info_pub = rospy.Publisher(self.cfg['frame'] + '/audio/info', AudioInfo, queue_size=3, latch=True)
            self.audio_pub = rospy.Publisher(self.cfg['frame'] + '/audio', AudioDataStamped, queue_size=15)
            thread_grp.append(threading.Thread(target=self.audioPublishing, 
                                      args=[self.audio_pub, self.audio_info_pub, 
                                            self.camera.getAudio, self.camera.getAudioInfo], 
                                      daemon=True))
            
            self.spectrum_pub = rospy.Publisher(self.cfg['frame'] + '/spectrum', Spectrum, queue_size=15)
            thread_grp.append(threading.Thread(target=self.spectrumPublishing, 
                                      args=[self.spectrum_pub, self.camera.getSpectrum], 
                                      daemon=True))
            
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

    def release_shared(self):
        self.camera.release_shared()
    
    def disconnect(self):
        self.canRun = False
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

    def videoPublishing(self, pub:rospy.Publisher, fn, streamType):
        rate = rospy.Rate(50)
        while(not rospy.is_shutdown()):
            if(not self.pauseContinuous):
                img_arr:np.array = fn()
                #print(img_arr.shape)
                #print(img_arr)
                if(img_arr is not None):
                    # MANUAL stream recording
                    if(self.manualTrigger and (self.streamType == streamType)):
                        self._manual_frames_ls.append(img_arr)
                        self.utils.limitMemUsage(self._manual_frames_ls, self.cfg['max_megabytes'])
                    
                    if(self.recordTrigger):
                        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                            self._auto_bw_frames_ls.append(img_arr)
                        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                            self._auto_tm_frames_ls.append(img_arr)
                        elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                            self._auto_overlay_frames_ls.append(img_arr)

                    if(pub.get_num_connections() > 0): # topic publishing
                        # if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                        #     img = self.bridge.cv2_to_imgmsg(img_arr, "mono8")
                        # else:
                        #     img = self.bridge.cv2_to_imgmsg(img_arr, "bgr8")
                        self.convertPublishCompressedImage(pub=pub, cv_image=img_arr)
            rate.sleep()
    
    def audioPublishing(self, pub:rospy.Publisher, pubinfo:rospy.Publisher, fn, fninfo):
        rate = rospy.Rate(80)
        msg = AudioDataStamped()
        msg.header.frame_id = self.cfg['frame']
        msginfo = AudioInfo()
        start_t = rospy.Time.now()
        info_published = False
        while(not rospy.is_shutdown()):
            aud_arr:np.array = fn()
            if(self.recordTrigger):
                self._auto_audio_frames_ls.append(aud_arr)
            if(pub.get_num_connections() > 0):
                msg.header.stamp = rospy.Time.now()
                if(aud_arr):
                    msg.audio.data = aud_arr
                    pub.publish(msg)
            if((pubinfo.get_num_connections() > 0) and 
               ((rospy.Time.now() - start_t) > 1.0) and 
               not info_published): # publish audio info
                meta = fninfo()
                if(meta):
                    msginfo.channels = meta['channels']
                    msginfo.sample_rate = meta['sample_rate']
                    msginfo.bitrate = meta['bitrate']
                    pubinfo.publish(msginfo)
                    start_t = rospy.Time.now()
                    info_published = True
            rate.sleep()

    def spectrumPublishing(self, pub:rospy.Publisher, fn):
        rate = rospy.Rate(40)
        msg = Spectrum()
        msg.header.frame_id = self.cfg['frame']
        while(not rospy.is_shutdown()):
            if(pub.get_num_connections() > 0):
                msg.header.stamp = rospy.Time.now()
                (msg.frequency, msg.amplitude) = fn()
                pub.publish(msg)
            rate.sleep()
    
    ''' Sends alert to other nodes about current detections '''
    def publishDetection(self, val, blobCoords:list):
        msg = SoundcamDetection()
        msg.header.frame_id = self.cfg['frame']
        msg.header.stamp = rospy.Time.now()
        msg.preset = self.curPreset
        msg.detector = val
        msg.blobXY = blobCoords
        self.detection_pub.publish(msg)

    def publishCaptureFeedback(self, pub:rospy.Publisher):
        pub.publish(Bool(True))
    
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
        return self.camera.updatePreset(mode=preset.scalingMode,
                                        distance=preset.distance,
                                        minFreq=preset.minFrequency,
                                        maxFreq=preset.maxFrequency,
                                        dynamic=preset.dynamic,
                                        crest=preset.crest,
                                        maximum=preset.maximum)
                
    def getRobotPose(self):
        try:
            msg:Pose = rospy.wait_for_message(self.cfg['pose_topic'], Pose, rospy.Duration(1.0))
            _, _, theta = self.utils.quaternionToEulerDegrees(msg.orientation.w, 
                                                            msg.orientation.x,
                                                            msg.orientation.y,
                                                            msg.orientation.z)
            return (msg.position.x, msg.position.y, theta)
        except Exception as e:
            return (0.0, 0.0, 90.0)
        
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
        if(streamType == SoundcamServiceRequest.VIDEO_STREAM):
            frame = self._getFrame(self.camera.getBWVideo)
            if(frame is None):
                return False
            try:
                self.utils.createSnapshotFromFrame(frame)
            except Exception as e:
                rospy.logerr('SC| Error taking BW snapshot: ', e)
                return False
        elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
            frame = self._getFrame(self.camera.getTMVideo)
            if(frame is None):
                return False
            try:
                self.utils.createSnapshotFromFrame(frame)
            except Exception as e:
                rospy.logerr('SC| Error taking TM snapshot: ', e)
                return False
        elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
            frame = self._getFrame(self.camera.getOverlayVideo)
            if(frame is None):
                return False
            try:
                self.utils.createSnapshotFromFrame(frame)
            except Exception as e:
                rospy.logerr('SC| Error taking OV snapshot: ', e)
                return False
        elif(streamType == SoundcamServiceRequest.ALL):
            try:
                media = list()
                timestamp = datetime.now().strftime("%H_%m_%S")
                filename = 'VIDEO_' + timestamp + '.jpg'
                frame = self._getFrame(self.camera.getBWVideo)
                if(frame is not None):
                    self.utils.createSnapshotFromFrame(frame, filename=filename)
                    media.append(filename)

                filename = 'THERMAL_' + timestamp + '.jpg'
                frame = self._getFrame(self.camera.getTMVideo)
                if(frame is not None):
                    self.utils.createSnapshotFromFrame(frame, filename=filename)
                    media.append(filename)

                filename = 'OVERLAY_' + timestamp + '.jpg'
                frame = self._getFrame(self.camera.getOverlayVideo)
                if(frame is not None):
                    self.utils.createSnapshotFromFrame(frame, filename=filename)
                    media.append(filename)

                if((extras is not None) and (len(media) > 0)):
                    self.utils.addMetaData(media=media, pose=(extras[1:]), id=extras[0])
            except Exception as e:
                rospy.logerr('SC| Error taking snapshots: ', e)
                return False
        self.publishCaptureFeedback(self.capture_pub)
        return True
    
    ''' Save recorded stream and returns True/False'''
    def _saveRecording(self, auto=False, start_t=time.time(), 
                       streamType=SoundcamServiceRequest.ALL,
                       isActPoint=False, pose_info=None, id=None):
        if(not auto): # manual recording save
            try:
                self.manualTrigger = False
                self.utils.createVideoFromFrames(self._manual_frames_ls)
                self.publishCaptureFeedback(self.capture_pub)
                return True
            except Exception as e:
                rospy.logerr("SC| Error saving recording, ", e)
                return False
        else: # auto recording save
            self.recordTrigger = False
            try:
                if((time.time() - start_t) >= self.cfg['min_record_time']):
                    timestamp = datetime.now().strftime("%H_%m_%S")
                    media = list()
                    if(streamType == SoundcamServiceRequest.ALL):
                        filename = 'VIDEO_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'THERMAL_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'OVERLAY_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'AUDIO_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.VIDEO_STREAM):
                        filename = 'VIDEO_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_bw_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'AUDIO_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.THERMAL_STREAM):
                        filename = 'THERMAL_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_tm_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'AUDIO_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)

                    elif(streamType == SoundcamServiceRequest.OVERLAY_STREAM):
                        filename = 'OVERLAY_' + timestamp + '.mp4'
                        self.utils.createVideoFromFrames(self._auto_overlay_frames_ls, 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                        filename = 'AUDIO_' + timestamp + '.wav'
                        self.utils.createAudioFromFrames(self._auto_audio_frames_ls, 
                                                        self.camera.getAudioInfo()['sample_rate'], 
                                                    os.path.join(self.utils.getPath(), filename))
                        media.append(filename)
                    
                    if(len(media) > 0):
                        self.utils.addMetaData(isActionPoint=isActPoint, media=media, pose=pose_info, id=id)
                        self.publishCaptureFeedback(self.capture_pub)
                else:
                    rospy.loginfo('SC| Recording time less the minimum specified. Discarding ...')
                    self._auto_overlay_frames_ls.clear()
                    self._auto_bw_frames_ls.clear()
                    self._auto_tm_frames_ls.clear()
                    self._auto_audio_frames_ls.clear()
            except Exception as e:
                self._auto_overlay_frames_ls.clear()
                self._auto_bw_frames_ls.clear()
                self._auto_tm_frames_ls.clear()
                self._auto_audio_frames_ls.clear()
                rospy.logerr('SC| Error AUTOMATICALLY saving streams: ', e)
                return False
            return True

    
    def serviceCB(self, req:SoundcamServiceRequest):
        resp = SoundcamServiceResponse()
        resp.results = SoundcamServiceResponse.SUCCESS
        if(req.command_type == SoundcamServiceRequest.CMD_TYPE_CONFIG):
            try:
                self.curCaptureTime = req.captureTime if (req.captureTime > 0.0) else self.curCaptureTime
                media = [int(x) for x in req.mediaType.split('|')]
                if(SoundcamServiceRequest.ALL not in media):
                    self.curMediaType = media
                else:
                    self.curMediaType = SoundcamServiceRequest.ALL
                # Get Mission Parameters
                dt:KeyValue = req.extras[0]
                self.utils.prepareDirectory(int(dt.key), dt.value)
                #Configure camera preset
                if(self.camera.isMeasuring()): #stop measurement if running
                    self.camera.stopMeasurement()
                if(self.setPreset(req.preset)):
                    rospy.loginfo('Preset sent!')
                    self.curPreset = req.preset
                    self.camera.startMeasurement() #start measurement
                else:
                    rospy.logerr('Unable to send preset!')
                    if(not self.camera.isMeasuring()):
                        self.setPreset(self.curPreset)
                        self.camera.startMeasurement()
                    resp.results = SoundcamServiceResponse.FAIL
            except Exception as e:
                rospy.logerr("Error occured! ", e)
                resp.results = SoundcamServiceResponse.FAIL
        elif(req.command_type == SoundcamServiceRequest.CMD_TYPE_OP):
            try:
                if(req.command == SoundcamServiceRequest.RESET_CAMERA):
                    self._isStartup = self.camera.restartCamera()
                elif((req.command == SoundcamServiceRequest.START_RECORD)):
                    dstrm = int(req.op_command1)
                    if(dstrm not in [SoundcamServiceRequest.VIDEO_STREAM,
                                 SoundcamServiceRequest.ACOUSTIC_STREAM,
                                 SoundcamServiceRequest.THERMAL_STREAM, 
                                 SoundcamServiceRequest.OVERLAY_STREAM]):
                        print('Stream type unknown: ', dstrm)
                        resp.results = SoundcamServiceResponse.FAIL
                    else:
                        self.streamType = dstrm
                        self.manualTrigger = True
                        rospy.loginfo('Recording video: Type -> %i' % dstrm)
                elif((req.command == SoundcamServiceRequest.STOP_RECORD)):
                    if(not self._saveRecording()):
                        resp.results = SoundcamServiceResponse.FAIL
                    rospy.loginfo('Recording saved!')
                elif((req.command == SoundcamServiceRequest.CANCEL_RECORD)):
                    self.manualTrigger = False
                    self._manual_frames_ls.clear()
                    rospy.loginfo('Recording cancelled!')
                elif(req.command == SoundcamServiceRequest.SNAPSHOT):
                    if(not self._takeSnapshot(int(req.op_command1))):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SET_DYNAMIC_CREST):
                    (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
                    dynamic = req.op_command1
                    crest = req.op_command2
                    if(not self.camera.setScalingMode()):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SET_SCALING_MODE):
                    (mode, dynamic, maximum, crest) = self.camera.getScalingMode()
                    mode = req.op_command1
                    if(mode == 0):
                        maximum = req.op_command1
                    else:
                        dynamic = req.op_command1
                        crest = req.op_command2
                    if(not self.camera.setScalingMode(mode=mode, dynamic=dynamic, 
                                        max=maximum, crest=crest)):
                        resp.results = SoundcamServiceResponse.FAIL
                elif(req.command == SoundcamServiceRequest.SCAN_DETECT):
                    self.autoDetect = True if (req.op_command1 == 1) else False
                else:
                    resp.results = SoundcamServiceResponse.FAIL
            except Exception as e:
                rospy.logerr("Error occured! ", e)
                resp.results = SoundcamServiceResponse.FAIL

        resp.ack.data = True
        return resp
    
    '''
    -------------------------ACTION SERVER
    '''
    
    def executeCB(self, goal: SoundcamGoal):
        rospy.loginfo('SC| Received goal!')
        #save detection/recording parameters
        autoDetectVar = self.autoDetect
        self.autoDetect = False
        self.recordTrigger = False #deactivate record triggers
        #extract parameters
        delay = int(goal.parameters['delay'])
        numCaptures = int(goal.parameters['numCaptures'])
        recordTime = float(goal.parameters['recordTime'])
        media = [int(x) for x in goal.parameters['streamType'].split('|')]
        if(SoundcamServiceRequest.ALL not in media):
            streamType = media
        else:
            streamType = SoundcamServiceRequest.ALL
        msnId = int(goal.parameters['msnId'])
        msnName = goal.parameters['msnName']
        wpId = int(goal.parameters['wpId'])
        wpX = float(goal.parameters['wpX'])
        wpY = float(goal.parameters['wpY'])
        wpTheta = float(goal.parameters['wpTheta'])
        #prepare directory
        if(not str(msnId) in self.utils.getPath()):
            self.utils.prepareDirectory(msnId, msnName)

        #run while loop
        rate = rospy.Rate(30)
        result = False
        cnt = 0
        record_start_t = time.time()
        while(not rospy.is_shutdown()):
            if((recordTime <= self.cfg['min_record_time']) and (numCaptures > 0)):
                #Take Snapshots
                if(self._takeSnapshot(streamType=streamType, 
                                   extras=(wpId, wpX, wpY, wpTheta))):
                    cnt += 1
                    self.act_feedbk.capture_count = cnt
                    self.act_feedbk.currentTime.data = rospy.Time.now()
                    self.act_srvr.publish_feedback(self.act_feedbk)
                    time.sleep(delay)
                else:
                    rospy.logerr('SC| Snapshot failed!')
                    break

                if(cnt >= numCaptures):
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
                                            pose_info=(wpX, wpY, wpTheta), id=wpId)):
                            cnt += 1
                            time.sleep(delay)
                    else:
                        rospy.loginfo('SC| Recording in progress ...')
                    
                    if(cnt >= numCaptures):
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
        self.restPeriod_t = time.time()
        self.autoDetect = autoDetectVar


    '''
        Ping, Connect & Configure Camera
    '''
    def attemptConnection(self):
        rate = rospy.Rate(1)
        while(not rospy.is_shutdown() and self.canRun):
            if(self.camera.pingCamera(deviceIP=self.cfg['ip'])):
                if(not self.camera.isConnected()):
                    if(self.camera.connect()):
                        rospy.loginfo('Connection established!')
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
        record_start_t = time.time()
        self.restPeriod_t = time.time()
        while(not rospy.is_shutdown()):
            if(self._isStartup):
                if(self.attemptConnection()):
                    self.bringUpInterfaces()
                    self._isStartup = False
                    rospy.loginfo_once('SC| Running ...')
            else: 
                if(self.autoDetect and ((time.time()-self.restPeriod_t) >= self.cfg['rest_time'])):
                    if(self.camera.hasDetection() and not self.recordTrigger):
                        rospy.loginfo('SC| Starting AUTO recording ...')
                        self.recordTrigger = True
                        self.robotPose = self.getRobotPose()
                        record_start_t = time.time()

                    if(self.camera.hasDetection() and ((time.time()-record_start_t)) >= self.curCaptureTime):
                        rospy.loginfo('SC| Saving AUTO recording [1] ...')
                        self._saveRecording(auto=True, start_t=record_start_t, pose_info=self.robotPose)
                        self.restPeriod_t = time.time()
                    
                    if(not self.camera.hasDetection() and self.recordTrigger):
                        rospy.loginfo('SC| Saving AUTO recording [2] ...')
                        self._saveRecording(auto=True, start_t=record_start_t, pose_info=self.robotPose)
                        self.restPeriod_t = time.time() 

            self.publishDetection(self.camera.hasDetection(), self.camera.getBlobData())
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('soundcam_node')
    rosObj = SoundcamROS()
    rospy.core.atexit.register(rosObj.release_shared)
    rospy.core.atexit.register(rosObj.disconnect)

    rosObj.run()