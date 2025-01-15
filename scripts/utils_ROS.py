'''
-----------------------------------------UTILITY METHODS FOR THE ROS LAYER
'''

from datetime import datetime
from dataclasses import dataclass
import yaml
import cv2, os, math
from typing import NamedTuple, List
import numpy as np
import soundfile as sf
from utils import SignalInfo
from soundcam_ros.msg import Preset

@dataclass
class MissionData:
    uuid: str
    id: int
    name: str
    result_dir: str

class ROSLayerUtils(object):
    PoseInfo = NamedTuple('PoseInfo', [('x', float), ('y', float), ('theta', float)])
    WaypointInfo = NamedTuple('WaypointInfo', [('id', int), ('x', float), ('y', float), ('theta', float)])
    DataPoint = NamedTuple('DataPoint', [('id', int), ('x', float), ('y', float), ('theta', float), 
                                         ('media', List),  
                                         ('mean_energy', float), ('std_dev', float),
                                         ('hi_thresh', float), ('current_energy', float), 
                                         ('lo_thresh', float), ('acoustic_energy', float), 
                                         ('snr', float), ('pre_activation', bool), 
                                         ('detection', bool),
                                         ('isSolved', bool), ('relevant_image', int),
                                         ('leak_rate', float),
                                         ('presetName', str), ('maximumFrequency', int), ('minimumFrequency', int),
                                         ('distance', float), ('crest', float), ('dynamic', float), ('maximum', float)])
    TileInfo = NamedTuple('TileInfo', [('id', int), ('relId', int)])
    def __init__(self, debug=False) -> None:
        self.mediaDir = os.path.expanduser("~") + '/current'
        if(not os.path.exists(self.mediaDir)):
            os.makedirs(self.mediaDir)
        self.msnDir = os.path.expanduser("~") + '/missionresults'
        if(not os.path.exists(self.msnDir)):
            os.makedirs(self.msnDir)
        
        self.curImg = 'current.jpg'
        self.curVid = 'current.mp4'
        self.curAud = 'current.wav'
        self.missionID = None
        self.missionName = None
        self.path = None
        self.localId = 1
        self.debug = debug

    def prepareDirectory(self, id, name, path=None):
        self.missionID = id
        self.missionName = name
        if((path is not None) and (len(path) > 0) and os.path.exists(path)):
            self.path = path
            self.localId = 1 #reset internal id
        else:
            today = datetime.now().strftime("%Y_%m_%d")
            msn_time = datetime.now().strftime("%H_%M")
            self.path = os.path.join(self.msnDir, id, today, msn_time)
            if(not os.path.exists(self.path)): 
                os.makedirs(self.path)
        self.localId = 1 #reset internal id
    
    def getUniqueName(self, isImg=True, suffix=''):
        today = datetime.now().strftime("%Y_%m_%d_")
        msn_time = datetime.now().strftime("%H_%M_%S")
        if(isImg):
            return ''.join(['IMG_', suffix, '_', today, msn_time, '.jpg'])
        else:
            return ''.join(['VID_', suffix, '_', today, msn_time, '.webm'])
    
    def getPath(self, fetchMsnDir=False):
        if(fetchMsnDir):
            return self.path
        else:
            return self.mediaDir
    
    def convert_numpy_types(self, data):
        if isinstance(data, dict):
            return {key: self.convert_numpy_types(value) for key, value in data.items()}
        elif isinstance(data, list):
            return [self.convert_numpy_types(item) for item in data]
        elif isinstance(data, np.ndarray):
            return data.tolist()
        elif isinstance(data, (np.int_, np.intc, np.intp, np.int8, np.int16, np.int32, np.int64)):
            return int(data)
        elif isinstance(data, (np.float_, np.float16, np.float32, np.float64)):
            return float(data)
        elif isinstance(data, (np.bool_)):
            return bool(data)
        else:
            return data
    
    def addMetaData(self, wpInfo:WaypointInfo, media:list, sigInfo:SignalInfo, isActionPoint=False, 
                    preset:Preset=None, loop=1, relevantIdx:int=0, leakRate:float=0.0, useMsnPath=False):
        try:
            if(wpInfo.id == 0):
                wpInfo._replace(id=self.localId)
            preset_dt = ('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if(preset is not None):
                preset_dt = (preset.presetName, preset.maxFrequency, preset.minFrequency,
                             preset.distance, preset.crest, preset.dynamic, preset.maximum)
            obj:ROSLayerUtils.DataPoint = ROSLayerUtils.DataPoint(
                                                *wpInfo, 
                                                media,
                                                *sigInfo,
                                                False, int(relevantIdx), float(leakRate), 
                                                *preset_dt)
            obj = self.convert_numpy_types(obj._asdict())
            path = self.getPath(fetchMsnDir=useMsnPath)
            loop = str(loop)
            #print('Current Loop is: ', loop)
            if(os.path.exists(os.path.join(path, 'meta-data.yaml'))): #read meta data file
                with open(os.path.join(path, 'meta-data.yaml') , 'r') as infofile:
                    self.metaData = yaml.safe_load(infofile)
                    print('MetaData: ', self.metaData)
                    #check by the current loop
                    if((self.metaData is not None) and (loop not in self.metaData.keys())):
                        self.metaData[loop] = {'datapoints':[], 'actionpoints':[]}
            else:
                self.metaData = dict({loop: {'datapoints':[], 'actionpoints':[]}})

            def updateDict(existing_obj, cur_obj):
                existing_obj['acoustic_energy'] = cur_obj['acoustic_energy']
                existing_obj['current_energy'] = cur_obj['current_energy']
                existing_obj['detection'] = cur_obj['detection']
                existing_obj['mean_energy'] = cur_obj['mean_energy']
                existing_obj['relevant_image'] = cur_obj['relevant_image']
                existing_obj['snr'] = cur_obj['snr']
                existing_obj['std_dev'] = cur_obj['std_dev']
                existing_obj['leak_rate'] = cur_obj['leak_rate']
                return existing_obj
        
            hasId = False
            if(isActionPoint):
                for obj_old in self.metaData[loop]['actionpoints']: #check if actionpoint in metadata
                    if(obj_old['id'] == wpInfo.id):
                        hasId = True
                        for dt in obj['media']:
                            obj_old['media'].append(dt)
                            # update signal Parameters
                            obj_old = updateDict(existing_obj=obj_old, cur_obj=obj)
                        break
                if(not hasId):
                    self.metaData[loop]['actionpoints'].append(obj)
            else:
                for obj_old in self.metaData[loop]['datapoints']: #check if datapoint in metadata
                    #print('Existing content: ', obj_old)
                    if(obj_old['id'] == wpInfo.id):
                        hasId = True
                        for dt in obj['media']:
                            obj_old['media'].append(dt)
                        # update signal Parameters
                        obj_old = updateDict(existing_obj=obj_old, cur_obj=obj)
                        break
                if(not hasId):
                    self.metaData[loop]['datapoints'].append(obj)
                self.localId += 1

            with open(os.path.join(path, 'meta-data.yaml') , 'w') as infofile: #write meta data file
                yaml.dump(self.metaData, infofile)
            return True
        except Exception as e:
            print('Exception caught! ', e)
            return False

    def eulerDistance(self, coords:list):
        return math.sqrt(math.pow((coords[0] - 320), 2) + 
                         math.pow((coords[1] - 240), 2))
    
    def quaternionToEulerDegrees(self, w, x, y, z):
        """
            Convert a quaternion into Euler angles (roll, pitch, yaw) in degrees.

            Parameters:
            w, x, y, z : float
                Quaternion components.

            Returns:
            tuple
                Tuple containing roll, pitch, and yaw in degrees.
        """
        # Convert quaternion to Euler angles in radians
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, +1.0)
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        # Convert radians to degrees
        return np.degrees([roll_x, pitch_y, yaw_z]).tolist()
    
    def calcEuclidDistance(self, pose_a:list, pose_b:list):
        return math.sqrt((math.pow((pose_a[0] - pose_b[0]), 2)) + 
                         (math.pow((pose_a[1] - pose_b[1]), 2)))

    def _calculateMemUsage(self, frame_list:list):
        # Calculate the total memory usage of the frames in Megabytes
        total_bytes = sum(frame.nbytes for frame in frame_list)
        total_megabytes = total_bytes / (1024 * 1024)
        return total_megabytes
    
    def limitMemUsage(self, frame_list:list, max_megabytes):
        # Limit the memory usage of the frames to a given size in Megabytes
        while (self._calculateMemUsage(frame_list) > max_megabytes):
            frame_list.pop(0)  # Remove the oldest frame
    
    def createSnapshotFromFrame(self, frame, filename=None):
        if(frame is None):
            return
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curImg)
        else:
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        if(self.debug):
            print('Saving snaphsot to: ', save_to)
        try:
            cv2.imwrite(save_to, frame)
        except Exception as e:
            print('SC| Error creating snapshot: ', e)
    
    def createVideoFromFrames(self, frame_list:list, filename=None, fps=25):
        if not frame_list:
            raise ValueError("Frame list is empty. Cannot create video.")
        print('Saving for file: ', filename, ' frame length: ', len(frame_list))
        # Get the shape of the frames
        layers = None
        try:
            height, width = frame_list[0].shape
        except Exception as e:
            height, width, layers = frame_list[0].shape
        frame_size = (width, height)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'vp80')  # You can use other codecs like 'XVID'
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curVid)
        else:
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        
        if('THM' in filename):
            fps = 9
        out = cv2.VideoWriter(save_to, fourcc, fps, frame_size)
        
        for frame in frame_list:
            #print('Shape: ', frame.shape)
            if(layers is None):
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif (layers == 4):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            out.write(frame)
        # Release everything if job is finished
        out.release()
        frame_list.clear()
    
    '''
    Publishes the frame to the given devstream object
    '''
    def publishDevStream(self, frame):
        # Get the shape of the frames
        layers = None
        try:
            _, _ = frame.shape
        except Exception as e:
            _, _, layers = frame.shape

        # Define the codec and push to VideoWriter object
        if(layers is None):
            proc_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        elif (layers == 4):
            proc_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        #strmObj.write(proc_frame)
        return proc_frame

    def createAudioFromFrames(self, audio_frames:list, samplerate, filename=None):
        # Concatenate the list of numpy arrays into a single numpy array
        audio_data = np.concatenate(audio_frames)
        # Save the concatenated audio data to a file
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curAud)
        else:
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        sf.write(save_to, audio_data, samplerate)
    

    def imageOverlay(self, bkg_img: np.array, fg_img: np.array) -> np.array:
        # Ensure both images are in BGRA format
        if len(bkg_img.shape) == 2:
            bkg_img = cv2.cvtColor(bkg_img, cv2.COLOR_GRAY2BGRA)
        elif bkg_img.shape[2] == 3:
            bkg_img = cv2.cvtColor(bkg_img, cv2.COLOR_BGR2BGRA)

        if len(fg_img.shape) == 2:
            fg_img = cv2.cvtColor(fg_img, cv2.COLOR_GRAY2BGRA)
        elif fg_img.shape[2] == 3:
            fg_img = cv2.cvtColor(fg_img, cv2.COLOR_BGR2BGRA)

        # Extract the alpha channel of the foreground
        alpha = fg_img[:, :, 3] / 255.0
        alpha_inv = 1.0 - alpha

        # Overlay the foreground on top of the background
        overlay = np.zeros_like(bkg_img, dtype=np.uint8)
        overlay[:, :, :3] = (fg_img[:, :, :3] * alpha[:, :, None] + bkg_img[:, :, :3] * alpha_inv[:, :, None]).astype(np.uint8)
        overlay[:, :, 3] = bkg_img[:, :, 3]  # Preserve the background alpha

        return overlay