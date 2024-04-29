import rospy
from config import cfgContext

class SoundcamROS(object):
    def __init__(self) -> None:
        self.cfg = cfgContext
        self.p_pubRaw = False #TODO: not used
        """ self.useBulkConf = cfgContext['bulk_config'] #should the startup parameters be configured in bulk or individually
        self.pcProtocolVersion = cfgContext['protocol']
        self.deviceIP = cfgContext['ip']
        self.devicePort = cfgContext['port']
        
        #Streaming parameters
        self.p_pubVideo = cfgContext['publish_video']
        self.p_pubAcousticVideo = cfgContext['publish_acoustic_video']
        self.p_pubSpectrum = cfgContext['publish_spectrum']
        self.p_pubAudio = cfgContext['publish_audio']
        self.p_pubThermal = cfgContext['publish_thermal']
        self.p_pubRaw = False #TODO: not used

        self.visAudio = cfgContext['visualize_audio']
        self.visVideo = cfgContext['visualize_video']
        self.visAcVid = cfgContext['visualize_acoustic_video']
        self.visSpec = cfgContext['visualize_spectrum'] """

        if(cfgContext['use_ros']):
            #start publishers and service servers
            pass

    def publishVideo(self, img):
        pass