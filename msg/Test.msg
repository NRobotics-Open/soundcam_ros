std_msgs/Header header

uint8 CMD_TYPE_CONFIG = 0
uint8 CMD_TYPE_OP = 1
#Command :let the backend know what of call this is
uint8 command_type

#Presets (Lookup Table)     
float32 captureTime     #how long to record the stream
soundcam_ros/Preset preset

uint8 VIDEO_STREAM = 0
uint8 AUDIO_STREAM = 1
uint8 ACOUSTIC_STREAM = 2
uint8 THERMAL_STREAM = 3
uint8 SPECTRUM_STREAM = 4
uint8 OVERLAY_STREAM = 5
uint8 ALL = 6
string mediaType      #'VIDEO_STREAM|ACOUSTIC_STREAM'

#Input Operational Parameters 
#==============================
uint8 RESET_CAMERA = 0
uint8 START_RECORD = 1      #op_command1 = {0->BW Image, 3->Thermal Image, 5-> Overlayed Image}
uint8 STOP_RECORD = 2
uint8 CANCEL_RECORD = 3
uint8 SNAPSHOT = 4          #op_command1 = {0->BW Image, 3->Thermal Image, 5-> Overlayed Image}
uint8 SET_DYNAMIC_CREST = 5 #op_command1 = DYNAMIC, op_command2 = CREST
uint8 SET_SCALING_MODE = 6  #op_command1 (0-off, 1-Auto, 2-Smart) |0-off ->(op_command2=maximum), 1-Auto, 2-Smart -> (op_command2=dynamic, op_command3=crest)
uint8 SCAN_DETECT = 7       #op_command1(scan) -> 1-true, 0-false | op_command2(detect)-> 1-true, 0-false
uint8 command

float32 op_command1
float32 op_command2
float32 op_command3

#if command_type=OP_CMD: -> set extras {"key":"value"}
#if CMD_TYPE_CONFIG
    #0- [MissionID, MissionName]
diagnostic_msgs/KeyValue[] extras   