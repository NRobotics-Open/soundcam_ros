Command:int
    InvokeId:int
    Reserved:int
    Status1:int
    DataLength:int
    DeviceProtocolVersion:int
    DeviceId:hex
    DeviceMode:int
    DeviceError:int
    DeviceSerialNumber:int
    SiliconSerialNumber:hex 
    HardwareVersion:int
    BootloaderVersion:hex 
    FirmwareVersion:hex
    FPGAVersion:int 
    ProductionDate:datetime 
    CalibrationDate:datetime 
    MACAddress:str
    IPAddress:str
    PCBSerialNumber:int 
    Features1:Features
    Features2:int 
    Features3:int 
    Status2:Status 
    ConnectIP:str 
    DeviceState:DeviceStates
    SubState:SubStates
    Error:int 
    _Step0:int
    _Step_1:int
    TemperatureCPU:float 
    TemperatureTop:float 
    TemperatureBottom:float
    CPULoad_1:int 
    CPULoad_2:int

# Gstreamer Server
(slow)
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! mpegtsmux ! tcpserversink host=127.0.0.1 port=8080

(fast)
gst-launch-1.0 -q v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast key-int-max=30 ! h264parse ! mpegtsmux ! tcpserversink host=127.0.0.1 port=5000 sync-method=2


# Client
gst-launch-1.0 -v tcpclientsrc host=127.0.0.1 port=5000 ! tsdemux ! h264parse ! avdec_h264 ! videoscale method=6 ! video/x-raw,width=1920,height=1080 ! videoconvert ! autovideosink sync=false

# Python Virtual Device publishing
    - Debug with: vlc v4l2:///dev/video4

