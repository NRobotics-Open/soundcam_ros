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
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! mpegtsmux ! tcpserversink host=127.0.0.1 port=8080

(fast)
gst-launch-1.0 -q v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast key-int-max=30 ! h264parse ! mpegtsmux ! tcpserversink host=127.0.0.1 port=5000 sync-method=2


# Client
gst-launch-1.0 -v tcpclientsrc host=127.0.0.1 port=5000 ! tsdemux ! h264parse ! avdec_h264 ! videoscale method=6 ! video/x-raw,width=1920,height=1080 ! videoconvert ! autovideosink sync=false

souphttpsrc location=http://192.168.3.100/mist?source=dc ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=I420,width=1280,height=960,framerate=24/1 ! timeoverlay draw-shadow=false draw-outline=false deltay=20 font-desc=\"Sans, 42\" color=0xFF000000 ! x264enc bframes=0 speed-preset=veryfast key-int-max=30 bitrate=100 ! video/x-h264,stream-format=byte-stream,profile=constrained-baseline ! queue silent=true ! rtph264pay mtu=1400 config-interval=-1 ! application/x-rtp,media=video,clock-rate=${$channels.video.clockRate},encoding-name=${$channels.video.encodingName},ssrc=(uint)${$channels.video.SSRC} ! queue silent=true ! udpsink host=127.0.0.1 port=${$pipeline.port} sync=false async=true

# Python Virtual Device publishing
    - Debug with: vlc v4l2:///dev/video4

    v4l2src device=/dev/video0 ! video/x-raw,format=I420,width=640,height=480,framerate=15/1 ! timeoverlay draw-shadow=false draw-outline=false deltay=20 font-desc=\"Sans, 42\" color=0xFF000000 ! x264enc bframes=0 speed-preset=veryfast key-int-max=30 bitrate=1500 ! video/x-h264,stream-format=byte-stream,profile=constrained-baseline ! queue silent=true ! rtph264pay mtu=1400 config-interval=-1 ! application/x-rtp,media=video,clock-rate=${$channels.video.clockRate},encoding-name=${$channels.video.encodingName},ssrc=(uint)${$channels.video.SSRC} ! queue silent=true ! udpsink host=127.0.0.1 port=${$pipeline.port} sync=false async=true


    v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=I420 ! x264enc bframes=0 speed-preset=veryfast key-int-max=30 bitrate=1500 ! video/x-h264,stream-format=byte-stream,profile=constrained-baseline ! queue silent=true ! rtph264pay mtu=1400 config-interval=-1 ! application/x-rtp,media=video,clock-rate=${$channels.video.clockRate},encoding-name=${$channels.video.encodingName},ssrc=(uint)${$channels.video.SSRC} ! queue silent=true ! udpsink host=127.0.0.1 port=${$pipeline.port} sync=false async=true


## VIRTUAL DEVICE CREATION [url](https://rmsol.de/2020/04/25/v4l2/)
    -   sudo modprobe v4l2loopback
    -   v4l2-ctl --list-devices

# SSH
    - ssh-keygen -f "/home/ephson/.ssh/known_hosts" -R "192.168.3.120"


