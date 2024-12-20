std_msgs/Header header #frame_id:=soundcam

std_msgs/String deviceId
uint8 protocolVersion
std_msgs/String hardwareVersion
std_msgs/String firmwareVersion
uint8 deviceError #0-> OK
std_msgs/String deviceIP
std_msgs/String deviceMAC
std_msgs/Float32 cpuTemperature
std_msgs/Float32 temperatureTop
std_msgs/Float32 temperatureBottom
std_msgs/String deviceState  #Idle, Measuring, Streaming, Service_1, Ultrasonic, Ultrasonic_plus
std_msgs/String subState     #Idle, Running, Preparing, Finished, Aborted, Locked
diagnostic_msgs/KeyValue[] features
diagnostic_msgs/KeyValue[] status
