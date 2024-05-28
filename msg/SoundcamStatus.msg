std_msgs/Header header

std_msgs/String deviceId
std_msgs/Int protocolVersion
std_msgs/String hardwareVersion
std_msgs/String firmwareVersion
std_msgs/Int deviceError #0-> OK
std_msgs/Float32 cpuTemperature
std_msgs/Float32 temperatureTop
std_msgs/Float32 temperatureBottom
std_msgs/String deviceState  #Idle, Measuring, Streaming, Service_1, Ultrasonic, Ultrasonic_plus
std_msgs/String subState     #Idle, Running, Preparing, Finished, Aborted, Locked
std_msgs/String deviceIP
std_msgs/String currentPreset
std_msgs/Bool detector   #True -> active detection