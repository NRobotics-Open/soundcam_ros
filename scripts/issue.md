# DATATOSEND ISSUE
When querying for the default DataToSend config, the camera returns 75
which translates to: '1101001' 
According to Revision 3 of the documentation, page 45,
    AcousticPictures = 1
    VideoPictures = 1
    ThermalPictures = 0
    LocalSound = 1
    MicLevels = 0
    MicRawData = 0
    Spectra = 1

This seem correct!

When configuring the DataToSend parameter setting the corresponding byte to 3,
which translates to: '11000000'

The camera seems to accept the configuration and returns '3' upon querrying after configuration. However, the cyclical data being published when 'StartMeasurement' is called
mirrors the default 'DataToSend' structure and not the configured structure

# UPDATE
    - Configuring 'DataToSend' after 'PrepareState' and before 'StartProcedure'
        works if at least AcousticPictures and VideoPictures is set. 
    - Video Data Issue still persists. When using full resolution (640x480) there's
        significant lag (~ 2 - 3 seconds) when the stream works.
        - The video stream is not continuous and breaks every now and then
        - The framerate (fps) is something default (it doesn't seem to accept my configuration)
        - At half resolution: 
            - The framerate drops to around 5Hz
            - The visual image received is just a corner of the full resolution image instead of a scaled down version
            - User configured framerate (fps) is completely ignored as well



# VIDEO DATA ISSUE
Similar to the 'DataToSend' issue above, the parameters:
    Distance - sDis
    Frequency range - sFre
    Camera resolution - sVRe
    Framerate Video - sVFr
    Framerate acoustics - sAFr
can be queried successfully.

The above parameters can also be configured successfully using 'DataMessage' in bulk. 
The configured parameters have also been successfully verified.

Observation:
    The reported Acoustics data is according to the configured AcousticsFrameRate. This 
    has been verified.
    However, the reported Video is always being published at 5Hz. The camera does not publish
    the video stream at the specified VideoDataFrameRate


We have also confirmed that this is not a buffer related issue and all bytes are being read and
processed as and when they are available.

Probable header contained here: 

b'u\r\x00\x00\x89\x0b\x00\x00\x8e\x0b\x00\x00\x00\t\x00\x00F\x0c\x00\x00\xae\x0e\x00\x00F\x0c\x00\x00J\n\x00\x00\xc9\x08\x00\x00H\x08\x00\x00\x93\x08\x00\x00z\x0c\x00\x00R\n\x00\x00c\t\x00\x00\x1d\x08\x00\x00{\x08\x00\x00\xfb\x07\x00\x00m\x08\x00\x00\xc2\n\x00\x00\xfc\x07\x00\x00\x00\x08\x00\x00\xe0\x05\x00\x00U\x06\x00\x00\x9a\x07\x00\x00\xf9\x08\x00\x00\xbe\x07\x00\x00\xb9\x04\x00\x00\xe3\x04\x00\x00^\x06\x00\x00\xc1\x05\x00\x00\xec\x06\x00\x00\xb1\x05\x00\x00'
