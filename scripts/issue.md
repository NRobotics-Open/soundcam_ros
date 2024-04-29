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
