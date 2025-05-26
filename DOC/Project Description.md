The System is comprised of:

Compute:
- raspberry pi 5

Inertial Measurement Unit:

- an arduino MKR Zero, which transfers IMU data to the pi. 
    See IMU_Data_Collection_v1.ino in ArudinoMKRZero.
    the data sent to the serial port is a per-line series of json.

    `{"t":timestamp,"acc":[acc_x,acc_y,acc_z],"gyro":[gyro_x,gyro_y,gyro_z],"mag":[mag_x,mag_y,mag_z]}`

    note that t is the time since the arduino boot in seconds. 
    It will need calibration with the raspberry pi system time.
    acc, gyro, and mag are the sensor vectors. They will need calibration
    mag may or may not exist in the data; it has a lower sample rate, and is therefore sent less often:
    ```
    {"t":28.381891,"acc":[0.0020,-0.9740,0.1580],"gyro":[0.3750,0.0625,-0.3125]}
    {"t":28.392819,"acc":[0.0020,-0.9750,0.1590],"gyro":[0.2500,0.0625,-0.2500],"mag":[-3.4,14.4,16.8]}
    ```

- attached to an Arduino MKR IMU shield, 
    which is a BNO055 intelligent absolute orientaion sensor. The MRKIMU arduino library 
    is here: https://github.com/arduino-libraries/MKRIMU

Lidar: An FHL-LD19 2d lidar. It sends data over the serial. 
    RaspberryPiCode/lidar_api.py exists to read the data on a separate thread. 
    Example usage in RaspberryPiCode/lidar_api_example.py

