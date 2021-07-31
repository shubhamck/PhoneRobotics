# PhoneRobotics

Every roboticist has used IMU, Barometer, Camera, etc sensors for his/her robotics projects. But little does he know that the machine in your pocket i.e smartphone is just a very powerful sensor package!

In this repository we use only a smartphone as a sensor for our small robotics projects!

---
## Calibration

Before using any sensor it is necessary to calibrate it. We have helper scripts to calibrate each sensor on the phone. If you get a new phone please rerun the calibration scripts.


### Camera Calibration
We use standard chessboard calibration method.
To calibrate the camera run : 
```shell
python phone_cam_calibration.py --ip <IP_address> --port <port_number> 
```
This will save the calibration file to `config`  folder as a `json` file

To check more options please run : 
```shell
python phone_cam_calibration.py --help
```
