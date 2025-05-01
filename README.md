# Autonomous vehicle path following
This repository contains the code for constant speed path following of an autonomous vehicle. The path following is achieved via controlling lateral error relative to the given path via a PID control while the speed remains constant.  
The project uses an RC truck that is modified by adding encoders, a Teensy 4.0 microcontroller, an IMU, and a Nvidia TX2 carrier board. Also, the position of the vehicle is captured via a Vicon motion capture system. The communications, sensor fusion, and control calculations are implemented in C++ and ROS1.
<!-- Row of images with captions -->
<table width="100%">
  <tr>
    <th>Encoder assembly</th>
    <th>Vehicle assembly</th>
    <th>Vehicle full assembly</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/ce45feba-8247-4a66-b54a-846ab87fa10f" width="100%"/></td>
    <td><img src="https://github.com/user-attachments/assets/a27c1f54-4c42-413b-97f9-8bc9c41987b7" width="100%"/></td>
    <td><img src="https://github.com/user-attachments/assets/cb7dcf55-d7c5-4871-ac28-588666ced912" width="100%"/></td>
  </tr>
</table>
<!-- Spacer -->
<br/>
<!-- Centered video and caption -->
<p align="left"><strong>Video of the vehicle following a racetrack-shaped path autonomously</strong></p>
<p align="center">
  <a href="https://www.youtube.com/watch?v=zFBqgbBdzSg">
    <img src="https://img.youtube.com/vi/zFBqgbBdzSg/0.jpg" width="50%"/>
  </a>
</p>
<!-- Row of images with captions -->
<table width="100%">
  <tr>
    <th>Steering command along path</th>
    <th>Wheels angle along the path</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/6c3d747c-619c-4936-a5bd-6865f8f10159" width="100%"/></td>
    <td><img src="https://github.com/user-attachments/assets/62d4a89e-0934-4604-8581-20efd6e972be" width="100%"/></td>
  </tr>
</table>
The path tracking error is relatively high due to the long delay of the steering servo (~150-180 ms). To reduce the error a controller that uses future information from the path, like Model Predictive Control, is needed.

## Structure of the code
The structure of this repository is:
```
src/
    kd_tree/
    path/
    CMakelists.txt
    log.cpp
    package.xml
    teensy.io
```
- `kd_tree/` contains my implementation of the kd-tree algorithm to identify the nearest point on the path to the vehicle's current position. For shorter paths with fewer points, a brute-force search is sufficiently fast.
- `path/` contains path objects that hold specifications of different paths.
- `CMakelists.txt` and `package.xml` are ROS1 configuration files.
- `log.cpp` is the main file that puts everything together. It manages sensor reading and fusion, control command calculation, and communicationg it to the vehicle's actuators. All ROS1 publishers and subscribers are defined in this file.
- `teensy.io` is the Teensy 4.0 driver that manages low-level sensor readings and control command execution, using rosserial for subscribing to commands and publishing sensor data.
