# unitree_actuator_sdk_ros2
ros2 driver for unitree motors
## Install
firstly change to your ros2 workspace
- download
```bash
cd src
git clone https://github.com/Leader-txt/unitree_actuator_sdk_ros2.git
```
- edit serial port at motor.cpp line 15
```c
serial = new SerialPort("/dev/ttyS7"); // edit here based on your port
```
- Install
``` bash
colcon build --packages-select unitree_actuator_sdk_ros2
```
## Useage
- run
```bash
source ${your_ros2_workspace}/install/setup.bash
ros2 run unitree_actuator_sdk_ros2 motor
```
## Message format
- Motor Cmd
```
uint8 id
uint8 mode
float32 tau
float32 vel
float32 pos
float32 kp
float32 kd
```
- Motor Data
```
uint8 id
uint8 mode
float32 tau
float32 vel
float32 pos
int8 temp
int8 error
uint16 force
```
- note
    - vel: dq
    - pos: q
## Test
- note : you shuld run the program firstly
- echo motor callback datas
```bash
source ${your_ros2_workspace}/install/setup.bash
ros2 topic echo /motor_data
```
- set motor:0 to none torque mode
``` bash
source ${your_ros2_workspace}/install/setup.bash
ros2 topic pub --once /motor_cmd unitree_actuator_sdk_ros2/msg/MotorCmd "{id: 0,mode: 1,tau: 0}"
```