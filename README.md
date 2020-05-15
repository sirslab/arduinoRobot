# arduinoRobot
Software for controlling an Arduino Robot, by Simone Marullo (2020)

## Installation and Usage
1. Move the content of arduino_robot folder into the `src` folder of your ROS workspace
2. Run `catkin_make`
3. Turn on the robot and the Bluetooth module of your computer
4. Use Bluetooth to establish communication on a serial port
5. Check the COM port in `nodes/bluetooth_proxy.py`
6. Run `roslaunch arduino_robot arduino_robot.launch`
