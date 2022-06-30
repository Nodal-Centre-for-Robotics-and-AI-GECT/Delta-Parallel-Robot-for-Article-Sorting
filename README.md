# Delta-Parallel-Robot
Delta Parallel Robot for Article Sorting

Delta parallel robot is used for sorting of articles based on the barcode data.A graphical user interface is also developed for manual control of the robot.

Connect the data cable from the arduino to the usb port.
Install Arduino IDE,if not already installed.
Upload the arduino code into the arduino of the delta parallel robot(delta arduino code.ino).

Check the port name to which the arduino is connected(it will be displayed in arduino IDE along with board name) and type it correctly in the python program.

The packages imported in the python should be installed before running the program.
The packages required are pyrealsense2,cv2,PySimpleGUI,pyzbar,math,numpy,serial,time.

Run the deltaparallelGUI_webcam.py to control the delta robot and webcam camera manually.
In order to work with the intel realsense SR300 camera,the required SDK need to be installed.
Then,Run the deltaparallelGUI_realsense.py to control the delta robot and realsense camera manually.

Run automated letter sorting using realsense.py to achieve the continous article sorting operation of the delta parallel robot using realsense SR300.

Inverse kinematics.py is function to solve the inverse kinematics of the delta robot.
conveyor.ino is the arduino program which can be used to continously run and check the belt conveyor.
