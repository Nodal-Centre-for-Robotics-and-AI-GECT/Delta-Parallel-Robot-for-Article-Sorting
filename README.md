# Delta-Parallel-Robot
Delta Parallel Robot for Article Sorting

Connect the data cable from the arduino to the usb port.
First,upload the arduino code into the arduino of the delta parallel robot(delta arduino code.ino).

Check the port name to which the arduino is connected(it will be displayed in arduino IDE along with board name) and type it correctly in the python program

The packages imported in the python should be installed before running the program.
Run the deltaparallelGUI_webcam.py to control the delta robot and webcam camera manually.

In order to work with the intel realsense SR300 camera,the required SDK need to be installed.
Run the deltaparallelGUI_webcam.py to control the delta robot and realsense camera manually.

Run automated letter sorting using realsense.py to achieve the continous article sorting operation of the delta parallel robot.
