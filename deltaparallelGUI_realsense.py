#note:the program is to be run with serial port connected to the arduino,otherwise error
#note:correct port number is to be defined,otherwise error

import cv2
import PySimpleGUI as sg
from pyzbar import pyzbar
import math as maths
import numpy as np
import serial
import time
from PIL import Image

arduino=serial.Serial('/dev/ttyS4', 9600)
time.sleep(2)

res =10

def  delta_calcAngleYZ(x0,y0,z0):               #function to calculate angle ,inverse kinematics

    e = 55     # end effector
    f = 110    # base
    re = 600   # forearm
    rf = 300   # arm
    stat = 0
    
    y1 = -0.5 * 0.57735 * f 
    #float y1 = yy1;
    y0 -= 0.5 * 0.57735 * e   # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if (d < 0): 
        #return 0
        stat = 1
    
    yj = (y1 - a*b - maths.sqrt(d))/(b*b + 1) # choosing outer point
    zj = a + b*yj
   
    theta = 180.0*maths.atan(-zj/(y1 - yj))/maths.pi
   
    if yj>y1:
              theta += 180.0
    if (theta < -42) or (theta > 90):
        #return 0
        stat = 1
        
    if (stat == 0):
        return theta
    else:        
        return 0       

def enco(T1,T2,T3):                         #function returns the number based on direction of motors,8 possibilities
    if T1 <= 0:
        if T2 <= 0:
            if T3 <= 0:
                no = 1
            elif T3 > 0:
                no = 2
        elif T2 > 0:
            if T3 <= 0:
                no = 3
            elif T3 > 0:
                no = 4
    elif T1 > 0:
        if T2 <= 0:
            if T3 <= 0:
                no = 5
            elif T3 > 0:
                no = 6
        elif T2 > 0:
            if T3 <= 0:
                no = 7
            elif T3 > 0:
               no = 8

    return no

def move(t1,t2,t3):
    T1 = int(t1)
    T2 = int(t2)
    T3 = int(t3)

    stp1 = int((6400/360)*T1)
    stp2 = int((6400/360)*T2)
    stp3 = int((6400/360)*T3)

    stp1 = abs(stp1)
    stp2 = abs(stp2)
    stp3 = abs(stp3)

    no = enco(T1,T2,T3)

    total = stp1 + stp2 + stp3
    if (stp1 > stp2 and stp1 > stp3):
        high = stp1
        no1 = 1
    elif (stp2 > stp1 and stp2 > stp3):
        high = stp2
        no1 = 2
    elif (stp3 > stp1 and stp3 > stp2):
        high = stp3
        no1 = 3
    else:
        high = stp1
        no1 = 1
        
    if (stp1 < stp2 and stp1 < stp3):
        low = stp1
        no3 = 1
    elif (stp2 < stp1 and stp2 < stp3):
        low = stp2
        no3 = 2
    elif (stp3 < stp1 and stp3 < stp2):
        low = stp3
        no3 = 3
    else:
        low = stp3
        no3 = 3    
          
    mid = total - (high + low)
    no2 = 6 - (no1 + no3)
    
    #move(mid,high,low,no2,no1,no3,1000)
    s1 = str(mid)
    s2 = str(high)
    s3 = str(low)
    n1 = str(no)
    m1 = str(no2)
    m2 = str(no1)
    m3 = str(no3)

    data = "901" + "+" + s1 + "+" + s2 + "+" + s3 + "+" + n1 + "+" + m1 + "+" + m2 + "+" + m3
    print(data)
    arduino.write(data.encode())        
    time.sleep(2)

def conveyor():
    conveyor = '902'
    print(conveyor)
    print("Moving conveyor")
    arduino.write(conveyor.encode())
    
def suction1():
    suction1 = '903'
    print(suction1)
    print("Suction cup on")
    arduino.write(suction1.encode())
    
def suction2():
    suction2 = '904'
    print(suction2)
    print("Suction cup off")
    arduino.write(suction2.encode())
  
def read_barcodes(frame):                               #inputs frame and returns barcode info and frame with barcode rectangle
    barcodes = pyzbar.decode(frame)
    barcode_info = 0
    for barcode in barcodes:
        x, y , w, h = barcode.rect

        barcode_info = barcode.data.decode('utf-8')
        cv2.rectangle(frame, (x, y),(x+w, y+h), (0, 0, 255), 5)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, barcode_info, (x + 6, y - 6), font, 2.0, (255, 0, 255), 3)
    return barcode_info,frame

def main():
    # Camera Settings
    camera_Width  = 480 #320 # 480 # 640 # 1024 # 1280
    camera_Heigth = 320 #240 # 320 # 480 # 780  # 960
    frameSize = (camera_Width, camera_Heigth)
    video_capture = cv2.VideoCapture(0)
    
    #flag to control start & stop
    i = 0

    # init Windows Manager
    sg.theme("LightGrey5")

    # def column1
    col11_layout = [[sg.Text("",size=(30,6))],
                    [sg.Button("Y+", size=(10, 1))],
                    [sg.Button('X-', size=(10, 1)),sg.Button('X+', size=(10, 1))],[],
                    [sg.Button('Y-', size=(10, 1))],
                    [sg.Text("",size=(30,6))]
                   ]
    col11 = sg.Column(col11_layout, element_justification='center')

    col12_layout = [[],[sg.Button('Z+', size=(10, 1))],[sg.Button('Z-', size=(10, 1))]]
    
    col12 = sg.Column(col12_layout, element_justification='center')
    
    colcam_layout = [[sg.Button('Camera ON', size=(10, 1)),sg.Button('Camera OFF', size=(10, 1))],
                     [sg.Text("Camera View", size=(60, 1), justification="center")],
                     [sg.Image(filename="",size=(30,30), key="cam1")],[],
                    ]
    colcam = sg.Column(colcam_layout, element_justification="center")

    row1layout = [col11,sg.VSeparator(),col12,sg.VSeparator(),colcam]
    
    col21_layout = [[sg.Text("X-Coordinate    :",size = (10,1)),sg.Input("0",size =(10,1), key="-X-")],[],
                    [sg.Text("Y-Coordinate    :",size = (10,1)),sg.Input("0",size =(10,1), key="-Y-")],[],
                    [sg.Text("Z-Coordinate(<0):",size = (10,1)),sg.Input("-520",size =(10,1), key="-Z-")],
                    [sg.Button('Calculate', size=(10, 1))]
                   ]
    col21 = sg.Column(col21_layout, element_justification='center')

    col22_layout = [[sg.Text("Current Angle",size = (10,1))],[sg.Input("60",size =(5,1), key="-I1-"),sg.Input("60",size =(5,1), key="-I2-"),sg.Input("60",size =(5,1), key="-I3-")],[],
                    [sg.Text("Angle 1:",size = (10,1)),sg.Input("0",size =(10,1), key="-A1-")],[],
                    [sg.Text("Angle 2:",size = (10,1)),sg.Input("0",size =(10,1), key="-A2-")],[],
                    [sg.Text("Angle 3:",size = (10,1)),sg.Input("0",size =(10,1), key="-A3-")],
                    [sg.Button('Move', size=(10, 1))]
                   ]
    col22 = sg.Column(col22_layout, element_justification='center')

    col23_layout = [[],
                    [sg.Button('Pick', size=(15, 1))],[sg.Button('Place', size=(15, 1))]
                   ]
    col23 = sg.Column(col23_layout, element_justification='center')

    row2layout = [col21,sg.VSeparator(),col22,sg.VSeparator(),col23,sg.VSeparator(),sg.Image(filename = '')]

    layout = [row1layout,[sg.HorizontalSeparator()],row2layout,[sg.Text("DELTA PARALLEL ROBOT", size=(80, 1),font='Lucida', justification="center")]]

    window    = sg.Window("Delta Parallel Robot", layout,size = (1100,650), 
                        no_titlebar=False, alpha_channel=1, grab_anywhere=False, 
                        return_keyboard_events=True, location=(100, 100))
    while True:
        event, values = window.read(timeout=20)
        
        if event == sg.WIN_CLOSED:
            break

        if event == 'X+':
            x = int(values["-X-"])
            x = x + res
            window["-X-"].update(x)
        if event == 'X-':
            x = int(values["-X-"])
            x = x - res
            window["-X-"].update(x)
        if event == 'Y+':
            y = int(values["-Y-"])
            y = y + res
            window["-Y-"].update(y)
        if event == 'Y-':
            y = int(values["-Y-"])
            y = y - res
            window["-Y-"].update(y)
        if event == 'Z+':
            z = int(values["-Z-"])
            z = z + res
            window["-Z-"].update(z)
        if event == 'Z-':
            z = int(values["-Z-"])
            z = z - res
            window["-Z-"].update(z)
        if event == 'Calculate':                            #calculate angle using inverse kinematics
            x0 = float(values["-X-"])
            y0 = float(values["-Y-"])
            z0 = float(values["-Z-"])

            cos120 = maths.cos(2.0*maths.pi/3.0)
            sin120 = maths.sin(2.0*maths.pi/3.0)

            theta1 = delta_calcAngleYZ(x0, y0, z0)
            theta2 = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0)  # rotate coords to +120 deg
            theta3 = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0)  # rotate coords to -120 deg

            window["-A1-"].update(theta1)
            window["-A2-"].update(theta2)
            window["-A3-"].update(theta3)

        if event == 'Move':                     ##send the data to the robot
            cur_th1 = float(values["-I1-"])
            cur_th2 = float(values["-I2-"])
            cur_th3 = float(values["-I3-"])
            
            th1 = float(values["-A1-"])
            th2 = float(values["-A2-"])
            th3 = float(values["-A3-"])
            
            tot_th1 = th1 - cur_th1
            tot_th2 = th2 - cur_th2
            tot_th3 = th3 - cur_th3
            move(tot_th1,tot_th2,tot_th3)

            window["-I1-"].update(th1)
            window["-I2-"].update(th2)
            window["-I3-"].update(th3)

        if event == 'Pick':
            suction1()
        if event == 'Place':
            suction2()
        if event == 'Camera ON':                    #turn on camera stream
            i = 1
        if event == 'Camera OFF':                   #turn off camera stream
            i = 0
                
        if i == 1:
            # get camera frame
            ret, frameOrig = video_capture.read()
            info,frameOrig = read_barcodes(frameOrig)
            frame = cv2.resize(frameOrig, frameSize)

            # # update webcam
            imgbytes = cv2.imencode(".png", frame)[1].tobytes()
            window["cam1"].update(data=imgbytes)
            
            if info:        #update barcode
                window["-BARCODE-"].update(info)
            #else:
                #window["-BARCODE-"].update("")
        if i == 0:
            window["cam1"].update()
    video_capture.release()
    cv2.destroyAllWindows()

main()
