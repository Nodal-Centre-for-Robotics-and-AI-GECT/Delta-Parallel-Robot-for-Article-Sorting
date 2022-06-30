#code without gui to run the delta

import cv2
import PySimpleGUI as sg
from pyzbar import pyzbar
import math as maths
import numpy as np
import serial
import time

arduino=serial.Serial('COM3', 9600)
time.sleep(2)

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
def main():
    video_capture = cv2.VideoCapture(0)
    cos120 = maths.cos(2.0*maths.pi/3.0)
    sin120 = maths.sin(2.0*maths.pi/3.0)
    move(-30,-30,-30)
    time.sleep(5)
    conveyor()
    time.sleep(9)
    while 1:
        info = '0'
        ret, frameOrig = video_capture.read()
        info,frameOrig = read_barcodes(frameOrig)
        if info == '680009' or info == '680010':
            conveyor()
            time.sleep(13)
            move(28,28,28)
            time.sleep(5)
            suction1()
            time.sleep(2)
            move(-30,-30,-30)
            time.sleep(5)
            if info == '680009':
                x0 = -300
                y0 = -300
                z0 = -650
                print("(-300,-300,-650)")
                theta1 = delta_calcAngleYZ(x0, y0, z0)
                theta2 = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0)  # rotate coords to +120 deg
                theta3 = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0)  # rotate coords to -120 deg
                tot_th1 = theta1 - 30
                tot_th2 = theta2 - 30
                tot_th3 = theta3 - 30
                move(tot_th1,tot_th2,tot_th3)
                time.sleep(5)
                suction2()
                time.sleep(7)
                move(-tot_th1,-tot_th2,-tot_th3)
                time.sleep(5)
            elif info == '680010':
                x0 = 100
                y0 = 400
                z0 = -650
                print("(100,400,-650)")
                theta1 = delta_calcAngleYZ(x0, y0, z0)
                theta2 = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0)  # rotate coords to +120 deg
                theta3 = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0)  # rotate coords to -120 deg
                tot_th1 = theta1 - 30
                tot_th2 = theta2 - 30
                tot_th3 = theta3 - 30
                move(tot_th1,tot_th2,tot_th3)
                time.sleep(5)
                suction2()
                time.sleep(7)
                move(-tot_th1,-tot_th2,-tot_th3)
                time.sleep(5)
            conveyor()
            time.sleep(9)
main()              
            
