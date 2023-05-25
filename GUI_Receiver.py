#!/usr/bin/env python

import numpy as np
import cv2
import socket
import time
import serial


    
    
    
HOST = '172.20.10.2'  # Raspberry Pi IP address
PORT = 56000  # Port number for communication
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        ser1 = serial.Serial('/dev/ttyACM0',115200,timeout=1)
        ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
        ser.reset_input_buffer()
        
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                data = conn.recv(1024)  # Receive data from the laptop
                if not data:
                    break
                x=int (data.decode())
                print('Received:',x)
            
                if(x==1):
                    print('Building 1 is received successfuly')
                    GUI=1
                    ser1.write(b'1')
                    print ('1 is sent to arduino')
                    
                elif(x==2):
                    print('Building 2 is received successfuly')
                    GUI=2
                    ser1.write(b'2')
                    print ('2 is sent to arduino')
                    
                elif(x==3):
                    print('Return Button is received successfuly')
                    GUI=3
                    ser1.write(b'3')
                    print ('3 is sent to arduino')
                fire_cascade = cv2.CascadeClassifier('cascade.xml')
                #cascade.xml is the classifier file that contains the parameters of classifier
                    #checks for fire detection
                cap = cv2.VideoCapture(-1) #start video capturing
                count = 0
                while cap.isOpened():
                    ret, img = cap.read() #capture a frame
                    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert image to grayscale
                    fire = fire_cascade.detectMultiScale(img, 12, 5) #test for fire detection
                    for (x,y,w,h) in fire:
                        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2) #highlight the area of image with fire
                        roi_gray = gray[y:y+h, x:x+w]
                        roi_color = img[y:y+h, x:x+w]
                        print( 'Fire is detected..!' + str(count))
                        count = count + 1
                        if (count>10):
                            ser.write (b'T')
                            print('sent to arduino')
                            time.sleep(5)
                            count=0
                        else:
                            ser.write(b'F')
                            print('NO FIRE')
                        #time.sleep(1)
            
            
            
              
                    cv2.imshow('img', img)
                    k = cv2.waitKey(100) & 0xff
                    if k == 27:
                        break
                cap.release()
                cv2.destroyAllWindows()
                    
              



           



    
            
                
                
                

                
               
        
              
          
            
                
            
           
            
            


