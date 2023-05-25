
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QToolButton, QWidget, QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets 
import time 
#import cv2
#from PIL import ImageQt
import socket

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fire Extinguish")
        self.resize(450, 450)  # Set window size
        self.new_window = None  # Reference to the new window instance
        
        layout = QVBoxLayout()
        
        button1 = QToolButton(self)
        button1.setText("Start")
        button1.setStyleSheet("background-color: green; color: white;")  # Set button background color and text color
        button1.setGeometry(110, 40, 200,100)
        button1.clicked.connect(self.open_new_window)
        layout.addWidget(button1)
        
        button2 = QToolButton(self)
        button2.setText("Return")
        button2.setStyleSheet("background-color: red; color: white;")  # Set button background color and text color
        button2.setGeometry(110, 300, 200,100)
        button2.clicked.connect(self.stop)
        layout.addWidget(button2)
       

        self.host = '172.20.10.2'  # Raspberry Pi IP address
        self.port = 56000  # Port number for communication
        self.socket = None

    def connect_to_raspberry_pi(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print("Connected to Raspberry Pi")
        self.socket.sendall("Start".encode())  # Send a start message to Raspberry Pi
        print("Start message sent to Raspberry Pi")
    

        
    #    self.socket.close()

    def open_new_window(self):
        
        if not self.new_window:  # Create a new window only if it doesn't exist
            self.new_window = NewWindow()
            self.new_window.show()
        
    def stop(self):
        if not self.socket:
            self.connect_to_raspberry_pi()
        print("Connected to Raspberry Pi")

        self.socket.sendall("3".encode())  # Send a stop message to Raspberry Pi
        print("Return message sent to Raspberry Pi")
        time.sleep(1)
        self.close()
        self.socket.close()
        sys.exit(app.exec_())
        
    def level1(self):
        if not self.socket:
            self.connect_to_raspberry_pi()
        print("Connected to Raspberry Pi")

        self.socket.sendall("1".encode())  # Send a level1 message to Raspberry Pi
        print("LAB motion is sent to Raspberry Pi")
        
        #    self.socket.close()

    def level2(self):
        if not self.socket:
            self.connect_to_raspberry_pi()
        print("Connected to Raspberry Pi")

        self.socket.sendall("2".encode())  # Send a level2 message to Raspberry Pi
        print("outside motionis sent to Raspberry Pi")
        
        #    self.socket.close()

    

class NewWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("New Window")
        self.resize(450, 450)

        layout = QVBoxLayout()

        level1 = QToolButton(self)
        level1.setText("Build 1")
        level1.setStyleSheet("background-color: yellow; color: black;")  # Set button background color and text color
        layout.addWidget(level1)
        level1.setGeometry(110, 20, 200,100)

        level2 = QToolButton(self)
        level2.setText("Build 2")
        level2.setStyleSheet("background-color: purple; color: white;")  # Set button background color and text color
        layout.addWidget(level2)
        level2.setGeometry(110, 150, 200,100)

       
        button3 = QToolButton(self)
        button3.setText("Stop")
        button3.setStyleSheet("background-color: red; color: white;")  # Set button background color and text color
        button3.setGeometry(110, 300, 200,100)
        button3.clicked.connect(self.stop)
        layout.addWidget(button3)

        #central_widget = QWidget()
        #central_widget.setLayout(layout)
        #self.setCentralWidget(central_widget)
        level1.clicked.connect(self.level1)
        level2.clicked.connect(self.level2)
        button3.clicked.connect(self.level2)
        
        
        self.host = '172.20.10.2'  # Raspberry Pi IP address
        self.port = 56000  # Port number for communication
        self.socket = None

    def connect_to_raspberry_pi(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print("Connected to Raspberry Pi")

    def level1(self):
        if not self.socket:
            self.connect_to_raspberry_pi()  # Replace with Raspberry Pi IP address
        print("Connected to Raspberry Pi")

        self.socket.sendall("1".encode())  # Send a level1 message to Raspberry Pi
        print("Build 1 is sent to Raspberry Pi")
        
        #    self.socket.close()

    def level2(self):
        if not self.socket:
            self.connect_to_raspberry_pi()  # Replace with Raspberry Pi IP address
        print("Connected to Raspberry Pi")

        self.socket.sendall("2".encode())  # Send a level2 message to Raspberry Pi
        print("Build 2 is sent to Raspberry Pi")
        
        #    self.socket.close()

   
    def stop(self):
        if not self.socket:
            self.connect_to_raspberry_pi()
        print("Connected to Raspberry Pi")

        self.socket.sendall("3".encode())  # Send a stop message to Raspberry Pi
        print("Stop message sent to Raspberry Pi")
        time.sleep(1)
        self.close()
        self.socket.close()
        sys.exit(app.exec_())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
   
    sys.exit(app.exec_())
