import socket
import struct
import serial
import time
import threading

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QDial, QPushButton
from PyQt5.QtCore import Qt

ser = []

dsadas0 = 0
dsadas1 = 0
dsadas2 = 0
dsadas3 = 0

from Plugin import PluginManagement

def setting( port = "COM4", bit_rate = 115200 ):
    global ser
    ser = serial.Serial(port, 115200, timeout = 1)

def viewer():

    global dsadas0,dsadas1,dsadas2,dsadas3
    #PluginManagement.inits()

    pack_format = '<ffffff'
    reset_code = "888880123".encode()
    a = 0
    while(True):

        a += 1
        
        #PluginManagement.update()

        a1 = dsadas0#float(PluginManagement.AngularVelocity[0])
        a2 = dsadas1#float(PluginManagement.AngularVelocity[1])
        a3 = dsadas2#float(PluginManagement.AngularVelocity[2])

        b1 = 0#float(PluginManagement.Velocity[0])
        b2 = 0#float(PluginManagement.Velocity[1])
        b3 = 0#float(PluginManagement.Velocity[2])

        t = struct.pack(  pack_format,  a1,a2,a3,  b1,b2,b3  )
        ser.write(reset_code)
        ser.write(t)
        time.sleep(1/20)
        #print(a1,a2,a3,1,2,3)
        #print(b1,b2,b3)

def calculate():
    setting()
    viewer()

def SerialMonitoring():
    
    time.sleep(2)
    global ser
    while(1):
        try:
            print(str(ser.read().decode()),end='')
            time.sleep(0.01)
        except:
            time.sleep(0.01)


class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        self.slider0 = QSlider(Qt.Horizontal, self)
        self.slider1 = QSlider(Qt.Horizontal, self)
        self.slider2 = QSlider(Qt.Horizontal, self)
        self.slider3 = QSlider(Qt.Horizontal, self)
        self.slider0.move(30, 30)
        self.slider1.move(30, 30+30)
        self.slider2.move(30, 30+30+30)
        self.slider3.move(30, 30+30+30+30)
        self.slider0.setRange(-100, 100)
        self.slider1.setRange(-100, 100)
        self.slider2.setRange(-100, 100)
        self.slider3.setRange(-100, 100)
        self.slider0.setSingleStep(1)
        self.slider1.setSingleStep(1)
        self.slider2.setSingleStep(1)
        self.slider3.setSingleStep(1)

        #self.dial = QDial(self)
        #self.dial.move(30, 50)
        #self.dial.setRange(-1, 1)

        btn = QPushButton('Default', self)
        btn.move(35, 160)


        self.slider0.valueChanged.connect(self.cvd)
        self.slider1.valueChanged.connect(self.cvd)
        self.slider2.valueChanged.connect(self.cvd)
        self.slider3.valueChanged.connect(self.cvd)
        #self.slider.valueChanged.connect(self.dial.setValue)
        #self.dial.valueChanged.connect(self.slider.setValue)
        btn.clicked.connect(self.button_clicked)

        self.setWindowTitle('QSlider and QDial')
        self.setGeometry(300, 300, 400, 200)
        self.show()

    def cvd(self):
        global dsadas0,dsadas1,dsadas2,dsadas3
        dsadas0 = self.slider0.value()/100
        dsadas1 = self.slider1.value()/100
        dsadas2 = self.slider2.value()/100
        dsadas3 = self.slider3.value()/100
        print("ST :",dsadas0,dsadas1,dsadas2)

    def button_clicked(self):
        self.slider0.setValue(0)
        self.slider1.setValue(0)
        self.slider2.setValue(0)
        self.slider3.setValue(0)
        #self.dial.setValue(0)


def ui_thread():
    
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())


thread1 = threading.Thread(target=calculate, args=())
thread2 = threading.Thread(target=SerialMonitoring, args=())
thread3 = threading.Thread(target=ui_thread, args=())

thread1.start()
thread2.start()
thread3.start()

thread1.join()
thread2.join()
thread3.join()