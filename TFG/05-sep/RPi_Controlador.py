#   CÓDIGO GENERADO A 07/09/2021
#   Ignacio Martínez Vignardi
#   Ingeniería Electrónica Industrial y Automática
#   RPi_Controlador_v1_0_0
#   
#   Este código corresponde con el realizado para la fecha de entrega
#   del TFG, Diseño y programación de un robot articulado filmográfico 
#   con visión por computador  

import sys
import time
import serial
import math
import constants
import cv2
#import cocodetect
from PyQt5.QtGui import * 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from imutils.video import FPS

inputFreq = [0, 0, 0, 0, 0, 0]
nextInputFreq = [0, 0, 0, 0, 0, 0] #Used for programmed movement
desiredPulses = [0, 0, 0, 0, 0, 0]
nextDesiredPulses = [0, 0, 0, 0, 0, 0] #Used for programmed movement
direction = [0, 0, 0, 0, 0, 0]
nextDirection = [0, 0, 0, 0, 0, 0] #Used for programmed movement

deltaArray = [0, 0, 0, 0, 0, 0]

freqMove = 0

controlByte = 0x00

calibrated = False


jointTimes = [0, 0, 0, 0, 0, 0]

delta = 0

prevX = constants.LENGTH_12B + constants.LENGTH_34
prevY = 0
prevZ = constants.LENGTH_12A + constants.LENGTH_23
previousPitch = 90
x = 0
y = 0
z = 0
pitch = 0
#0x00 stop joint movement
#0x01 move joint
#0x02 move linear
#0x03 set position
#0x04 erase positions
#0xFF stop programmed movement
#0xFE pause programmed movement
#0xFD start programmed movement


speedScaler = 0 #0.0 to 1.0 in 0.1 increments
isFloat = False

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.isPaused=False
        self.endCalib=0x00
        self.firstPass=False
        self.finish=False
        self.focusConfirmed = False
        self.initialFocus = False
        self.focused = False
        self.count=0
        self.j1a = constants.INITIAL_ANG[0]
        self.j2a = constants.INITIAL_ANG[1]
        self.j3a = constants.INITIAL_ANG[2]
        self.j4a = constants.INITIAL_ANG[3]
        self.j5a = constants.INITIAL_ANG[4]
        self.j6a = constants.INITIAL_ANG[5]
        self.x = constants.LENGTH_12B + constants.LENGTH_34
        self.y = 0
        self.z = constants.LENGTH_12A + constants.LENGTH_23 - constants.LENGTH_WR
        self.pitch = 90
        self.remainder = [0,0,0,0,0,0]
        self.setWindowTitle("Robot Controller")
                        
        self.initUI()
        
        
    def initUI(self):
        layout = QGridLayout()
        
        self.tabs = QTabWidget()
        self.mainmenu = QWidget()
        self.tracker = QWidget()
        self.tabs.resize(300,200)
        
        self.tabs.addTab(self.mainmenu, "Trajectories")
        self.tabs.addTab(self.tracker, "Object following")
        
        self.mainmenu.layout=QGridLayout(self)
        
        label_initial = QLabel("LINEAR CONTROL")
        label_initial.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_initial, 0, 0, 1, 3)        
        
        
        self.button_less_X = QPushButton('-X')
        self.mainmenu.layout.addWidget(self.button_less_X, 1, 0)
        self.button_less_X.pressed.connect(lambda:self.on_click_linear(-1,0,0))
        self.button_less_X.released.connect(lambda:self.on_release_no_move())
        
        label_X = QLabel("X variation")
        label_X.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_X, 1, 1)
        
        self.button_more_X = QPushButton('+X', self)
        self.mainmenu.layout.addWidget(self.button_more_X, 1, 2)
        self.button_more_X.pressed.connect(lambda:self.on_click_linear(1,0,0))
        self.button_more_X.released.connect(lambda:self.on_release_no_move())
        
        
        
        self.button_less_Y = QPushButton('-Y')
        self.mainmenu.layout.addWidget(self.button_less_Y, 2, 0)
        self.button_less_Y.pressed.connect(lambda:self.on_click_linear(0,-1,0))
        self.button_less_Y.released.connect(lambda:self.on_release_no_move())
        
        label_Y = QLabel("Y variation")
        label_Y.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_Y, 2, 1)
        
        self.button_more_Y = QPushButton('+Y', self)
        self.mainmenu.layout.addWidget(self.button_more_Y, 2, 2)
        self.button_more_Y.pressed.connect(lambda:self.on_click_linear(0,1,0))
        self.button_more_Y.released.connect(lambda:self.on_release_no_move())
        
        
        
        self.button_less_Z = QPushButton('-Z')
        self.mainmenu.layout.addWidget(self.button_less_Z, 3, 0)
        self.button_less_Z.pressed.connect(lambda:self.on_click_linear(0,0,-1))
        self.button_less_Z.released.connect(lambda:self.on_release_no_move())
        
        label_Z = QLabel("Z variation")
        label_Z.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_Z, 3, 1)
        
        self.button_more_Z = QPushButton('+Z', self)
        self.mainmenu.layout.addWidget(self.button_more_Z, 3, 2)
        self.button_more_Z.pressed.connect(lambda:self.on_click_linear(0,0,1))
        self.button_more_Z.released.connect(lambda:self.on_release_no_move())
        
        
        
        label_empty = QLabel("")
        label_empty.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_empty, 4, 0, 1, 3)  
        
        
        
        label_inter = QLabel("JOINT CONTROL")
        label_inter.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_inter, 5, 0, 1, 6)        
        
        
        
        
        self.button_less_J1 = QPushButton('-J1')
        self.mainmenu.layout.addWidget(self.button_less_J1, 6, 0)
        self.button_less_J1.pressed.connect(lambda:self.on_click_J(1, -1))
        self.button_less_J1.released.connect(lambda:self.on_release_no_move())
        
        label_J1 = QLabel("J1 variation")
        label_J1.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J1, 6, 1)
        
        self.button_more_J1 = QPushButton('+J1', self)
        self.mainmenu.layout.addWidget(self.button_more_J1, 6, 2)
        self.button_more_J1.pressed.connect(lambda:self.on_click_J(1, 1))
        self.button_more_J1.released.connect(lambda:self.on_release_no_move())
        
        
        
        
        self.button_less_J2 = QPushButton('-J2')
        self.mainmenu.layout.addWidget(self.button_less_J2, 7, 0)
        self.button_less_J2.pressed.connect(lambda:self.on_click_J(2, -1))
        self.button_less_J2.released.connect(lambda:self.on_release_no_move())
        
        label_J2 = QLabel("J2 variation")
        label_J2.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J2, 7, 1)
        
        self.button_more_J2 = QPushButton('+J2', self)
        self.mainmenu.layout.addWidget(self.button_more_J2, 7, 2)
        self.button_more_J2.pressed.connect(lambda:self.on_click_J(2, 1))
        self.button_more_J2.released.connect(lambda:self.on_release_no_move())
        
        
        
        self.button_less_J3 = QPushButton('-J3')
        self.mainmenu.layout.addWidget(self.button_less_J3, 8, 0)
        self.button_less_J3.pressed.connect(lambda:self.on_click_J(3, -1))
        self.button_less_J3.released.connect(lambda:self.on_release_no_move())
        
        label_J3 = QLabel("J3 variation")
        label_J3.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J3, 8, 1)
        
        self.button_more_J3 = QPushButton('+J3', self)
        self.mainmenu.layout.addWidget(self.button_more_J3, 8, 2)
        self.button_more_J3.pressed.connect(lambda:self.on_click_J(3, 1))
        self.button_more_J3.released.connect(lambda:self.on_release_no_move())
        
        
        
        
        self.button_less_J4 = QPushButton('-J4')
        self.mainmenu.layout.addWidget(self.button_less_J4, 6, 3)
        self.button_less_J4.pressed.connect(lambda:self.on_click_J(4, -1))
        self.button_less_J4.released.connect(lambda:self.on_release_no_move())
        
        label_J4 = QLabel("J4 variation")
        label_J4.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J4, 6, 4)
        
        self.button_more_J4 = QPushButton('+J4', self)
        self.mainmenu.layout.addWidget(self.button_more_J4, 6, 5)
        self.button_more_J4.pressed.connect(lambda:self.on_click_J(4, 1))
        self.button_more_J4.released.connect(lambda:self.on_release_no_move())
        
        
        
        
        self.button_less_J5 = QPushButton('-J5')
        self.mainmenu.layout.addWidget(self.button_less_J5, 7, 3)
        self.button_less_J5.pressed.connect(lambda:self.on_click_J(5, -1))
        self.button_less_J5.released.connect(lambda:self.on_release_no_move())
        
        label_J5 = QLabel("J5 variation")
        label_J5.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J5, 7, 4)
        
        self.button_more_J5 = QPushButton('+J5', self)
        self.mainmenu.layout.addWidget(self.button_more_J5, 7, 5)
        self.button_more_J5.pressed.connect(lambda:self.on_click_J(5, 1))
        self.button_more_J5.released.connect(lambda:self.on_release_no_move())
        
        
        
        
        self.button_less_J6 = QPushButton('-J6')
        self.mainmenu.layout.addWidget(self.button_less_J6, 8, 3)
        self.button_less_J6.pressed.connect(lambda:self.on_click_J(6, -1))
        self.button_less_J6.released.connect(lambda:self.on_release_no_move())
        
        label_J6 = QLabel("J6 variation")
        label_J6.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(label_J6, 8, 4)
        
        self.button_more_J6 = QPushButton('+J6', self)
        self.mainmenu.layout.addWidget(self.button_more_J6, 8, 5)
        self.button_more_J6.pressed.connect(lambda:self.on_click_J(6, 1))
        self.button_more_J6.released.connect(lambda:self.on_release_no_move())
        
        
        
        
        self.button_save_position = QPushButton('SAVE POSITION')
        self.mainmenu.layout.addWidget(self.button_save_position, 1, 3, 1, 3)
        self.button_save_position.clicked.connect(lambda:self.on_click_save())
        
        
        
        
        self.button_erase_position = QPushButton('ERASE POSITIONS')
        self.mainmenu.layout.addWidget(self.button_erase_position, 2, 3, 1, 3)
        self.button_erase_position.clicked.connect(lambda:self.on_click_erase())
        
        
        
        self.gotta_loop = QCheckBox("LOOP")
        self.button_start = QPushButton('START')
        self.button_start.clicked.connect(lambda:self.on_click_start())
        self.horizontal = QHBoxLayout()
        self.horizontal.addWidget(self.gotta_loop)
        self.horizontal.addWidget(self.button_start)
        self.mainmenu.layout.addLayout(self.horizontal, 3, 3)
        
        
        self.button_pause = QPushButton('PAUSE')
        self.button_pause.setEnabled(False)
        self.mainmenu.layout.addWidget(self.button_pause, 3, 4)
        self.button_pause.clicked.connect(lambda:self.on_click_pause())
        
        
        
        self.button_stop = QPushButton('STOP')
        self.button_stop.setEnabled(False)
        self.mainmenu.layout.addWidget(self.button_stop, 3, 5)
        self.button_stop.clicked.connect(lambda:self.on_click_stop())
        
            
        
        self.edit_speed = QSlider(Qt.Horizontal)
        self.edit_speed.setMinimum(10)
        self.edit_speed.setMaximum(100)
        self.edit_speed.setValue(10)
        self.edit_speed.setSingleStep(10)
        self.edit_speed.setTickInterval(10)
        self.edit_speed.setTickPosition(QSlider.TicksBelow)
        self.edit_speed.valueChanged.connect(lambda:self.valuechanged())
        self.mainmenu.layout.addWidget(self.edit_speed, 0, 4, 1, 2)
        
        
        self.label_speed = QLabel("SPEED MULTIPLIER: %.2f"%float(self.edit_speed.value()/100))
        self.label_speed.setAlignment(Qt.AlignCenter)
        self.mainmenu.layout.addWidget(self.label_speed, 0, 3)
        
        self.mainmenu.setLayout(self.mainmenu.layout)
        
        
        
        self.tracker.layout = QGridLayout(self)
        
        self.box = QVBoxLayout()
        
        self.feed = QLabel()
        self.tracker.layout.addWidget(self.feed, 0, 0)
               
        self.setTracker = QPushButton("Assign object")
        self.setTracker.clicked.connect(lambda:self.assign_obj())
        self.box.addWidget(self.setTracker)
        
        self.initialize = QPushButton("Start tracking")
        self.initialize.clicked.connect(lambda:self.start_tracking())
        self.box.addWidget(self.initialize)
        
        self.stopfollow = QPushButton("Stop tracking")
        self.stopfollow.clicked.connect(lambda:self.stop_tracking())
        self.box.addWidget(self.stopfollow)

        self.tracker.layout.addLayout(self.box, 0, 1)
        
        self.myThread = myThread(None)
        self.myThread.videoFeed.connect(self.videoUpdater)
        self.myThread.start()
        
        self.tracker.setLayout(self.tracker.layout)
        
        
        layout.addWidget(self.tabs)
        self.setLayout(layout)
        
        
        self.calibMsgBox = QMessageBox()
        self.calibMsgBox.setWindowTitle("Calibration")
        self.calibMsgBox.setText("An initial calibration is required\nPlease, stand at a safe distance\nStart?")
        self.calibMsgBox.setIcon(QMessageBox.Warning)
        self.calibMsgBox.buttonClicked.connect(self.popup_actions)
        
        self.calibMsgBox2 = QMessageBox()
        self.calibMsgBox2.setWindowTitle("cALIBRATION")
        self.calibMsgBox2.setText("Robot calibration started.\nNo actions will be available until the process finishes")
        
        self.posSavedMsgBox = QMessageBox()
        self.posSavedMsgBox.setWindowTitle("Saved")
        self.posSavedMsgBox.setText("The position for the trajectory was saved")
        
        self.focusCamera = QMessageBox()
        self.focusCamera.setWindowTitle("Focus")
        self.focusCamera.setText("Se va a realizar el enfoque de la cámara, pulse para confirmar inicio")
        self.focusCamera.buttonClicked.connect(self.popup_actions2)
        
        self.focusCameraOk = QMessageBox()
        self.focusCameraOk.setWindowTitle("Focus confirmation")
        self.focusCameraOk.setText("Is the object focused?")
#        self.focusCameraOk.buttonClicked.connect(self.popup_actions2)
        self.focusCameraOk.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        
        self.initiating = QMessageBox()
        self.initiating.setWindowTitle("Initiating")
        self.initiating.setText("Moving robot to the initial position, the window will close automatically.")
        self.initiating.buttonClicked.connect(self.popup_actions_init)
        
    def start_tracking(self):
        self.myThread.beginTrack()
        self.mainmenu.setEnabled(False)
    
    def stop_tracking(self):
        self.myThread.stopTrack()
        self.mainmenu.setEnabled(True)

    def assign_obj(self):
        self.myThread.stop()
        self.videocap = cv2.VideoCapture(0)
        success, img = self.videocap.read()
        img = cv2.resize(img, (320, 240))
        bbox = cv2.selectROI("Tracker", img, False)
        print(bbox)
        self.videocap.release()
        cv2.destroyWindow("Tracker")
        self.myThread = myThread(bbox)
        self.myThread.videoFeed.connect(self.videoUpdater)
        self.myThread.start()


    def videoUpdater(self, image):
        self.feed.setPixmap(QPixmap.fromImage(image))
    
        
    def valuechanged(self):
        value=self.edit_speed.value()
        self.label_speed.setText("SPEED MULTIPLIER: %.2f"%(float(value/100.0)))
    
    def paintEvent(self, event):
        if self.endCalib==0x00:
            self.calibMsgBox.exec_()
            
            
    def popup_actions(self):
        arduino.write((15).to_bytes(1, "little")) #COMMENT FOR PROGRAM TESTING
        print((15).to_bytes(1, byteorder="little"))
        self.endCalib=0x01 #UNCOMMENT FOR PROGRAM TESTING! 
        self.calibMsgBox2.exec_() #Warning message
        
        while(self.endCalib==0x00):
            if(arduino.in_waiting==1): 
                self.endCalib=arduino.read()
        if self.endCalib==0xFF:
            print("Calibration error")
            exit()
            
    def popup_actions2(self):
        if self.initialFocus==False:
            arduino.write(0x01)
            focusCameraOk.exec_()
        elif self.initialFocus==True:
            if self.focusCameraOk.standardButton(self.focusCameraOk.clickedButton()) == QMessageBox.Yes:
                arduino.write(0x02)
            else:
                arduino.write(0x01)
        
            
    def popup_actions_init(self):
        while arduino.in_waiting==0:
            if arduino.in_waiting>0:
                t=arduino.read()
                self.initiating.close()
            #GUI locked state
    
    def on_click_linear(self, isX, isY, isZ):                            
        controlByte = (2).to_bytes(1, byteorder="little")
        # print(controlByte)
        arduino.write(controlByte)
        try:
            float(self.edit_speed.value()/100.0)
            isFloat = True
        except ValueError:
            isFloat = False
        
            
        if isFloat and float(self.edit_speed.value()/100)>=0.05 and float(self.edit_speed.value()/100)<1.05:
            speed_scaler = float(self.edit_speed.value()/100)
            speed_scaler = round(speed_scaler, 1)
            freqMove = int(constants.MAX_FREQ*speed_scaler)
        
            freqMove = freqMove.to_bytes(2, byteorder = "little")
            # print(freqMove)
            arduino.write(freqMove)
            
            if isX!=0:
                arduino.write((1).to_bytes(1, byteorder="little", signed=True))
                arduino.write(isX.to_bytes(1, byteorder="little", signed=True))
            elif isY!=0:
                # print((2).to_bytes(1, byteorder="little", signed=True))
                # print(isY.to_bytes(1, byteorder="little", signed=True))
                arduino.write((2).to_bytes(1, byteorder="little", signed=True))
                arduino.write(isY.to_bytes(1, byteorder="little", signed=True))
            elif isZ!=0:
                arduino.write((3).to_bytes(1, byteorder="little", signed=True))
                arduino.write(isZ.to_bytes(1, byteorder="little", signed=True))
        else:
            print("Introduzca un float válido (Entre 0.1 y 1.0 en incrementos de 0.1)")
            print("Si no se cumplen los incrementos el programa redondeará el valor\n")
            
        
        
    def on_click_J(self, joint, thisDirection):
        
        controlByte = (1).to_bytes(1, byteorder="little") #Joint movement data byte, direct assignment not working
        arduino.write(controlByte)
        # print(controlByte)
        try:
            float(self.edit_speed.value()/100.0)
            isFloat = True
        except ValueError:
            isFloat = False
            
        if isFloat and float(self.edit_speed.value()/100)>=0.05 and float(self.edit_speed.value()/100)<1.05:
            speed_scaler = float(self.edit_speed.value()/100)
            speed_scaler = round(speed_scaler, 1)
            
            freqMove = int(constants.MAX_FREQ*speed_scaler)
            freqMove=freqMove.to_bytes(2, byteorder="little")
            # print(freqMove);
            arduino.write(freqMove)
            joint = joint.to_bytes(1, byteorder="little")
            arduino.write(joint)
            # print(joint);
            thisDirection = thisDirection.to_bytes(1, byteorder="little", signed=True)
            # print(thisDirection);
            arduino.write(thisDirection)
        else:
            print("Introduzca un float válido (Entre 0.1 y 1.0 en incrementos de 0.1)")
            print("Si no se cumplen los incrementos el programa redondeará el valor\n")
            
        
    def on_release_no_move(self):
        #When advance button released put to 0 everything
        while arduino.in_waiting>0:
            t = arduino.read()
        arduino.write(0x00)
        arduino.flush()
            
            
    def on_click_save(self):
        controlByte = (3).to_bytes(1, byteorder="little")
        # print(controlByte)
        arduino.write(controlByte)
        try:
            float(self.edit_speed.value()/100.0)
            isFloat = True
        except ValueError:
            isFloat = False
            
        if isFloat and float(self.edit_speed.value()/100)>=0.05 and float(self.edit_speed.value()/100)<1.05:
            speed_scaler = float(self.edit_speed.value()/100)
            speed_scaler = round(speed_scaler, 1)
            
            freqMove = int(constants.MAX_FREQ*speed_scaler)
            freqMove=freqMove.to_bytes(2, byteorder="little")
            # print(freqMove);
            arduino.write(freqMove)
            
        arduino.flush()
        posSavedMessage.exec_()
    
    def on_click_erase(self):
        controlByte = (7).to_bytes(1, byteorder="little")
        print(controlByte)
        
    def on_click_start(self):
        controlByte = (4).to_bytes(1, byteorder="little")
        mode = (4).to_bytes(1, byteorder="little")
        arduino.write(controlByte)
        arduino.write(mode) #movType
        # print(controlByte)
        # print(mode)
        if self.gotta_loop.isChecked()==True:
            # loop = (1).to_bytes(1, byteorder="little")
            arduino.write(loop)
        else:
            # loop = (0).to_bytes(1, byteorder="little")
            arduino.write(loop)
        # print(loop)
        arduino.flush()
        self.button_pause.setEnabled(True)
        self.button_stop.setEnabled(True)

        self.button_less_X.setEnabled(False)
        self.button_less_Y.setEnabled(False)
        self.button_less_Z.setEnabled(False)
        self.button_more_X.setEnabled(False)
        self.button_more_Y.setEnabled(False)
        self.button_more_Z.setEnabled(False)
        
        self.button_less_J1.setEnabled(False)
        self.button_less_J2.setEnabled(False)
        self.button_less_J3.setEnabled(False)
        self.button_more_J1.setEnabled(False)
        self.button_more_J2.setEnabled(False)
        self.button_more_J3.setEnabled(False)
        self.button_less_J4.setEnabled(False)
        self.button_less_J5.setEnabled(False)
        self.button_less_J6.setEnabled(False)
        self.button_more_J4.setEnabled(False)
        self.button_more_J5.setEnabled(False)
        self.button_more_J6.setEnabled(False)
        
        self.button_save_position.setEnabled(False)
        self.button_erase_position.setEnabled(False)
        
        self.initiating.exec_() 
        self.focusCamera.exec_()
        self.initialFocus = False
        self.focused = False
        print("started")
        
        self.button_start.setEnabled(False)
    
    def on_click_pause(self):
        controlByte = (5).to_bytes(1, byteorder="little")
        arduino.write(controlByte)
        if self.isPaused==False:
            arduino.write((1).to_bytes(1, byteorder="little"))
            self.isPaused=True
        else:
            arduino.write((0).to_bytes(1, byteorder="little"))
            self.isPaused=False

        
        
    def on_click_stop(self):
        controlByte = (6).to_bytes(1, byteorder="little")
        arduino.write(controlByte)

        self.button_pause.setEnabled(False)
        self.button_stop.setEnabled(False)

        self.button_less_X.setEnabled(True)
        self.button_less_Y.setEnabled(True)
        self.button_less_Z.setEnabled(True)
        self.button_more_X.setEnabled(True)
        self.button_more_Y.setEnabled(True)
        self.button_more_Z.setEnabled(True)
        
        self.button_less_J1.setEnabled(True)
        self.button_less_J2.setEnabled(True)
        self.button_less_J3.setEnabled(True)
        self.button_more_J1.setEnabled(True)
        self.button_more_J2.setEnabled(True)
        self.button_more_J3.setEnabled(True)
        self.button_less_J4.setEnabled(True)
        self.button_less_J5.setEnabled(True)
        self.button_less_J6.setEnabled(True)
        self.button_more_J4.setEnabled(True)
        self.button_more_J5.setEnabled(True)
        self.button_more_J6.setEnabled(True)
        
        self.button_save_position.setEnabled(True)
        self.button_erase_position.setEnabled(True)
        self.button_start.setEnabled(True)
            

class myThread(QThread):
    videoFeed = pyqtSignal(QImage)
    def __init__(self, bbox):
        super(myThread, self).__init__()
        self.bbox=bbox
    def run(self):
        self.isActive=True
        self.isFollowing=False
        self.wasActiveM5=False
        self.wasActiveM6=False
        tracker = cv2.TrackerKCF_create()
        self.feed = cv2.VideoCapture(0)
        ret, frame = self.feed.read()
        initial=False
        fps = 0.0
        new_time = 0.0
        old_time = 0.0
        while self.isActive:
            ret, frame = self.feed.read()
            if initial is False and ret is True and self.bbox is not None:
                frame = cv2.resize(frame, (320,240))
                tracker.init(frame, self.bbox)
                success, container = tracker.update(frame)
                initial=True
            if ret:
                new_time = time.time()
                try:
                    fps = round(1/(new_time-old_time))
                except:
                    continue
                old_time=new_time
                frame = cv2.resize(frame, (320,240))
                oneFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                success, container = tracker.update(frame)
                if success:
                    p1 = (int(container[0]), int(container[1]))
                    p2 = (int(container[0]+self.bbox[2]), int(container[1]+container[3]))
                    p3 = (int(container[0]+self.bbox[2]/2), int(container[1]+container[3]/2))
                    cv2.circle(oneFrame, p3, 1, (255,0,0), thickness=2)
                    cv2.rectangle(oneFrame, p1, p2, (255,0,0), 2, 1)
                    cv2.putText(oneFrame, "FPS : " + str(fps), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,170,50),2)
                    resultImage = QImage(oneFrame.data, oneFrame.shape[1], oneFrame.shape[0], QImage.Format_RGB888)
                    self.videoFeed.emit(resultImage)
                    if self.isFollowing==True:
                        print(p3[0])
                        if p3[0]<30:
                            # print("TOO FAR LEFT")
                            arduino.write((5).to_bytes(1, byteorder="little"))
                            arduino.write((1).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM5=True
                        elif p3[0]>290:
                            # print("TOO FAR RIGHT")
                            arduino.write((5).to_bytes(1, byteorder="little"))
                            arduino.write((-1).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM5=True
                        elif self.wasActiveM5 and p3[0]>30 and p3[0]<290:
                            arduino.write((5).to_bytes(1, byteorder="little"))
                            arduino.write((0).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM5=False

                        if p3[1]<30:
                            # print("TOO FAR UP")
                            arduino.write((6).to_bytes(1, byteorder="little"))
                            arduino.write(-(1).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM6=True
                        elif p3[1]>210:
                            # print("TOO FAR DOWN")
                            arduino.write((6).to_bytes(1, byteorder="little"))
                            arduino.write((1).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM6=True
                        elif self.wasActiveM6 and p3[1]>30 and p3[1]<290:
                            arduino.write((6).to_bytes(1, byteorder="little"))
                            arduino.write((0).to_bytes(1, byteorder="little", signed=True))
                            self.wasActiveM6=False
                else:
                    oneFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    cv2.putText(oneFrame, "FPS : " + str(fps), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,170,50),2)
                    cv2.putText(oneFrame, "Tracking failure", (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255),2)
                    resultImage = QImage(oneFrame.data, oneFrame.shape[1], oneFrame.shape[0], QImage.Format_RGB888)
                    self.videoFeed.emit(resultImage)       

    def stop(self):
        self.isActive=False
        self.feed.release()
    
    def beginTrack(self):
        self.isFollowing=True
    
    def stopTrack(self):
        self.isFollowing=False
        arduino.write((0).to_bytes(2, byteorder="little"))
        arduino.flush()
            
                
            
            
        

if __name__ == '__main__':
    
    # arduino = serial.Serial('/dev/ttyACM0', 115200)
    arduino = serial.Serial('COM3', 115200)    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
