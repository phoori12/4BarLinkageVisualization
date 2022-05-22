from math import radians
import sys
import os
import time
import numpy as np
from numpy import degrees, pi, sin, cos, sqrt, absolute, arccos, arctan, sign
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from Fbarequations import FBarEquations
from serialcomms import SerialComms
import csv
from datetime import datetime

class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("4Bar Linkage Visualization")

        #################### Class Variable Initialization ####################
        self.isConnected = False
        self.flag = 0
        self.defDeg = 90
        self.i = self.defDeg
        self.x = []
        self.y = []
        self.deg1 = 0
        self.deg2 = 0
        self.prev_deg4 = 0
        self.omega4 = 0
        self.vR4 = 0
        self.prev_vR4 = 0
        self.aR4 = 0
        self.gyro1 = 0
        self.gyro2 = 0
        self.gyroOffset1 = 0
        self.gyroOffset2 = 0
        # input parameters
        self.link1 = 0.12 # d center-to-center distance
        self.link2 = [0.09, 0.14, 0.19] # r
        self.link3 = [0.19,0.18, 0.12] # l
        self.link4 = [0.18, 0.19, 0.14] # rr
        self.jointsCalculator = FBarEquations(self.link1, self.link2, self.link3, self.link4)
        self.serialComm = SerialComms()
        # Variable for gyro forced resets
        self.defaultDegParam = [[],[],[]]
        self.jointsCalculator.mode = 1
        self.defaultDegParam[1] = self.jointsCalculator.calculateLinks(90, 1) # องศา default ของโหมด 1 (Double Crank)
        self.jointsCalculator.mode = 2
        self.defaultDegParam[2] = self.jointsCalculator.calculateLinks(90, 1) # องศา default ของโหมด 2 (Double Rocker)
        self.jointsCalculator.mode = 0
        self.defaultDegParam[0] = self.jointsCalculator.calculateLinks(90, 1) # องศา default ของโหมด 0 (Crank Rocker)
        # Velocity calculation variables
        self.speedLink2 = 0.0 # XZ
        self.speedLink4 = 0.0
        self.accelLink2 = 0.0
        self.accelLink4 = 0.0
        self.time_current = 0
        self.prev_time = 0
        self.dv1 = [0.0, 0.0, 0.0]
        self.v1 = [0.0, 0.0, 0.0]
        self.dv2 = [0.0, 0.0, 0.0]
        self.v2 = [0.0, 0.0, 0.0]
        self.gYPR_1 = [0.0, 0.0, 0.0]
        self.aXYZ_1 = [0.0, 0.0, 0.0]
        self.aXYZ_offset_1 = [0.0, 0.0, 0.0]
        self.gYPR_2 = [0.0, 0.0, 0.0]
        self.aXYZ_2 = [0.0, 0.0, 0.0]
        self.aXYZ_offset_2 = [0.0, 0.0, 0.0]
        # Logging Variable
        self.LogState = False
        self.dirname = os.path.dirname(__file__)
        self.date_time = datetime.fromtimestamp(datetime.timestamp(datetime.now()))
        self.str_date_time = self.date_time.strftime("%d-%m-%Y:%H:%M:%S:%f")
        self.main_direc = os.path.join(self.dirname, 'logging')
        self.logTime = 0

        self.fileName = None
        self.csvFile= None
        self.writer = None
        self.LogDict = {
            "Timestamp": self.str_date_time,
            "Gyro1":"0.0",
            "Vx1":"0.0",
            "Vy1":"0.0",
            "Vz1":"0.0",
            "Vm1":"0.0",
            "Ax1":"0.0",
            "Ay1":"0.0",
            "Az1":"0.0",
            "Gyro2":"0.0",
            "Vx2":"0.0",
            "Vy2":"0.0",
            "Vz2":"0.0",
            "Vm2":"0.0",
            "Ax2":"0.0",
            "Ay2":"0.0",
            "Az2":"0.0",
            "TheoV":"0.0",
            "TheoA":"0.0"
        }
        #######################################################################

        
        #################### Window's Widgets Initialization ####################
        widget = QWidget()
        self.graphWidget = pg.PlotWidget()
        self.dropdownBox = QComboBox()
        self.serialBox = QHBoxLayout()
        self.serialPortsList = QComboBox()
        self.serialConnect = QPushButton()
        self.serialDisconnect = QPushButton()
        self.serialRefresh = QPushButton()
        self.graphTitle = QLabel()
        self.visualizeBox = QHBoxLayout()
        self.graphBox = QVBoxLayout()
        self.sensorReadH = QLabel()
        self.link2H = QLabel()
        self.link4H = QLabel()
        self.link4TH = QLabel()
        self.infolayout = QGridLayout()
        self.page_layout = QVBoxLayout()
        self.velocityBox1 = Velocity()
        self.velocityBox2 = Velocity()
        self.theoVelocityBox1 = Velocity()
        self.accelerationBox1 = Acceleration()
        self.accelerationBox2 = Acceleration()
        self.theoAccelerationBox1 = Acceleration()
        self.resetButton = QPushButton()
        self.startLoggingButton = QPushButton()
        self.stopLoggingButton = QPushButton()

        # Serial UInterface Setup
        self.serialConnect.setText("Connect")
        self.serialConnect.move(64,32)
        self.serialConnect.clicked.connect(self.serialConnectEvent)
        self.serialDisconnect.setText("Disconnect")
        self.serialDisconnect.move(64,32)
        self.serialDisconnect.clicked.connect(self.serialDisconnectEvent)
        self.serialRefresh.setText("Refresh")
        self.serialRefresh.move(64,32)
        self.serialRefresh.clicked.connect(self.serialRefreshEvent)
        serialPorts = self.serialComm.listPorts()
        for s in serialPorts:
            self.serialPortsList.addItem(s)
        self.serialPortsList.currentIndexChanged.connect(self.serialSelectionChange)
        self.serialComm.ser.port = self.serialPortsList.itemText(0)
        self.serialBox.addWidget(self.serialPortsList)
        self.serialBox.addWidget(self.serialRefresh)
        self.serialBox.addWidget(self.serialConnect)
        self.serialBox.addWidget(self.serialDisconnect)
        
        # QTGRAPH Widget Setup
        self.graphTitle.setText("จอแสดงผลการเคลื่อนที่ของก้านโยง")
        self.graphTitle.setAlignment(Qt.AlignCenter)
        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(-0.1,0.3)
        self.graphWidget.setYRange(-0.5,0.5)
        # self.graphWidget.setXRange(-1,1)
        # self.graphWidget.setYRange(-1,1)
        self.graphWidget.setStyleSheet("border: 4px solid black;")
        pen = pg.mkPen(color=(0, 255, 255), width=3)
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen, symbol='o')
        self.graphWidget.setAspectLocked(True)
        self.graphWidget.showGrid(x=True, y=True)

        # Dropdown Widget Setup
        self.dropdownBox.addItem("Crank Rocker")
        self.dropdownBox.addItem("Double Crank")
        self.dropdownBox.addItem("Double Rocker")
        self.dropdownBox.currentIndexChanged.connect(self.selectionChange)

        # Header Widget Setup
        self.sensorReadH.setText("ค่าที่อ่านได้จากเซ็นเซอร์")
        self.link2H.setText("Link 2")
        self.link4H.setText("Link 4")
        self.link4TH.setText("Link 4 (Theory)")

        # Button Setup
        self.resetButton.setText("Reset")
        self.resetButton.move(64,32)
        self.resetButton.clicked.connect(self.resetEvent)

        # Start Button Setup
        self.startLoggingButton.setText("Start Logging")
        self.startLoggingButton.move(64,32)
        self.startLoggingButton.clicked.connect(self.startLogEvent)

        # Stop Button Setup
        self.stopLoggingButton.setText("Stop Logging")
        self.stopLoggingButton.move(64,32)
        self.stopLoggingButton.clicked.connect(self.stopLogEvent)

        # Logging Box and Button Setup
        self.loggingBox = QGridLayout()
        self.loggingBox.addWidget(self.sensorReadH, 0,1)
        self.loggingBox.addWidget(self.startLoggingButton, 0,2)
        self.loggingBox.addWidget(self.stopLoggingButton, 0,3)

        # Page Construction
        self.graphBox.addWidget(self.graphTitle)
        self.graphBox.addWidget(self.graphWidget)
        self.visualizeBox.addWidget(self.dropdownBox)
        self.visualizeBox.addLayout(self.graphBox)
        self.page_layout.addLayout(self.visualizeBox)
        self.page_layout.addLayout(self.serialBox)
        self.page_layout.addLayout(self.loggingBox)
        self.page_layout.addLayout(self.infolayout)
        self.infolayout.setContentsMargins(10,10,10,10)
        self.infolayout.addWidget(self.link2H, 0,0, alignment=Qt.AlignTop)
        self.infolayout.addLayout(self.velocityBox1.velocityBox, 0, 1)
        self.infolayout.addLayout(self.velocityBox2.velocityBox, 1, 1)
        self.infolayout.addWidget(self.link4H, 1,0, alignment=Qt.AlignTop)
        self.infolayout.addLayout(self.accelerationBox1.accBox, 0, 2)
        self.infolayout.addLayout(self.accelerationBox2.accBox, 1, 2)
        self.infolayout.addWidget(self.link4TH, 2,0, alignment=Qt.AlignTop)
        self.infolayout.addLayout(self.theoVelocityBox1.velocityBox, 2, 1)
        self.infolayout.addLayout(self.theoAccelerationBox1.accBox, 2, 2)
        self.infolayout.addWidget(self.resetButton,3 ,1, alignment=Qt.AlignCenter)
        self.infolayout.columnStretch(2)
        widget.setLayout(self.page_layout)
        self.setCentralWidget(widget)
        exit_action = QAction('EXIT', self)
        exit_action.triggered.connect(self.closeEvent)
        #self.iii = 0
        #######################################################################

        self.x,self.y = self.jointsCalculator.calculateLinks(self.i)
        self.time_current = round(time.time() * 1000)
        self.prev_time = self.time_current
        self.update_plot()

        self.show()
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()


    def update_plot(self):
        if self.isConnected:
            _buffer = self.serialComm.getBuffer(2)
            if _buffer != 0:
                buffer = self.serialComm.castBuffer(_buffer)
                self.gYPR_1[0] = buffer[3]
                self.gYPR_1[1] = buffer[4]
                self.gYPR_1[2] = buffer[5]
                self.aXYZ_1[0] = buffer[0]
                self.aXYZ_1[1] = buffer[1]
                self.aXYZ_1[2] = buffer[2]
                self.gYPR_2[0] = buffer[9]
                self.gYPR_2[1] = buffer[10]
                self.gYPR_2[2] = buffer[11]
                self.aXYZ_2[0] = buffer[6]
                self.aXYZ_2[1] = buffer[7]
                self.aXYZ_2[2] = buffer[8]
                
        self.deg1 = self.gYPR_1[0] + self.defaultDegParam[self.jointsCalculator.mode][0] - self.gyroOffset1
        self.deg2 = self.gYPR_2[0] + self.defaultDegParam[self.jointsCalculator.mode][1] - self.gyroOffset2
        #print(self.gYPR_1[2])

        # คำนวนความเร็ว #
        calDeg = self.jointsCalculator.calculateLinks(self.deg1, 1) # self.deg1
        self.time_current = round(time.time(),3)
        for i in range(3):
            self.dv1[i] = (self.aXYZ_1[i]-self.aXYZ_offset_1[i]) * (self.time_current - self.prev_time) 
            self.v1[i] = self.v1[i] + self.dv1[i]
            self.dv2[i] = (self.aXYZ_2[i]-self.aXYZ_offset_2[i]) * (self.time_current - self.prev_time)
            self.v2[i] = self.v2[i] + self.dv2[i]
        
        self.omega4 = (radians(calDeg[1]) - radians(self.prev_deg4)) / (self.time_current - self.prev_time)
        self.vR4 = round((self.omega4 * self.link2[self.jointsCalculator.mode]), 3)
        self.aR4 = round(((self.vR4 - self.prev_vR4) / (self.time_current - self.prev_time)), 3)
        self.speedLink2 = round((sqrt(self.v1[0]**2 + self.v1[1]**2)),3)
        self.speedLink4 = round((sqrt(self.v2[0]**2 + self.v2[1]**2)),3)
        self.accelLink2 = round((sqrt(self.aXYZ_1[0]**2 + self.aXYZ_1[1]**2 + self.aXYZ_1[2]**2)),3)
        self.accelLink4 = round((sqrt(self.aXYZ_2[0]**2 + self.aXYZ_2[1]**2 + self.aXYZ_2[2]**2)),3)
        #print(self.speedLink2)
        self.theoVelocityBox1.vM.setText(str(self.vR4))
        self.theoAccelerationBox1.aM.setText(str(self.aR4))
        self.velocityBox1.vM.setText(str(self.speedLink2))
        self.accelerationBox1.aM.setText(str(self.accelLink2))
        self.velocityBox2.vM.setText(str(self.speedLink4))
        self.accelerationBox2.aM.setText(str(self.accelLink4))
        
        # นำค่า Gyro (Pitch) มาวาดแขน #
        self.x,self.y=self.jointsCalculator.calculateLinks(self.deg1)  # ใช้ค่าของ Gyro แล้วคำนวนองศาแขนอีกข้างเอง
        #self.x,self.y=self.jointsCalculator.drawFromBothDegree(self.deg1, self.deg2)  # วาดแขนจากองศาของ Gyro ทั้ง 2 ตัว
        self.data_line.setData(self.x, self.y)

        self.date_time = datetime.fromtimestamp(datetime.timestamp(datetime.now()))
        self.str_date_time = self.date_time.strftime("%d-%m-%Y:%H:%M:%S:%f")
        self.LogDict = {
            "Timestamp": self.str_date_time,
            "Gyro1":str(round(self.deg1, 3)),
            "Vx1": str(round(self.v1[0],3)),
            "Vy1": str(round(self.v1[1],3)),
            "Vz1": str(round(self.v1[2],3)),
            "Vm1":str(self.speedLink2),
            "Ax1":str(round(self.aXYZ_1[0],3)),
            "Ay1":str(round(self.aXYZ_1[1],3)),
            "Az1":str(round(self.aXYZ_1[2],3)),
            "Gyro2":str(round(self.deg2,3)),
            "Vx2":str(round(self.v2[0],3)),
            "Vy2":str(round(self.v2[1],3)),
            "Vz2":str(round(self.v2[2],3)),
            "Vm2":str(self.speedLink4),
            "Ax2":str(round(self.aXYZ_2[0],3)),
            "Ay2":str(round(self.aXYZ_2[1],3)),
            "Az2":str(round(self.aXYZ_2[2],3)),
            "TheoV":str(self.vR4),
            "TheoA":str(self.aR4)
        }
        if self.LogState and (self.time_current - self.logTime >= 1):
            self.event_handler_values_update(self.LogDict, self.writer)
            self.logTime = self.time_current
        
        self.prev_deg4 = calDeg[1]
        self.prev_vR4 = self.vR4
        self.prev_time = self.time_current

        #self.iii += 1


    def selectionChange(self, i):
        if i != 0:
            self.graphWidget.setXRange(-0.2,0.3)
            self.graphWidget.setYRange(-0.5,0.5)
        else:
            self.graphWidget.setXRange(-0.1,0.3)
            self.graphWidget.setYRange(-0.5,0.5)
        self.jointsCalculator.mode = i
        self.resetEvent()

    def resetEvent(self):
        # RESET ค่่า Gyro กับความเร็ว
        for i in range(3):
            self.dv1[i] = 0
            self.v1[i] = 0
            self.dv2[i] = 0
            self.v2[i] = 0
            self.aXYZ_offset_1[i] = self.aXYZ_1[i]
            self.aXYZ_offset_2[i] = self.aXYZ_2[i]

        self.omega4 = 0
        self.vR4 = 0
        self.aR4 = 0
        self.speedLink2 = 0
        self.speedLink4 = 0
        self.accelLink2 = 0
        self.accelLink4 = 0
        self.prev_deg4 = 0
        self.prev_vR4 = 0
        self.gyroOffset1 = self.gYPR_1[0] # real gyro value
        self.gyroOffset2 = self.gYPR_2[0]  # real gyro value
        self.prev_time = self.time_current = round(time.time(),3)
        self.x,self.y=self.jointsCalculator.drawFromBothDegree(self.defaultDegParam[self.jointsCalculator.mode][0], self.defaultDegParam[self.jointsCalculator.mode][1]) # Set มุมต่างๆกลับเป็น Default และวาด link
        self.data_line.setData(self.x, self.y)

    def serialRefreshEvent(self):
        self.serialPortsList.clear()
        serialPorts = self.serialComm.listPorts()
        for s in serialPorts:
            self.serialPortsList.addItem(s)
        self.serialPortsList.currentIndexChanged.connect(self.serialSelectionChange)
        self.serialComm.ser.port = self.serialPortsList.itemText(0)

    def serialConnectEvent(self):
        msg = QMessageBox()
        msg.setStandardButtons(QMessageBox.Ok)
        if self.flag == 0:
            self.flag = self.serialComm.connect()
            if self.flag == 0: # Cannot connect
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Connection fail")
            elif self.flag == 1: # Connected
                msg.setIcon(QMessageBox.Information)
                msg.setText("Connected!")
                self.isConnected = True
        elif self.flag == 1:
            msg.setIcon(QMessageBox.Question)
            msg.setText("Already Connected to " + self.serialComm.ser.port)
        msg.exec_()

    def serialDisconnectEvent(self):
        msg = QMessageBox()
        msg.setStandardButtons(QMessageBox.Ok)
        if self.flag == 1: # Device is connected
            self.flag = self.serialComm.disconnect()
            if self.flag == 2: # Cannot Disconect
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Disconnection fail")
                self.flag = 1
            elif self.flag == 1: # Disconnected
                msg.setIcon(QMessageBox.Information)
                msg.setText("Disconnected!")
                self.isConnected = False
                self.flag = 0
        elif self.flag == 0:
            msg.setIcon(QMessageBox.Question)
            msg.setText("No device connected")
        msg.exec_()

    def serialSelectionChange(self, i):
        if self.flag == 0:
            self.serialComm.ser.port = self.serialPortsList.itemText(i)
        else:
            msg = QMessageBox()
            msg.setStandardButtons(QMessageBox.Ok)
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Please choose Port after successful disconnection")
            msg.exec_()

    def closeEvent(self, event):
        self.serialComm.disconnect()
        if self.LogState:
            self.LogState = False
            self.stopLogEvent()

    def startLogEvent(self):
        msg = QMessageBox()
        msg.setStandardButtons(QMessageBox.Ok)
        if not self.LogState:
            self.LogState = True
            self.date_time = datetime.fromtimestamp(datetime.timestamp(datetime.now()))
            self.main_direc = os.path.join(self.dirname, 'logging')
            self.fileName = f'results{self.date_time.strftime("%d%m%Y%H%M%S")}.csv'
            self.fileName = os.path.join(self.main_direc, self.fileName)
            self.csvFile= open(self.fileName, 'w')
            self.writer = csv.writer(self.csvFile)
            msg.setIcon(QMessageBox.Information)
            msg.setText("Logging Start!")
        else:
            msg.setIcon(QMessageBox.Question)
            msg.setText("Logging has Already Started!")
        msg.exec_()
         

    def stopLogEvent(self):
        msg = QMessageBox()
        msg.setStandardButtons(QMessageBox.Ok)
        if self.LogState:
            self.LogState = False
            self.csvFile.close()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Logging Stop!")
        else:
            msg.setIcon(QMessageBox.Question)
            msg.setText("No Logging has been started")
        msg.exec_()
        

    def event_handler_values_update(self, logMsg, writer):
        writer.writerow([logMsg]) ##

class Velocity(QWidget):
    def __init__(self):
        super(Velocity, self).__init__()
        self.velo = QLabel()
        self.m = QLabel()
        self.unit = QLabel()
        self.velocityBox = QVBoxLayout()
        self.velocityGrid = QGridLayout()
        self.vM = QLabel()

        self.velo.setText("Velocity")   
        self.m.setText("Magnitute:")  
        self.unit.setText("m/s")
        self.vM.setText("0.0")
        self.velocityBox.addWidget(self.velo)
        self.velocityGrid.addWidget(self.m, 0, 0)
        self.velocityGrid.addWidget(self.vM, 0, 1)
        self.velocityGrid.addWidget(self.unit, 0, 2)
        self.velocityBox.addLayout(self.velocityGrid)

class Acceleration(QWidget):
    def __init__(self):
        super(Acceleration, self).__init__()
        self.acc = QLabel()
        self.m = QLabel()
        self.unit = QLabel()
        self.accBox = QVBoxLayout()
        self.accGrid = QGridLayout()
        self.aM = QLabel()

        self.acc.setText("Acceleration")
        self.m.setText("Magnitute:")
        self.unit.setText("m/s^2")
        self.aM.setText("0.0")
        self.accBox.addWidget(self.acc)
        self.accGrid.addWidget(self.m, 0, 0)
        self.accGrid.addWidget(self.aM, 0, 1)
        self.accGrid.addWidget(self.unit, 0, 2)
        self.accBox.addLayout(self.accGrid)



app = QApplication(sys.argv)
w = MainWindow()
app.exec_()