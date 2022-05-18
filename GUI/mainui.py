import sys
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
        self.infolayout = QGridLayout()
        self.page_layout = QVBoxLayout()
        self.velocityBox1 = Velocity()
        self.velocityBox2 = Velocity()
        self.accelerationBox1 = Acceleration()
        self.accelerationBox2 = Acceleration()
        self.resetButton = QPushButton()

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
        self.graphWidget.setStyleSheet("border: 4px solid black;")
        pen = pg.mkPen(color=(0, 255, 255), width=3)
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen, symbol='o')

        # Dropdown Widget Setup
        self.dropdownBox.addItem("Crank Rocker")
        self.dropdownBox.addItem("Double Crank")
        self.dropdownBox.addItem("Double Rocker")
        self.dropdownBox.currentIndexChanged.connect(self.selectionChange)

        # Header Widget Setup
        self.sensorReadH.setText("ค่าที่อ่านได้จากเซ็นเซอร์")
        self.link2H.setText("Link 2")
        self.link4H.setText("Link 4")

        # Button Setup
        self.resetButton.setText("Reset")
        self.resetButton.move(64,32)
        self.resetButton.clicked.connect(self.resetEvent)

        # Page Construction
        self.graphBox.addWidget(self.graphTitle)
        self.graphBox.addWidget(self.graphWidget)
        self.visualizeBox.addWidget(self.dropdownBox)
        self.visualizeBox.addLayout(self.graphBox)
        self.page_layout.addLayout(self.visualizeBox)
        self.page_layout.addLayout(self.serialBox)
        self.page_layout.addWidget(self.sensorReadH)
        self.page_layout.addLayout(self.infolayout)
        self.infolayout.setContentsMargins(10,10,10,10)
        self.infolayout.addWidget(self.link2H, 0,0, alignment=Qt.AlignTop)
        self.infolayout.addLayout(self.velocityBox1.velocityBox, 0, 1)
        self.infolayout.addLayout(self.velocityBox2.velocityBox, 1, 1)
        self.infolayout.addWidget(self.link4H, 1,0, alignment=Qt.AlignTop)
        self.infolayout.addLayout(self.accelerationBox1.accBox, 0, 2)
        self.infolayout.addLayout(self.accelerationBox2.accBox, 1, 2)
        self.infolayout.addWidget(self.resetButton,2 ,1, alignment=Qt.AlignCenter)
        self.infolayout.columnStretch(2)
        widget.setLayout(self.page_layout)
        self.setCentralWidget(widget)
        exit_action = QAction('EXIT', self)
        exit_action.triggered.connect(self.closeEvent)
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
                
        # คำนวนความเร็ว #
        self.time_current = round(time.time())
        for i in range(3):
            self.dv1[i] = (self.aXYZ_1[i]-self.aXYZ_offset_1[i]) * (self.time_current - self.prev_time) 
            self.v1[i] = round((self.v1[i] + self.dv1[i]),2)
            self.dv2[i] = (self.aXYZ_2[i]-self.aXYZ_offset_2[i]) * (self.time_current - self.prev_time)
            self.v2[i] = round((self.v2[i] + self.dv2[i]),2)
        self.prev_time = self.time_current

        self.deg1 = self.gYPR_1[0] + self.defaultDegParam[self.jointsCalculator.mode][0] - self.gyroOffset1
        self.deg2 = self.gYPR_2[0] + self.defaultDegParam[self.jointsCalculator.mode][1] - self.gyroOffset2

        self.speedLink2 = sqrt(self.v1[0]**2 + self.v1[2]**2)
        self.speedLink4 = sqrt(self.v2[0]**2 + self.v2[2]**2)

        self.velocityBox1.vM.setText(str(self.speedLink2))
        self.velocityBox1.vX.setText(str(self.v1[0]))
        self.velocityBox1.vY.setText(str(self.v1[1]))
        self.velocityBox1.vZ.setText(str(self.v1[2]))
        self.accelerationBox1.aX.setText(str(self.aXYZ_1[0]))
        self.accelerationBox1.aY.setText(str(self.aXYZ_1[1]))
        self.accelerationBox1.aZ.setText(str(self.aXYZ_1[2]))
        self.velocityBox2.vM.setText(str(self.speedLink4))
        self.velocityBox2.vX.setText(str(self.v2[0]))
        self.velocityBox2.vY.setText(str(self.v2[1]))
        self.velocityBox2.vZ.setText(str(self.v2[2]))
        self.accelerationBox2.aX.setText(str(self.aXYZ_2[0]))
        self.accelerationBox2.aY.setText(str(self.aXYZ_2[1]))
        self.accelerationBox2.aZ.setText(str(self.aXYZ_2[2]))

        # นำค่า Gyro (Pitch) มาวาดแขน #
        self.x,self.y=self.jointsCalculator.calculateLinks(self.deg1)  # ใช้ค่าของ Gyro แล้วคำนวนองศาแขนอีกข้างเอง
        #self.x,self.y=self.jointsCalculator.drawFromBothDegree(self.deg1, self.deg2)  # วาดแขนจากองศาของ Gyro ทั้ง 2 ตัว
        self.data_line.setData(self.x, self.y)

    def selectionChange(self, i):
        if i != 0:
            self.graphWidget.setXRange(-0.2,0.3)
        else:
            self.graphWidget.setXRange(-0.1,0.3)
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

        self.gyroOffset1 = self.gYPR_1[0] # real gyro value
        self.gyroOffset2 = self.gYPR_2[0]  # real gyro value
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


class Velocity(QWidget):
    def __init__(self):
        super(Velocity, self).__init__()
        self.velo = QLabel()
        self.m = QLabel()
        self.x = QLabel()
        self.y = QLabel()
        self.z = QLabel()
        self.unit0 = QLabel()
        self.unit1 = QLabel()
        self.unit2 = QLabel()
        self.unit3 = QLabel()
        self.velocityBox = QVBoxLayout()
        self.velocityGrid = QGridLayout()
        self.vX = QLabel()
        self.vY = QLabel()
        self.vZ = QLabel()
        self.vM = QLabel()

        self.velo.setText("Velocity")   
        self.m.setText("Magnitute:")  
        self.x.setText("X:")  
        self.y.setText("Y:")
        self.z.setText("Z:")
        self.unit0.setText("m/s")
        self.unit1.setText("m/s")
        self.unit2.setText("m/s")
        self.unit3.setText("m/s")
        self.vM.setText("0.0")
        self.vX.setText("0.0")
        self.vY.setText("0.0")
        self.vZ.setText("0.0")
        self.velocityBox.addWidget(self.velo)
        self.velocityGrid.addWidget(self.m, 0, 0)
        self.velocityGrid.addWidget(self.x, 1, 0)
        self.velocityGrid.addWidget(self.y, 2, 0)
        self.velocityGrid.addWidget(self.z, 3, 0)
        self.velocityGrid.addWidget(self.vM, 0, 1)
        self.velocityGrid.addWidget(self.vX, 1, 1)
        self.velocityGrid.addWidget(self.vY, 2, 1)
        self.velocityGrid.addWidget(self.vZ, 3, 1)
        self.velocityGrid.addWidget(self.unit0, 0, 2)
        self.velocityGrid.addWidget(self.unit1, 1, 2)
        self.velocityGrid.addWidget(self.unit2, 2, 2)
        self.velocityGrid.addWidget(self.unit3, 3, 2)
        self.velocityBox.addLayout(self.velocityGrid)

class Acceleration(QWidget):
    def __init__(self):
        super(Acceleration, self).__init__()
        self.acc = QLabel()
        self.x = QLabel()
        self.y = QLabel()
        self.z = QLabel()
        self.unit1 = QLabel()
        self.unit2 = QLabel()
        self.unit3 = QLabel()
        self.accBox = QVBoxLayout()
        self.accGrid = QGridLayout()
        self.aX = QLabel()
        self.aY = QLabel()
        self.aZ = QLabel()

        self.acc.setText("Accelerometer")
        self.x.setText("X:")
        self.y.setText("Y:")
        self.z.setText("Z:")
        self.unit1.setText("m/s^2")
        self.unit2.setText("m/s^2")
        self.unit3.setText("m/s^2")
        self.aX.setText("0.0")
        self.aY.setText("0.0")
        self.aZ.setText("0.0")
        self.accBox.addWidget(self.acc)
        self.accGrid.addWidget(self.x, 0, 0)
        self.accGrid.addWidget(self.y, 1, 0)
        self.accGrid.addWidget(self.z, 2, 0)
        self.accGrid.addWidget(self.aX, 0, 1)
        self.accGrid.addWidget(self.aY, 1, 1)
        self.accGrid.addWidget(self.aZ, 2, 1)
        self.accGrid.addWidget(self.unit1, 0, 2)
        self.accGrid.addWidget(self.unit2, 1, 2)
        self.accGrid.addWidget(self.unit3, 2, 2)
        self.accBox.addLayout(self.accGrid)

app = QApplication(sys.argv)
w = MainWindow()
app.exec_()