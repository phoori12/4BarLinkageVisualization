import sys
import numpy as np
from numpy import degrees, pi, sin, cos, sqrt, absolute, arccos, arctan, sign
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from Fbarequations import FBarEquations


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("IMU Project")
        #self.setFixedSize(self.size())

        #################### Class Variable Initialization ####################
        self.defDeg = 90
        self.i = self.defDeg
        self.x = []
        self.y = []
        self.gyroOffset1 = 0
        self.gyroOffset2 = 0
        # input parameters
        self.link1 = 0.12 # d center-to-center distance
        self.link2 = [0.09, 0.14, 0.19] # r
        self.link3 = [0.19,0.18, 0.12] # l
        self.link4 = [0.18, 0.19, 0.14] # rr
        self.jointsCalculator = FBarEquations(self.link1, self.link2, self.link3, self.link4)
        # Variable for gyro forced resets
        self.defaultDegParam = [[],[],[]]
        self.jointsCalculator.mode = 1
        self.defaultDegParam[1] = self.jointsCalculator.calculateLinks(90, 1)
        self.jointsCalculator.mode = 2
        self.defaultDegParam[2] = self.jointsCalculator.calculateLinks(90, 1)
        self.jointsCalculator.mode = 0
        self.defaultDegParam[0] = self.jointsCalculator.calculateLinks(90, 1)
        #######################################################################
        #print(self.defaultDegParam)

        
        #################### Window's Widgets Initialization ####################
        widget = QWidget()
        self.graphWidget = pg.PlotWidget()
        self.dropdownBox = QComboBox()
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
        # QTGRAPH Widget Setup
        self.graphTitle.setText("จอแสดงผลการเคลื่อนที่ของก้านโยง")
        self.graphTitle.setAlignment(Qt.AlignCenter)
        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(-0.1,0.3)
        self.graphWidget.setYRange(-0.5,0.5)
        self.graphWidget.setStyleSheet("border: 4px solid black;")
        pen = pg.mkPen(color=(0, 255, 255), width=3)
        #self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)
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
        #######################################################################

        self.x,self.y = self.jointsCalculator.calculateLinks(self.i)
        self.update_plot()

        self.show()
        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        # fetch from arduino v and a 
        v1X = 1.0
        v1Y = 2.0
        v1Z = 3.0
        a1X = 4.0
        a1Y = 5.0
        a1Z = 6.0

        v2X = 7.0
        v2Y = 8.0
        v2Z = 9.0
        a2X = 1.1
        a2Y = 1.2
        a2Z = 1.3

        self.velocityBox1.vX.setText(str(v1X))
        self.velocityBox1.vY.setText(str(v1Y))
        self.velocityBox1.vZ.setText(str(v1Z))

        self.accelerationBox1.aX.setText(str(a1X))
        self.accelerationBox1.aY.setText(str(a1Y))
        self.accelerationBox1.aZ.setText(str(a1Z))

        self.velocityBox2.vX.setText(str(v2X))
        self.velocityBox2.vY.setText(str(v2Y))
        self.velocityBox2.vZ.setText(str(v2Z))

        self.accelerationBox2.aX.setText(str(a2X))
        self.accelerationBox2.aY.setText(str(a2Y))
        self.accelerationBox2.aZ.setText(str(a2Z))
        # calculate joints
        # TODO: Calculate
        self.x,self.y=self.jointsCalculator.drawFromBothDegree(self.defaultDegParam[self.jointsCalculator.mode][0], self.defaultDegParam[self.jointsCalculator.mode][1])
        self.data_line.setData(self.x, self.y)
        #self.i = self.i + 10

    def selectionChange(self, i):
        if i != 0:
            self.graphWidget.setXRange(-0.2,0.3)
        else:
            self.graphWidget.setXRange(-0.1,0.3)
        self.jointsCalculator.mode = i
        self.resetEvent()
        # self.x,self.y=self.jointsCalculator.calculateLinks(self.i)

    def resetEvent(self):
        # reset gyro back to default position
        self.x,self.y=self.jointsCalculator.calculateLinks(self.defDeg)
        # current gyro-val = gyro-offset



class Velocity(QWidget):
    def __init__(self):
        super(Velocity, self).__init__()
        self.velo = QLabel()
        self.x = QLabel()
        self.y = QLabel()
        self.z = QLabel()
        self.unit1 = QLabel()
        self.unit2 = QLabel()
        self.unit3 = QLabel()
        self.velocityBox = QVBoxLayout()
        self.velocityGrid = QGridLayout()
        self.vX = QLabel()
        self.vY = QLabel()
        self.vZ = QLabel()

        self.velo.setText("Velocity")   
        self.x.setText("X:")  
        self.y.setText("Y:")
        self.z.setText("Z:")
        self.unit1.setText("m/s")
        self.unit2.setText("m/s")
        self.unit3.setText("m/s")
        self.vX.setText("0.0")
        self.vY.setText("0.0")
        self.vZ.setText("0.0")
        self.velocityBox.addWidget(self.velo)
        self.velocityGrid.addWidget(self.x, 0, 0)
        self.velocityGrid.addWidget(self.y, 1, 0)
        self.velocityGrid.addWidget(self.z, 2, 0)
        self.velocityGrid.addWidget(self.vX, 0, 1)
        self.velocityGrid.addWidget(self.vY, 1, 1)
        self.velocityGrid.addWidget(self.vZ, 2, 1)
        self.velocityGrid.addWidget(self.unit1, 0, 2)
        self.velocityGrid.addWidget(self.unit2, 1, 2)
        self.velocityGrid.addWidget(self.unit3, 2, 2)
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