import sys
import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("IMU Project")
        #self.setFixedSize(self.size())

        #################### Class Variable Initialization ####################
        self.mode = 0
        self.i = 0
        self.x = []
        self.y = []
        # input parameters
        self.link1 = 0.12 # d center-to-center distance
        self.link2 = [0.09, 0.14, 0.19] # r
        self.link3 = [0.19,0.18, 0.12] # l
        self.link4 = [0.18, 0.19, 0.14] # rr
        self.rot_num = 6  # number of crank rotations
        self.increment = 0.1  # angle incremement
        self.over = 1  # if over = 1 --> mechanism is on top, If over = -1, mechanism on bottom
        self.s = self.over / absolute(self.over)
        # create the angle array, where the last angle is the number of rotations*2*pi
        self.angle_minus_last = np.arange(0, self.rot_num * 2 * pi, self.increment)
        self.R_Angles = np.append(self.angle_minus_last, self.rot_num * 2 * pi)
        # coordinates of the crank center point : Point 1
        self.x1 = 0
        self.y1 = 0
        # Coordinates of the rocker center point: Point 4
        self.x4 = self.link1
        self.y4 = 0
        self.X2 = np.zeros(len(self.R_Angles))  # array of crank x-positions: Point 2
        self.Y2 = np.zeros(len(self.R_Angles))  # array of crank y-positions: Point 2
        self.RR_Angle = np.zeros(len(self.R_Angles))  # array of rocker arm angles
        self.X3 = np.zeros(len(self.R_Angles))  # array of rocker x-positions: Point 3
        self.Y3 = np.zeros(len(self.R_Angles))  # array of rocker y-positions: Point 3
        #######################################################################
        
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

        self.calculateJoint()
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

        if self.i >= len(self.X2): self.i = 0
        self.x = [self.x1, self.X2[self.i], self.X3[self.i], self.x4]
        self.y = [self.y1, self.Y2[self.i], self.Y3[self.i], self.y4]
        self.data_line.setData(self.x, self.y)
        self.i = self.i + 1

    def selectionChange(self, i):
        if i != 0:
            self.graphWidget.setXRange(-0.2,0.3)
        else:
            self.graphWidget.setXRange(-0.1,0.3)
        self.mode = i
        self.calculateJoint()

    def resetEvent(self):
        # reset gyro back to default position
        print("shit")

    def calculateJoint(self):
        # find the crank and connecting rod positions for each angle
        for index, R_Angle in enumerate(self.R_Angles, start=0):
            #theta1 = np.radians(90)
            theta1 = R_Angle # gyro
            x2 = self.link2[self.mode] * cos(theta1)  # x-cooridnate of the crank: Point 2
            y2 = self.link2[self.mode] * sin(theta1)  # y-cooridnate of the crank: Point 2
            e = sqrt((x2 - self.link1) ** 2 + (y2 ** 2))
            phi2 = arccos((e ** 2 + self.link4[self.mode] ** 2 - self.link3[self.mode] ** 2) / (2 * e * self.link4[self.mode]))
            phi1 = arctan(y2 / (x2 - self.link1)) + (1 - sign(x2 - self.link1)) * pi / 2
            theta3 = phi1 - self.s * phi2 #gyro
            #theta3 = np.radians(90)
            self.RR_Angle[index] = theta3
            # if np.degrees(RR_Angle[index]) != None:
            #     print(print(np.degrees(RR_Angle[index])))
            x3 = self.link4[self.mode] * cos(theta3) + self.link1
            # x cooridnate of the rocker moving point: Point 3
            y3 = self.link4[self.mode] * sin(theta3)
            # y cooridnate of the rocker moving point: Point 3

            theta2 = arctan((y3 - y2) / (x3 - x2)) + (1 - sign(x3 - x2)) * pi / 2

            self.X2[index] = x2  # grab the crankshaft x-position
            self.Y2[index] = y2  # grab the crankshaft y-position
            self.X3[index] = x3  # grab the connecting rod x-position
            self.Y3[index] = y3  # grab the connecting rod y-position

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