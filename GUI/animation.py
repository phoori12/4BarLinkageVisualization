import sys
import random
import matplotlib
import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
matplotlib.use('Qt5Agg')

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib.pyplot as plt


class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = plt.figure()
        self.axes = fig.add_subplot(
            111, aspect="equal", autoscale_on=False, xlim=(-2, 6), ylim=(-2, 3)
        )
        
        super(MplCanvas, self).__init__(fig)


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setFixedSize(self.size())
        self.canvas = MplCanvas(self)
        (self.line, ) = self.canvas.axes.plot(
            [], [], "o-", lw=5, color="#2b8cbe"
        )  # color from: http://colorbrewer2.org/
        self.line.set_data([], [])
         # add grid lines, title and take out the axis tick labels
        self.canvas.axes.grid(alpha=0.5)
        self.canvas.axes.set_title("Crank and Rocker Motion")
        self.canvas.axes.set_xticklabels([])
        self.canvas.axes.set_yticklabels([])

        sensorReadH = QLabel()
        sensorReadH.setText("ค่าที่อ่านได้จากเซ็นเซอร์")
        link2H = QLabel()
        link2H.setText("Link 2")
        link4H = QLabel()
        link4H.setText("Link 4")

        infolayout = QGridLayout()
        page_layout = QVBoxLayout()
        velocityBox1 = Velocity()
        velocityBox2 = Velocity()
        accelerationBox1 = Acceleration()
        accelerationBox2 = Acceleration()


        page_layout.addWidget(self.canvas)
        page_layout.addWidget(sensorReadH)


        page_layout.addLayout(infolayout)
        infolayout.setContentsMargins(10,10,10,10)
        infolayout.addWidget(link2H, 0,0, alignment=Qt.AlignTop)
        infolayout.addLayout(velocityBox1.velocityBox, 0, 1)
        infolayout.addLayout(velocityBox2.velocityBox, 1, 1)
        infolayout.addWidget(link4H, 1,0, alignment=Qt.AlignTop)
        infolayout.addLayout(accelerationBox1.accBox, 0, 2)
        infolayout.addLayout(accelerationBox2.accBox, 1, 2)
        # infolayout.addLayout(velocityBox, 1, 0)
        infolayout.columnStretch(2)

        widget = QWidget()
        widget.setLayout(page_layout)
        self.setCentralWidget(widget)
        

        self.i = 0
        # input parameters
        self.r = 0.395  # crank radius
        self.l = 4.27  # connecting rod length
        self.rr = 1.15 # rocker radius
        self.d = 4.03  # center-to-center distance
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
        self.x4 = self.d
        self.y4 = 0
        self.X2 = np.zeros(len(self.R_Angles))  # array of crank x-positions: Point 2
        self.Y2 = np.zeros(len(self.R_Angles))  # array of crank y-positions: Point 2
        self.RR_Angle = np.zeros(len(self.R_Angles))  # array of rocker arm angles
        self.X3 = np.zeros(len(self.R_Angles))  # array of rocker x-positions: Point 3
        self.Y3 = np.zeros(len(self.R_Angles))  # array of rocker y-positions: Point 3

        # find the crank and connecting rod positions for each angle
        for index, R_Angle in enumerate(self.R_Angles, start=0):
            # theta1 = np.radians(300)
            theta1 = R_Angle # gyro
            x2 = self.r * cos(theta1)  # x-cooridnate of the crank: Point 2
            y2 = self.r * sin(theta1)  # y-cooridnate of the crank: Point 2
            e = sqrt((x2 - self.d) ** 2 + (y2 ** 2))
            phi2 = arccos((e ** 2 + self.rr ** 2 - self.l ** 2) / (2 * e * self.rr))
            phi1 = arctan(y2 / (x2 - self.d)) + (1 - sign(x2 - self.d)) * pi / 2
            theta3 = phi1 - self.s * phi2 #gyro
            # theta3 = np.radians(300)
            self.RR_Angle[index] = theta3
            # if np.degrees(RR_Angle[index]) != None:
            #     print(print(np.degrees(RR_Angle[index])))
            x3 = self.rr * cos(theta3) + self.d
            # x cooridnate of the rocker moving point: Point 3
            y3 = self.rr * sin(theta3)
            # y cooridnate of the rocker moving point: Point 3

            theta2 = arctan((y3 - y2) / (x3 - x2)) + (1 - sign(x3 - x2)) * pi / 2

            self.X2[index] = x2  # grab the crankshaft x-position
            self.Y2[index] = y2  # grab the crankshaft y-position
            self.X3[index] = x3  # grab the connecting rod x-position
            self.Y3[index] = y3  # grab the connecting rod y-position

        self.update_plot()
        self.show()
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        if self.i >= len(self.X2): self.i = 0
        x_points = [self.x1, self.X2[self.i], self.X3[self.i], self.x4]
        y_points = [self.y1, self.Y2[self.i], self.Y3[self.i], self.y4]
        self.line.set_data(x_points, y_points)
        
        #self.canvas.axes.cla()
        self.canvas.axes.add_line(self.line)
        # Trigger the canvas to update and redraw.
        # self.canvas.draw()
        self.canvas.draw()

        self.i = self.i + 1

class Color(QWidget):

    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class Velocity(QWidget):
    def __init__(self):
        super(Velocity, self).__init__()
        self.velo = QLabel()
        self.velo.setText("Velocity")
        # velo.setAlignment(Qt.AlignCenter)
        self.x = QLabel()
        self.x.setText("X:")
        self.y = QLabel()
        self.y.setText("Y:")
        self.z = QLabel()
        self.z.setText("Z:")
        self.speed_unit1 = QLabel()
        self.speed_unit1.setText("m/s")
        self.speed_unit2 = QLabel()
        self.speed_unit2.setText("m/s")
        self.speed_unit3 = QLabel()
        self.speed_unit3.setText("m/s")
        self.velocityBox = QVBoxLayout()
        self.velocityGrid = QGridLayout()
        self.velocityBox.addWidget(self.velo)
        self.velocityGrid.addWidget(self.x, 0, 0)
        self.velocityGrid.addWidget(self.y, 1, 0)
        self.velocityGrid.addWidget(self.z, 2, 0)
        self.velocityGrid.addWidget(self.speed_unit1, 0, 2)
        self.velocityGrid.addWidget(self.speed_unit2, 1, 2)
        self.velocityGrid.addWidget(self.speed_unit3, 2, 2)
        self.velocityBox.addLayout(self.velocityGrid)

class Acceleration(QWidget):
    def __init__(self):
        super(Acceleration, self).__init__()
        self.acc = QLabel()
        self.acc.setText("Accelerometer")
        # velo.setAlignment(Qt.AlignCenter)
        self.x = QLabel()
        self.x.setText("X:")
        self.y = QLabel()
        self.y.setText("Y:")
        self.z = QLabel()
        self.z.setText("Z:")
        self.unit1 = QLabel()
        self.unit1.setText("m/s^2")
        self.unit2 = QLabel()
        self.unit2.setText("m/s^2")
        self.unit3 = QLabel()
        self.unit3.setText("m/s^2")
        self.accBox = QVBoxLayout()
        self.accGrid = QGridLayout()
        self.accBox.addWidget(self.acc)
        self.accGrid.addWidget(self.x, 0, 0)
        self.accGrid.addWidget(self.y, 1, 0)
        self.accGrid.addWidget(self.z, 2, 0)
        self.accGrid.addWidget(self.unit1, 0, 2)
        self.accGrid.addWidget(self.unit2, 1, 2)
        self.accGrid.addWidget(self.unit3, 2, 2)
        self.accBox.addLayout(self.accGrid)

app = QApplication(sys.argv)
w = MainWindow()
app.exec_()