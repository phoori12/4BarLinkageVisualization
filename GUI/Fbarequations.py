from numpy import floor, pi, sin, cos, sqrt, radians, arccos, arctan, sign, absolute

class FBarEquations:
    def __init__(self, link0, link1, link2, link3):
        self.mode = 0
        self.link0 = link0
        self.link1 = link1
        self.link2 = link2
        self.link3 = link3
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0

    def calculateLinks(self, input_deg): #กฎ 3 เหลี่ยม
        self.theta2 = radians(input_deg)

        div = floor(self.theta2 / (2*pi))
        self.theta2 = self.theta2 - (2*pi * div)

        L0 = self.link0
        L1 = self.link1[self.mode]
        L2 = self.link2[self.mode]
        L3 = self.link3[self.mode]

        AC = sqrt(L0*L0 + L1*L1 - 2*L0*L1*cos(self.theta2))
        beta = arccos((L0*L0 + AC*AC - L1*L1)/(2*L0*AC)) # rad
        psi = arccos((L2*L2 + AC*AC - L3*L3)/(2*L2*AC))
        lamb = arccos((L3*L3 + AC*AC - L2*L2)/(2*L3*AC))

        self.theta3 = psi - beta
        self.theta4 = pi - lamb - beta

        if self.theta2 > pi:
            self.theta3 = psi + beta
            self.theta4 = pi - lamb + beta

        # Define joint position
        Ox = 0
        Oy = 0

        Ax = Ox + L1*cos(self.theta2)
        Ay = Oy + L1*sin(self.theta2)

        Bx  = Ox + Ax + L2*cos(self.theta3)
        By  = Oy + Ay + L2*sin(self.theta3)

        Cx = L0
        Cy = 0
        
        # TODO Traigle Px, Py
        return [Ox, Ax, Bx, Cx], [Oy, Ay, By, Cy]