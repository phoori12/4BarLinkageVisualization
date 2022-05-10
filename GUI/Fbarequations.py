from numpy import floor, pi, sin, cos, sqrt, radians, arccos, degrees

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

    def calculateLinks(self, input_deg, mode=0): #กฎ 3 เหลี่ยม mode 0 get coordinate / mode 1 get angles
        self.theta2 = radians(input_deg)

        div = floor(self.theta2 / (2*pi))
        self.theta2 = self.theta2 - (2*pi * div)

        L0 = self.link0
        L1 = self.link1[self.mode]
        L2 = self.link2[self.mode]
        L3 = self.link3[self.mode]

        AC = sqrt(L0*L0 + L1*L1 - 2*L0*L1*cos(self.theta2))
        betaAcosAttrib = (L0*L0 + AC*AC - L1*L1)/(2*L0*AC)
        psiAcosAttrib = (L2*L2 + AC*AC - L3*L3)/(2*L2*AC)
        lambAcosAttrib = (L3*L3 + AC*AC - L2*L2)/(2*L3*AC)
        if betaAcosAttrib > 1:
            betaAcosAttrib = 1
        elif betaAcosAttrib < -1:
            betaAcosAttrib = -1

        if psiAcosAttrib > 1:
            psiAcosAttrib = 1
        elif psiAcosAttrib < -1:
            psiAcosAttrib = -1
        
        if lambAcosAttrib > 1:
            lambAcosAttrib = 1
        elif lambAcosAttrib < -1:
            lambAcosAttrib = -1

        beta = arccos(betaAcosAttrib) 
        psi = arccos(psiAcosAttrib)
        lamb = arccos(lambAcosAttrib)

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
        if mode == 0:
            return [Ox, Ax, Bx, Cx], [Oy, Ay, By, Cy]
        
        return [degrees(self.theta2), degrees(self.theta4)]

    def drawFromBothDegree(self, theta2, theta4):
        theta2 = radians(theta2)
        theta4 = radians(theta4)
        L0 = self.link0
        L1 = self.link1[self.mode]
        # L2 = self.link2[self.mode]
        L3 = self.link3[self.mode]

        Ox = 0
        Oy = 0

        Ax = Ox + L1*cos(theta2)
        Ay = Oy + L1*sin(theta2)

        Cx = L0
        Cy = 0

        Bx  = Cx + L3*cos(theta4)
        By  = Cy + L3*sin(theta4)

        return [Ox, Ax, Bx, Cx], [Oy, Ay, By, Cy]

        

