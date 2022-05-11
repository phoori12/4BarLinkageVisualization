import serial.tools.list_ports
import serial

class SerialComms:
    def __init__(self, baudrate=9600):
        self.ports = []
        self.ser = serial.Serial()
        self.ser.port = ''
        self.ser.baudrate = baudrate
        
    def listPorts(self):
        self.ports = []
        ports = serial.tools.list_ports.comports()
        for port in sorted(ports):
            self.ports.append(port.device)
        return self.ports
        #print(self.ports)

    def connect(self):
        try:
            self.ser.open()
            return 1
        except:
            print("invalid port")
            return 0

    def disconnect(self):
        if self.ser.is_open:
            try:
                self.ser.close()
                return 1
            except:
                print("what?")
                return 2
        else:
            return 0
            


    def getBuffer(self):
        if not self.ser.is_open:
            return 0

        # read buffer and return each value to main ui