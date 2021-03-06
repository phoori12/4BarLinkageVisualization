import string
import serial.tools.list_ports
import serial
import struct
import time

class SerialComms:
    def __init__(self, baudrate=9600):
        self.ports = []
        self.ser = serial.Serial()
        self.ser.port = ''
        self.ser.baudrate = baudrate
        self.buffer = [None]*52
        self.cmdIndex = 0
        self.cmdLength = 52
        
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
            # self.ser.write("<")
            return 1
        except:
            print("invalid port")
            return 0

    def disconnect(self):
        if self.ser.is_open:
            try:
                #self.ser.write(">")
                self.ser.close()
                return 1
            except:
                print("what?")
                return 2
        else:
            return 0
            
    def getBuffer(self, timeout):
        loop_start_time = time.time()
        while self.ser.is_open and self.ser.in_waiting:
            
            recv_ = self.ser.read(1)
            
            try:
                self.buffer[self.cmdIndex] = recv_
            except:
                print(str(self.cmdIndex))
            
            if (self.cmdIndex == 0) and self.buffer[0] != b'#':
                continue

            #print('got #')
            if (self.buffer[self.cmdIndex - 51] == b'#') and (self.buffer[self.cmdIndex - 50] == b's') and (self.buffer[self.cmdIndex - 1] == b'\r') and (self.buffer[self.cmdIndex] == b'\n'):
                self.cmdIndex = 0
                return_buffer = self.buffer
                self.buffer = [None]*52
                #self.ser.reset_input_buffer()
                #print('buffer filled')
                return return_buffer
            else:
                if self.cmdIndex < self.cmdLength -1:
                    self.cmdIndex += 1
                    #print(f'cmdIndex, {0}', self.cmdIndex)
                else:
                    print(self.buffer)
                    self.buffer = [None]*52
                    self.cmdIndex = 0
                    #self.ser.reset_input_buffer()
                    #print('flushed buffer')

            if time.time() - loop_start_time > timeout:
                self.buffer = [None]*52
                self.cmdIndex = 0
                #self.ser.reset_input_buffer()
                #print('timed out')
                return 0
        return 0
    
    def castBuffer(self, buffer):
        data = []
        buffer.pop(0)
        buffer.pop(0)
        buffer.pop(-1)
        buffer.pop(-1)

        for i in range(len(buffer)):
            if (i+1) % 4 == 0:
                temp = buffer[i] + buffer[i-1] + buffer[i-2] + buffer[i-3]
                temp = struct.unpack('f', temp)
                data.append(round(temp[0],2))
        return data