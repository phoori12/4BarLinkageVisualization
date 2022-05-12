from serialcomms import SerialComms

x = SerialComms()
x.ser.port = '/dev/cu.usbserial-0001'
x.connect()

test = x.getBuffer(2)
test = x.castBuffer(test)
print(test)
