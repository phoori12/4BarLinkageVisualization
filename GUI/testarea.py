from serialcomms import SerialComms

x = SerialComms()
x.ser.port = '/dev/cu.usbserial-0001'
x.connect()

test = x.getBuffer()
test = x.castBuffer(test)
print(test)
test = x.getBuffer()
test = x.castBuffer(test)

