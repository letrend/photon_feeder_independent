import serial

ser = serial.Serial('/dev/tnt0', 57600)  # replace with your serial port and baud rate

ser.write(b'M485\n')  # write a string to the serial port

response = ser.readline()  # read from the serial port
print(response)  # print the response
