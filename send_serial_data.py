       
import serial

serial_writer = serial.Serial()
serial_writer.baudrate = 115200
serial_writer.port = '/dev/ttyUSB0'
serial_writer.open()

data = "TESTE\r\n"

# data = input()
while(1):
    serial_writer.write(str.encode(data))