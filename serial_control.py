import serial
import time

class serial_control:
    def __init__(self, port):
        self.port = port
        print('start a new serial connection')
        self.host = serial.Serial(self.port, 1000000, timeout=1)
        time.sleep(1)
        
    def send(self, data):
        self.host.write(bytes(data, encoding='utf-8'))

    def receive(self):
        return self.host.read(50)

    def finish(self):
        print('serial connection ended')
        self.host.close()
    
