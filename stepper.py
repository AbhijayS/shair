import serial
import config

class Stepper:
    DELIMETER = ","

    def __init__(self) -> None:
        open()
    
    def write(self, length_inches):
        self.serial.write((str(length_inches) + self.DELIMETER).encode('utf-8'))
    
    def close(self):
        self.serial.close()
    
    def open(self):
        self.serial = serial.Serial(port=config.STEPPER_COM_PORT, baudrate=115200, write_timeout=0)

if __name__ == "__main__":
    myStepper = Stepper()
    myStepper.write(0)
    myStepper.close()