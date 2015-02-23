#!/usr/bin/python
import time
import serial

# configure the serial connections (the parameters differs on the
# device you are connecting to)

class Bumper(object):

    def __init__(self):
        try:
            self.ser = serial.Serial(
                port="/dev/ttyS0",
                baudrate=9600,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_TWO,
                bytesize=serial.SEVENBITS
            )
        except Exception:
            print("Bad initialisation! Check the configuration of "
                  "the serial port!")
            exit()
        self.ser.open()
        self.ser.isOpen()

    def loop(self):
        input=1
        while 1 :
            # get keyboard input
            input = raw_input(">> ")
            # Python 3 users
            # input = input(">> ")
            if input == "exit":
                self.ser.close()
                exit()
            else:
                # send the character to the device
                # (note that I happend a \r\n carriage return and line feed to
                # the characters - this is requested by my device)
                self.ser.write(input + "\r\n")
                out = ""
                # let's wait one second before reading output (let's give
                # device time to answer)
                time.sleep(1)
                while self.ser.inWaiting() > 0:
                    out += self.ser.read(1)

                if out != "":
                    print ">> " + out


def main():
    b = Bumper()
    b.loop()



if __name__ == "__main__":
    main()
