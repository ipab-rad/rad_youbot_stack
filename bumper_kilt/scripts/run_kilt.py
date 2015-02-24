#!/usr/bin/python
import time
import serial
# configure the serial connections (the parameters differs on the
# device you are connecting to)

global DEBUG
DEBUG = 1

def dbprint(s, newline=True):
    global DEBUG
    if not DEBUG:
        pass
    if newline:
        print(s)
    else:
        print(s),


class Bumper(object):

    def __init__(self):
        self.ser = serial.Serial(
            port="/dev/ttyS0",
            baudrate=38400,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.SEVENBITS
            )
        try:
            print("Opening port...")
            self.ser.open()
        except:
            print("Port is already open!")
            self.ser.isOpen()

    def set_auto(self):
        print("Setting mode to auto... "),
        self.ser.write('a')
        out = ""
        time.sleep(1)
        while self.ser.inWaiting() > 0:
            r = self.ser.read(1)
            out += r
        if str(out) != "AUTO_MODE\r\n":
            print ""
            print "MSG: " + [o for o in out]
            raise Exception("Could not set mode to auto!")
        else:
            print("DONE")

    def get_sensors(self, msg):
        n_bits = 6
        h2b = lambda x: bin(int(x, 16))[2:].zfill(n_bits)
        dbprint(msg)
        bits = [h2b(msg[i:i+2]) for i in range(0, len(msg), 2)]
        dbprint("Single bits: " + str(bits))

    def auto_loop(self):
        while True:
            out = ""
            time.sleep(0.05)
            while self.ser.inWaiting() > 0:
                r = self.ser.read(1)
                out += r
            if len(out) == 12:
                dbprint(str([o for o in out]))
                print("New message: " + out[:-2])
                s = self.get_sensors(out[2:-2])
                print s


def main():
    b = Bumper()
    b.set_auto()
    b.auto_loop()



if __name__ == "__main__":
    main()
