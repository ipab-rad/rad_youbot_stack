#!/usr/bin/python

"""
Interface to listen, analyse and publish the data from the bumper
kilt of the youbots.
Data is published in an array of 8 integers, starting from the front
bumper group and then going clockwise with the rest of the groups.
"""
import os
import time
import serial
import copy

import rospy
from std_msgs.msg import Int32MultiArray

global DEBUG
DEBUG = 0

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
        """
        Initialise Bumper class.
        ROBOT_NAME is used as namespace of the publisher.
        TODO: node should also be initialised with namespace
        """
        # clear screen for lolz
        os.system('cls' if os.name == 'nt' else 'clear')

        self.robot = os.environ["ROBOT_NAME"]
        self.nodename = rospy.get_name()
        rospy.loginfo("Starting bumper_kilt node", self.nodename)
        self.pub = rospy.Publisher(self.robot + '/bumper_kilt',
                              Int32MultiArray,
                              queue_size=10)
        rospy.init_node(self.robot + "_bumper_kilt_node")
        self.rate = rospy.Rate(10)

        self.bits = []
        self.ser = serial.Serial(
            port="/dev/ttyS0",
            baudrate=38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
            )
        try:
            print("Opening port...")
            self.ser.open()
        except:
            print("Port is already open!")
            self.ser.isOpen()

    def set_auto(self):
        """
        Set serial port mode to Automatic so that we just need to
        listen to the port.
        """
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

    def get_sensor_data(self, msg):
        """
        Assign binary data to `self.bits`.
        """
        n_bits = 6
        h2b = lambda x: bin(int(x, 16))[2:].zfill(n_bits)
        dbprint(msg)
        self.bits = [h2b(msg[i:i+2]) for i in range(0, len(msg), 2)]
        dbprint("Single bits: " + str(self.bits))

    def bumped_north(self):
        b3 = self.bits[2]
        b4 = self.bits[3]
        return int(b4[-1]) or int(b3[-1]) or int(b3[-2]) or int(b4[-2])

    def bumped_north_east(self):
        b3 = self.bits[2]
        return int(b3[3]) or int(b3[2])

    def bumped_east(self):
        b3 = self.bits[2]
        b1 = self.bits[0]
        return int(b3[1]) or int(b3[0]) or int(b1[0]) or int(b1[1])

    def bumped_south_east(self):
        b1 = self.bits[0]
        return int(b1[3]) or int(b1[2])

    def bumped_south(self):
        b1 = self.bits[0]
        b2 = self.bits[1]
        return int(b1[-2]) or int(b1[-1]) or int(b2[-1]) or int(b2[-2])

    def bumped_south_west(self):
        b2 = self.bits[1]
        return int(b2[3]) or int(b2[2])

    def bumped_west(self):
        b2 = self.bits[1]
        b4 = self.bits[3]
        return int(b2[1]) or int(b2[0]) or int(b4[0]) or int(b4[1])

    def bumped_north_west(self):
        b4 = self.bits[3]
        return int(b4[3]) or int(b4[2])

    def get_bumps(self):
        """
        Create data structure to contain bumps data.
        `self.bumps[0]` indicates the front group of bumpers and the
        following groups come clockwise.
        Value of 1: switch is active (touched);
        Value of 0: swithc is inactive (untouched).
        """
        bumps = []
        bits = copy.deepcopy(self.bits)
        if self.bumped_north():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_north_east():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_east():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_south_east():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_south():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_south_west():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_west():
            bumps.append(1)
        else:
            bumps.append(0)
        if self.bumped_north_west():
            bumps.append(1)
        else:
            bumps.append(0)
        self.bumps = bumps

    def auto_loop(self):
        """
        In auto mode, get data from serial port and publish it to the
        initialised ROS node.
        """
        counter = 0
        while True:
            out = ""
            time.sleep(0.05)
            while self.ser.inWaiting() > 0:
                r = self.ser.read(1)
                out += r
            if len(out) == 12:
                dbprint(str([o for o in out]))
                print(str(counter) + " - New message: " + out[:-2])
                self.get_sensor_data(out[2:-2])
                self.get_bumps()
                counter += 1
                os.system('cls' if os.name == 'nt' else 'clear')
                print("CURRENTLY TOUCHING: " + str(self.bumps))
                msg = Int32MultiArray()
                for e in self.bumps:
                    msg.data.append(e)
                self.pub.publish(msg)
                self.rate.sleep()

def main():
    b = Bumper()
    b.set_auto()
    b.auto_loop()


if __name__ == "__main__":
    main()
