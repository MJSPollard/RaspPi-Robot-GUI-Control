import serial
from sys import version_info
import threading
import time
import sys

try:
    # for Python2
    # from Tkinter import *
    import Tkinter as tk
except ImportError:
    # for Python3
    # from tkinter import *
    import tkinter as tk

PY2 = version_info[0] == 2  # Running Python 2.x?


#
# ---------------------------
# Maestro Servo Controller
# ---------------------------
#
# Support for the Pololu Maestro line of servo controllers
#
# Steven Jacobs -- Aug 2013
# https://github.com/FRC4564/Maestro/
#
# These functions provide access to many of the Maestro's capabilities using the
# Pololu serial protocol
#
class Controller:
    # When connected via USB, the Maestro creates two virtual serial ports
    # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode.
    # "USB Chained Mode" may work as well, but hasn't been tested.
    #
    # Pololu protocol allows for multiple Maestros to be connected to a single
    # serial port. Each connected device is then indexed by number.
    # This device number defaults to 0x0C (or 12 in decimal), which this module
    # assumes.  If two or more controllers are connected to different serial
    # ports, or you are using a Windows OS, you can provide the tty port.  For
    # example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    def __init__(self, ttyStr='/dev/ttyACM0', device=0x0c):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device number are sent for each Pololu serial command.
        self.PololuCmd = chr(0xaa) + chr(device)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.Targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.Mins = [0] * 24
        self.Maxs = [0] * 24

    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr, 'latin-1'))

    # Set channels min and max value range.  Use this as a safety to protect
    # from accidentally moving outside known safe parameters. A setting of 0
    # allows unrestricted movement.
    #
    # ***Note that the Maestro itself is configured to limit the range of servo travel
    # which has precedence over these values.  Use the Maestro Control Center to configure
    # ranges that are saved to the controller.  Use setRange for software controllable ranges.
    def setRange(self, chan, min, max):
        self.Mins[chan] = min
        self.Maxs[chan] = max

    # Return Minimum channel range value
    def getMin(self, chan):
        return self.Mins[chan]

    # Return Maximum channel range value
    def getMax(self, chan):
        return self.Maxs[chan]

    # Set channel to a specified target value.  Servo will begin moving based
    # on Speed and Acceleration parameters previously set.
    # Target values will be constrained within Min and Max range, if set.
    # For servos, target represents the pulse width in of quarter-microseconds
    # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
    # Typcially valid servo range is 3000 to 9000 quarter-microseconds
    # If channel is configured for digital output, values < 6000 = Low ouput
    def setTarget(self, chan, target):
        # if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #
        lsb = target & 0x7f  # 7 bits for least significant byte
        msb = (target >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        # Record Target value
        self.Targets[chan] = target

    # Set speed of channel
    # Speed is measured as 0.25microseconds/10milliseconds
    # For the standard 1ms pulse width change to move a servo between extremes, a speed
    # of 1 will take 1 minute, and a speed of 60 would take 1 second.
    # Speed of 0 is unrestricted.
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f  # 7 bits for least significant byte
        msb = (speed >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
    # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
    def setAccel(self, chan, accel):
        lsb = accel & 0x7f  # 7 bits for least significant byte
        msb = (accel >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

        # Get the current position of the device on the specified channel
        # The result is returned in a measure of quarter-microseconds, which mirrors
        # the Target parameter of setTarget.
        # This is not reading the true servo position, but the last target position sent
        # to the servo. If the Speed is set to below the top speed of the servo, then
        # the position result will align well with the acutal servo position, assuming
        # it is not stalled or slowed.


def getPosition(self, chan):
    cmd = chr(0x10) + chr(chan)
    self.sendCmd(cmd)
    lsb = ord(self.usb.read())
    msb = ord(self.usb.read())
    return (msb << 8) + lsb

    # Test to see if a servo has reached the set target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo.  Servo range must be defined first using setRange. See setRange comment.
    #
    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to always be
    # moving to the target.
    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False

    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True or False.
    # Not available with Micro Maestro.
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        # can pass a param with command 0x28
        # cmd = chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Stop the current Maestro Script
    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)


class Gui455:
    command_list = []  # the list of commands - [moveType, value, time]

    # initilizies the main stuff in the class
    def __init__(self, master, contr):
        self.contr = contr
        self.master = master
        master.title("455GUI")

        # variables used by various parts of program
        self.step = 32
        self.timeChoice = 1
        self.valCount = 6000
        self.isRunning = False
        self.runCount = 0

        # set up the canvas and add a label to prompt user
        self.myCan = tk.Canvas(master, bg="#333333", width="500", height="250")
        self.myCan.pack(side="top", fill="both", expand=True)
        self.label = tk.Label(master, text="Choose a Command")
        self.label.pack()

        # adds text info on canvas
        self.curStri = self.myCan.create_text(450, 200, fill="red", anchor="center", text="PROGRAM NOT RUNNING")
        self.stringCommand = self.myCan.create_text(12, 12, fill="white", anchor="nw", text="Command Queue (max of 8)")

        # create 8 string input places to show user in gui
        self.stringName = {}
        for i in range(1, 9):
            self.stringName[i] = self.myCan.create_text(12, self.step, fill="white", anchor="nw", text=str(i) + ". ")
            self.step += 20
        self.step = 32

        # add buttons to the main gui window
        self.motor_button = tk.Button(master, text="motor", command=lambda: self.button_pressed(1))
        self.motor_button.pack(side=tk.LEFT)
        self.turn_button = tk.Button(master, text="turning", command=lambda: self.button_pressed(2))
        self.turn_button.pack(side=tk.LEFT)
        self.body_button = tk.Button(master, text="body", command=lambda: self.button_pressed(3))
        self.body_button.pack(side=tk.LEFT)
        self.headv_button = tk.Button(master, text="headv", command=lambda: self.button_pressed(4))
        self.headv_button.pack(side=tk.LEFT)
        self.headh_button = tk.Button(master, text="headh", command=lambda: self.button_pressed(5))
        self.headh_button.pack(side=tk.LEFT)
        self.delete_button = tk.Button(master, text="delete last", command=lambda: self.button_pressed(6))
        self.delete_button.pack(side=tk.LEFT)
        self.run_button = tk.Button(master, text="run", command=lambda: self.button_pressed(7))
        self.run_button.pack(side=tk.RIGHT)
        self.run_button = tk.Button(master, text="stop", command=lambda: self.button_pressed(8))
        self.run_button.pack(side=tk.RIGHT)

    # sets up the window that allows users to select a value/speed for robot movement
    def choose_values_window(self, val):
        # set up window with values
        self.wind = tk.Tk()
        self.wind.title("Val")
        self.numCan = tk.Canvas(self.wind, bg="#333333", width="100", height="100")
        self.numCan.pack(side="top", fill="both", expand=True)
        label1 = tk.Label(self.wind, text="select a value")
        label1.pack()

        # shows user what they have entered
        self.curValue = self.numCan.create_text(12, 12, fill="white", anchor="nw",
                                                text="Current Value = " + str(self.valCount))

        # add buttons to the value selction window
        self.up_button = tk.Button(self.wind, text="up", command=lambda: self.change_values("up", "value"))
        self.up_button.pack(side=tk.LEFT)
        self.down_button = tk.Button(self.wind, text="down", command=lambda: self.change_values("down", "value"))
        self.down_button.pack(side=tk.LEFT)
        self.done_button = tk.Button(self.wind, text="done", command=lambda: self.quit_window1(val))
        self.done_button.pack(side=tk.LEFT)
        self.wind.mainloop()  # wait for input

    # sets up the window that allows users to select the time for robot movement
    def choose_time_window(self, val):
        # set up window with values
        self.win = tk.Tk()
        self.win.title("Time")
        self.numCan = tk.Canvas(self.win, bg="#333333", width="100", height="100")
        self.numCan.pack(side="top", fill="both", expand=True)
        label1 = tk.Label(self.win, text="select a time")
        label1.pack()

        # shows user what they have entered
        self.curValue = self.numCan.create_text(12, 12, fill="white", anchor="nw",
                                                text="Current Time = " + str(self.timeChoice))

        # add buttons to the time selection window
        self.up_button = tk.Button(self.win, text="up", command=lambda: self.change_values("up", "time"))
        self.up_button.pack(side=tk.LEFT)
        self.down_button = tk.Button(self.win, text="down", command=lambda: self.change_values("down", "time"))
        self.down_button.pack(side=tk.LEFT)
        self.done_button = tk.Button(self.win, text="done", command=lambda: self.quit_window2(val))
        self.done_button.pack(side=tk.LEFT)
        self.win.mainloop()

    # alters the values that the user selects and displays them
    def change_values(self, val, inputType):
        if (val == "up"):
            if (inputType == "value"):
                self.valCount += 100
                self.numCan.itemconfig(self.curValue, text="Current Value = " + str(self.valCount))
            else:
                self.timeChoice += 1
                self.numCan.itemconfig(self.curValue, text="Current Time = " + str(self.timeChoice))

        elif (val == "down"):
            if (inputType == "value"):
                self.valCount -= 100
                self.numCan.itemconfig(self.curValue, text="Current Value = " + str(self.valCount))
            else:
                if (self.timeChoice > 1):
                    self.timeChoice -= 1
                    self.numCan.itemconfig(self.curValue, text="Current Time = " + str(self.timeChoice))

    # quick hack to differentiate the first window from second window closing
    def quit_window1(self, val):
        self.wind.quit()
        self.wind.destroy()

    # sets values that were input by user into the list and displays text showing so
    def quit_window2(self, val):
        self.command_list.append([val, self.valCount, self.timeChoice])
        wordType = ""
        currPlace = len(self.command_list)

        # figure out which word to show user
        if (val == 1):
            wordType = "Motor"
        elif (val == 2):
            wordType = "Turning"
        elif (val == 3):
            wordType = "Body Movement"
        elif (val == 4):
            wordType = "Vert. Head Movement"
        elif (val == 5):
            wordType = "Horiz. Head Movement"
        else:
            wordType = "error"

        # shows user what they input in the right position
        self.myCan.itemconfig(self.stringName[currPlace], text=str(currPlace) + ". > " + wordType + ", Value = " + str(
            self.valCount) + ", Time Allowed = " + str(self.timeChoice))

        # reset and destroy
        self.win.quit()
        self.win.destroy()

    # Calls the threads and sets values for when program is running
    def execute_threads(self, val):
        self.pos_x = 300
        self.pos_y = 220
        self.myCan.itemconfig(self.curStri, text="")
        self.curStri = self.myCan.create_text(450, 200, fill="green", anchor="center", text="PROGRAM RUNNING")
        self.curStr = self.myCan.create_text(self.pos_x, self.pos_y, fill="green", anchor="center",
                                             text="Stage = " + str(self.runCount))
        self.isRunning = True
        threading.Thread(target=self.animation_thread).start()
        threading.Thread(target=self.command_thread).start()

    # runs the thread that controls robot movement
    def command_thread(self):
        neutralPos = 6000

        # loops through the command list extracting values and calls the controller methods
        for i in range(len(self.command_list)):
            if (not self.isRunning):
                break  # terminates thread if isRunning = false

            self.runCount += 1
            moveType = self.command_list[i][0]
            valueGiven = self.command_list[i][1]
            timeAllowed = self.command_list[i][2]
            if (moveType == 1):  # motor
                cont.setTarget(1, valueGiven)
                time.sleep(timeAllowed)
                self.contr.setTarget(1, neutralPos)
                time.sleep(1)  # makes the robot rest after moving to prevent violent jerking

            elif (moveType == 2):  # turning
                self.contr.setTarget(2, valueGiven)
                time.sleep(timeAllowed)
                cont.setTarget(2, neutralPos)
                time.sleep(1)

            elif (moveType == 3):  # body
                self.contr.setTarget(0, valueGiven)
                time.sleep(timeAllowed)

            elif (moveType == 4):  # headv
                print("turn head")
                self.contr.setTarget(4, valueGiven)
                time.sleep(timeAllowed)

            elif (moveType == 5):  # headh
                self.contr.setTarget(3, valueGiven)
                time.sleep(timeAllowed)
            else:
                print("error")
        self.isRunning = False
        self.endMethod()

    # the method that the animation thread runs in the background
    def animation_thread(self):
        color_value = ""

        # ensures the program is running for the animation to occur
        while (self.isRunning):

            # text flashes, moves back and forth and changes stage number
            for i in range(1, 7):
                if (not self.isRunning):
                    break
                if (i % 2 == 0):
                    color_value = "green"
                else:
                    color_value = "aqua"
                time.sleep(0.4)
                self.pos_x += 60
                self.myCan.itemconfig(self.curStr, text="")
                self.curStr = self.myCan.create_text(self.pos_x, self.pos_y, fill=color_value, anchor="center",
                                                     text="Stage = " + str(self.runCount))

            for i in range(1, 7):
                if (not self.isRunning):
                    break
                if (i % 2 == 0):
                    color_value = "green"
                else:
                    color_value = "aqua"
                time.sleep(0.4)
                self.pos_x -= 60
                self.myCan.itemconfig(self.curStr, text="")
                self.curStr = self.myCan.create_text(self.pos_x, self.pos_y, fill=color_value, anchor="center",
                                                     text="Stage = " + str(self.runCount))

            self.myCan.itemconfig(self.curStr, text="")

    # called when the running sequence is over or stop was pressed
    def endMethod(self):
        # stop threads
        self.isRunning == False
        # reset text animations
        self.myCan.itemconfig(self.curStri, text="")
        self.curStri = self.myCan.create_text(450, 200, fill="red", anchor="center", text="PROGRAM NOT RUNNING")
        self.myCan.itemconfig(self.curStr, text="")
        # reset robot to original positions
        self.contr.setTarget(0, 6000)
        self.contr.setTarget(1, 6000)
        self.contr.setTarget(2, 6000)
        self.contr.setTarget(3, 6000)
        self.contr.setTarget(4, 6000)
        # reset values
        for i in range(len(self.command_list)):
            self.myCan.itemconfig(self.stringName[i + 1], text=str(i + 1) + ".")

        self.runCount = 0
        self.command_list[:] = []

    # called when a button in the main gui is pressed and reacts accordingly
    def button_pressed(self, val):
        # logic statements for robot movement and user input
        if (len(self.command_list) < 8 and self.isRunning == False):
            if (val == 1):
                self.choose_values_window(val)
                self.choose_time_window(val)
                self.step += 20
                print("motor pressed")
            elif (val == 2):
                self.choose_values_window(val)
                self.choose_time_window(val)
                self.step += 20
                print("turn pressed")
            elif (val == 3):
                self.choose_values_window(val)
                self.choose_time_window(val)
                self.step += 20
                print("body pressed")
            elif (val == 4):
                self.choose_values_window(val)
                self.choose_time_window(val)
                self.step += 20
                print("headv pressed")
            elif (val == 5):
                self.choose_values_window(val)
                self.choose_time_window(val)
                self.step += 20
                print("headh pressed")
        else:
            print("you may not exceed 8 commands at a time")

        # logic statements for delete and run commands
        if (val == 6):
            if (len(self.command_list) > 0 and self.isRunning == False):
                self.command_list = self.command_list[:-1]
                self.myCan.itemconfig(self.stringName[len(self.command_list) + 1],
                                      text=str(len(self.command_list) + 1) + ".")
                self.step -= 20
                print("delete pressed")
            else:
                print("Invalid, enter a command first")

        if (val == 7):
            if (len(self.command_list) > 0 and self.isRunning == False):
                self.execute_threads(val)
                print("run pressed")
            else:
                print("Invalid, enter a command first")
        if (val == 8 and self.isRunning == True):
            self.isRunning = False
            self.endMethod()
            print("stopped pressed")

        # resets user input values
        self.valCount = 6000
        self.timeChoice = 1
        print(self.command_list)


# beginning of program
if __name__ == '__main__':
    root = tk.Tk()  # the gui window
    cont = Controller()  # the maestro controller
    gui = Gui455(root, cont)
    root.mainloop()
