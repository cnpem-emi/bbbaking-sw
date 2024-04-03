#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Control Software for Sirius Baking Tapes and Jackets
BEAGLEBONE BLACK
Author: Marcelo Bacchetti | Patricia Nallin

Last release:
August 28 2019
"""

import sys, os, traceback
import signal
import time, datetime
import numpy as np
import MBTemp
import socket
import converters
from threading import Event, Thread
import Adafruit_BBIO.GPIO as GPIO

# Global variables
global CHANNELS, channel_active, periods, EnabledChannels, HoldedChannels
global voltage, voltage_offset_all, voltage_offset, voltage_gain_all, voltage_gain
global current, current_offset_all, current_offset, current_gain_all, current_gain, CurrentLimit
global power, resistance, temperature, Curve, temp_tape_read, temp_tape, temp_tape_difference, temp_Pt100, init_temperature, temp_coef, t0, r0, resistance_cable, resistance_cable_all
global PID_setpoint, PID_term, PID_integration, PID_error, PID_value
global DAC_value, ADC_value, ADC2_value
global OFFSET, AUTO, INTERPOL

LED = "P9_14"
BUTTON = "P8_11"

WORKING_DIR = os.path.dirname(os.path.realpath(sys.argv[0])) + "/calibracoes"

# System Flags
PT100 = True                   # Set True if use Pt100 to measure temperature
AUTO = False                    # Set True if want to PID control
INTERPOL = False                # Set True if have setpoint vector to interpolate

# Initial Active Channels
channel_active = [0,1,2,3,4,5,6,7]       # List of channels
CHANNELS = len(channel_active)
resistance_cable_all = [0.47]*8
CurrentLimit = [0.0]*8

# Gain and Offset
current_offset_all = np.array([])
current_gain_all = np.array([])

voltage_offset_all = np.array([])
voltage_gain_all = np.array([])

# Control limits
tolerance = 1.1
MIN_VOLTAGE = 4.0 # Volts
MAX_VOLTAGE = 48.0 # Volts
MAX_POWER = tolerance * 300 # Watts
MAX_RESISTANCE = 200 # Ohms

# Gaveta info - Number
f = open(WORKING_DIR + "Gaveta.info", "r")
gaveta_id = f.read()[:-1]

# Calibration values
calibration_file = open(WORKING_DIR + "Calib_Gaveta" + gaveta_id + ".csv", "r")
calibration_file.readline()
for i in range(8):
    CalibCh = calibration_file.readline().split("\n")[0].split(";")

    current_gain_all = np.append(current_gain_all, float(CalibCh[1]))
    current_offset_all = np.append(current_offset_all, float(CalibCh[2]))

    voltage_gain_all = np.append(voltage_gain_all, float(CalibCh[3]))
    voltage_offset_all = np.append(voltage_offset_all, float(CalibCh[4]))


current_gain = current_gain_all
current_offset = current_offset_all
voltage_gain = voltage_gain_all
voltage_offset = voltage_offset_all


# Tape Config
temp_coef = [255.0]*CHANNELS

# DAC and ADC functions
MAX_DAC = 4095
MIN_DAC = 410
MIN_DAC_RESISTANCE_MEASURE = 820
DAC = converters.DAC()
ADC = converters.ADC()
ADC2 = converters.ADC2()


# PID Configs
PID_value = [0.0004, 0.0001, 0]
periods = [0]*len(channel_active)     # Number of updates since init

# Constants
UP_FREQ = 5.0

# Temperature Config
temperature_ID = 4              # MBTemp to read temperature
alpha_temp = 0.9
channel_via_Pt100 = channel_active


# Pt100 Equation
# R = R0*(1+ B.t + A.t^2)
# R = (1+ Pt100_B.t + Pt100_A.t^2)
# temp[i] = (-Pt100_B + (sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(R0-resistence)))))/(2*Pt100_A);
A = -5.775e-7
B = 3.9083e-3
Pt100_R0 = 100.0
Pt100_A = A*Pt100_R0
Pt100_B = B*Pt100_R0

# Datetime string
def time_string():
    return(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f")[:-4] + " - ")

# Function to clear and resize variables
def resetVariables():
    global CHANNELS, channel_active, EnabledChannels, HoldedChannels, Curve, CurrentLimit
    global voltage, voltage_offset_all, voltage_offset, voltage_gain_all, voltage_gain, current, current_offset_all, current_offset, current_gain_all, current_gain
    global power, resistance, temperature, temp_coef, temp_tape_read, temp_tape, temp_tape_difference, temp_Pt100, r0, t0, init_temperature, init_resistance, resistance_cable, resistance_cable_all
    global PID_setpoint, PID_CV, PID_integration, PID_error, periods
    global DAC_value, ADC_value, ADC2_value
    global OFFSET

    CHANNELS = len(channel_active)  # How many channels are connected
    voltage = [0.0]*CHANNELS          # Applied voltage
    current = [0.0]*CHANNELS          # Measured current
    power = [0.0]*CHANNELS
    resistance = [0.0]*CHANNELS       # Calulated resistance
    temperature = np.array([0.0]*CHANNELS)      # Calculated temperature
    temp_coef = [255.0]*CHANNELS
    init_temperature = []
    init_resistance = []
    resistance_cable = []
    current_offset = []
    current_gain = []
    voltage_offset = []
    voltage_gain = []

    for chn in (channel_active):
        init_temperature.append(t0[chn])
        init_resistance.append(r0[chn])
        current_offset.append(current_offset_all[chn])
        current_gain.append(current_gain_all[chn])
        voltage_offset.append(voltage_offset_all[chn])
        voltage_gain.append(voltage_gain_all[chn])
        resistance_cable.append(resistance_cable_all[chn])

    init_temperature = np.array(init_temperature)
    init_resistance = np.array(init_resistance)
    current_offset = np.array(current_offset)
    current_gain = np.array(current_gain)
    voltage_offset = np.array(voltage_offset)
    voltage_gain = np.array(voltage_gain)
    resistance_cable = np.array(resistance_cable)

    CurrentLimit = [0.0]*CHANNELS
    channel_via_Pt100 = channel_active
    temp_tape = np.array([0.0]*CHANNELS)
    temp_tape_read = np.array([0.0]*CHANNELS)
    temp_tape_difference = [0]*CHANNELS
    temp_Pt100 = np.array([0.0]*CHANNELS)
    PID_setpoint = [0]*CHANNELS
    PID_CV = np.power((48*MIN_DAC/4096.0),2)/init_resistance
    PID_integration = [0.0]*CHANNELS
    PID_error = [[0.0]*CHANNELS]*3
    periods = [0]*CHANNELS
    DAC_value = [0]*CHANNELS
    ADC_value = [0]*CHANNELS
    ADC2_value = [0]*CHANNELS
    EnabledChannels = []
    HoldedChannels = []
    Curve = []
    for i in range(CHANNELS):
        Curve.append([])


# When Ctrl+C, turn outputs off
def signal_handler(signal, frame):
    # Stop running and end process
    run_control.clear()
    end_control.set()


# Thread class to control inputs/outputs
class IO_control(Thread):
    def __init__(self, event, period):
        Thread.__init__(self)
        self.stopped = event
        self.period = period    # Update period

    def run(self):
        # Global variables
        global periods, INTERPOL
        global ADC_value, DAC_value, ADC2_value
        global channel_active, HoldedChannels

        # To get exactly time
        def g_tick():
            t = time.time()
            count = 0
            while True:
                count += 1
                yield max(t + count*self.period - time.time(),0)

        g = g_tick()

        # Thread loop
        while True:
            time.sleep(g.next())
            if run_control.isSet():

                # Update de periods counter, para os canais habilitados e que nao estao em Hold
                if INTERPOL:
                    for index, channel in enumerate(channel_active):
                        if channel in HoldedChannels:
                            continue
                        else:
                            periods[index] += 1

                # Set DAC output
                DAC.write(DAC_value, channel_active)

                # Read ADC - Average of n measures
                aux = np.array([0]*CHANNELS)
                aux2 = np.array([0]*CHANNELS)
                for i in range(25):
                    aux += ADC.read(channel_active)
                    aux2 += ADC2.read(channel_active)

                ADC_value = aux / 25.0
                ADC2_value = aux2 / 25.0

#                os.system("clear")
#                print time_string()
#                print periods
#                print "ADC", ADC_value
#                print "DAC", DAC_value
#                print "ADC Tensao puro ",ADC2_value

                # New measure was completed
                new_measure.set()

# Thead to send and receive values on demand
class Communication(Thread):
    HOST = ''
    def __init__(self, port):
        Thread.__init__(self)
        self.port = port

    def run(self):
        global DAC_value
        global PID_value, PID_setpoint
        global channel_active, channel_via_Pt100, EnabledChannels, HoldedChannels, CurrentLimit
        global r0, t0, init_resistance, init_temperature, temperature, temp_tape, temp_Pt100, temp_coef
        global Curve, AUTO, INTERPOL
        global periods
        while (True):
            try:

                # TCP/IP socket initialization
                self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp.bind(("", self.port))
                self.tcp.listen(1)

                sys.stdout.write(time_string() + "TCP/IP Server on port " + str(self.port) + " started.\n")
                sys.stdout.flush()

                while (True):

                    sys.stdout.write(time_string() + "Waiting for connection.\n")
                    sys.stdout.flush()
                    self.tcp.settimeout(2)

                    client_info = []
                    while (client_info == []):
                        try:
                            con, client_info = self.tcp.accept()
                        except socket.timeout:
                            GPIO.output(LED, GPIO.HIGH)
                            time.sleep(2)
                            GPIO.output(LED, GPIO.LOW)



                    # New connection
                    sys.stdout.write(time_string() + "Connection accepted from " + client_info[0] + ":" + str(client_info[1]) + ".\n")
                    sys.stdout.flush()
                    GPIO.output(LED, GPIO.HIGH)
                    self.tcp.settimeout(None)

                    while (True):
                        # Get message
                        RW = con.recv(1)

                        if (RW):
                            msg = con.recv(1)

                            # Command A - Alpha (R = R0(1 + alpha*deltaT))
                            if msg == "A":
                                if (RW == "R"):
                                    con.send(";".join(map(str, temp_coef)))
                                    #sys.stdout.write(time_string() + "Read alpha" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    temp_coef = np.array(map(float, aux.split(";")))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write alpha to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command C - Set PID
                            if msg == "C":
                                if (RW == "R"):
                                    con.send(";".join(map(str, PID_value)))
                                    #sys.stdout.write(time_string() + "Read PID constants" + " \n")
                                    #sys.stdout.flush()
                                    continue
                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    PID_value = map(float, aux.split(";"))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write PID constants to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command E - Enable Channels
                            if msg == "E":
                                if (RW == "R"):
                                    if len(EnabledChannels):
                                        con.send(";".join(map(str, EnabledChannels)))
                                    else:
                                        con.send("NONE")
                                    #sys.stdout.write(time_string() + "Read enabled channels" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    if (aux == 'NONE'):
                                        EnabledChannels = []
                                    else:
                                        EnabledChannels = map(int, aux.split(";"))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write enabled channels to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue



                            # Command F
                            if msg == "F":
                                if (RW == "W"):
                                    aux = con.recv(1024)
                                    flags = int(aux[1:])
                                    aux = aux[0]
                                    if aux == "S": # Command "S" - Set flags
                                        if (flags & 1) == 1:
                                            run_control.set() # bit0 Flag
                                            con.send(RW+msg+"OK")
                                            sys.stdout.write("Run Control SET")
                                            sys.stdout.flush()
                                        if (flags & 2) == 2:
                                            end_control.set() # bit1 Flag
                                            con.send(RW+msg+"OK")
                                            sys.stdout.write(time_string() + "End Control SET \n")
                                            sys.stdout.flush()
                                        continue
                                    if aux == "R": # Command "R" - Reset flags
                                        if (flags & 1) == 1:
                                            run_control.clear() # bit0 Flag
                                            DAC.write([0]*8, [i for i in range(8)])
                                            time.sleep(0.5)
                                            con.send(RW+msg+"OK")
                                            sys.stdout.write(time_string() + "Run Control RESET \n")
                                            sys.stdout.flush()
                                        if (flags & 2) == 2:
                                            end_control.clear() # bit1 Flag
                                            con.send(RW+msg+"OK")
                                            sys.stdout.write(time_string() + "End Control RESET \n")
                                            sys.stdout.flush()

                                        continue
                                    continue

                            # Command G - Go: Starts curve running
                            if msg == "G":
                                if (RW == "R"):
                                    if (AUTO and INTERPOL):
                                        con.send("RUNNING")
                                    else:
                                        con.send("NOT RUNNING")

                                    #sys.stdout.write(time_string() + "Read curve status" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1)
                                    if aux == "S":
                                        # Clear time
                                        periods = [0]*len(channel_active)
                                        # Starts interpolation
                                        AUTO = True
                                        INTERPOL = True

                                        con.send(RW+msg+"OK")
                                        sys.stdout.write(time_string() + "Run curves from begining \n")
                                        sys.stdout.flush()
                                        continue

                                    if aux == "R":
                                        # Stops interpolation
                                        AUTO = False
                                        INTERPOL = False

                                        # Clear outputs
                                        DAC_value = [0]*len(channel_active)

                                        con.send(RW+msg+"OK")
                                        sys.stdout.write(time_string() + "Stop curves and clear outputs \n")
                                        sys.stdout.flush()
                                        continue

                            # Command H - Set cHannels
                            if msg == "H":
                                if (RW == "R"):
                                    con.send(";".join(map(str, channel_active)))
                                    #sys.stdout.write(time_string() + "Read active channels" + " \n")
                                    #sys.stdout.flush()
                                    continue
                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    channel_active = map(int, aux.split(";"))
                                    resetVariables()
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write active channels to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command h - Set temperatures READ BY Pt100s
                            if msg == "h":
                                if (RW == "R"):
                                    if len(channel_via_Pt100):
                                        con.send(";".join(map(str, channel_via_Pt100)))
                                    else:
                                        con.send("NONE")
                                    #sys.stdout.write(time_string() + "Read channels read by Pt100s" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    if (aux == 'NONE'):
                                        channel_via_Pt100 = []
                                    else:
                                        channel_via_Pt100 = map(int, aux.split(";"))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write channels read by Pt100s to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command I - Get Current
                            if msg == "I":
                                if (RW == "R"):
                                    con.send(";".join(map(str, current)))
                                    #sys.stdout.write(time_string() + "Read current current" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command i - Limit Current
                            if msg == "i":
                                if (RW == "R"):
                                    con.send(";".join(map(str, CurrentLimit)))
                                    #sys.stdout.write(time_string() + "Read initial temperature" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    CurrentLimit = np.array(map(float, aux.split(";")))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write CurrentLimit to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command L - Hold Channels (Curve)
                            if msg == "L":
                                if (RW == "R"):
                                    if len(HoldedChannels):
                                        con.send(";".join(map(str, HoldedChannels)))
                                    else:
                                        con.send("NONE")
                                    #sys.stdout.write(time_string() + "Read holded channels" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    if (aux == 'NONE'):
                                        HoldedChannels = []
                                    else:
                                        HoldedChannels = map(int, aux.split(";"))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write holded channels to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command O - Initial temperature
                            if msg == "O":
                                if (RW == "R"):
                                    con.send(";".join(map(str, init_temperature)))
                                    #sys.stdout.write(time_string() + "Read initial temperature" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    init_temperature = np.array(map(float, aux.split(";")))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write initial temperature to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                                elif (RW == "G"):
                                    aux = con.recv(1024)
                                    channels = map(int, aux.split(";"))
                                    answer = measure.GetTemperature(channels)
                                    for index, chn in enumerate(channels):
                                        t0[chn] = answer[index]
                                        if chn in channel_active:
                                            init_temperature[channel_active.index(chn)] = answer[index]
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Get new initial temperature in channels: " + aux + " - New values: " + "%s" % answer + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command P - Write DAC
                            if msg == "P":
                                if (RW == "R"):
                                    con.send(";".join(map(str, DAC_value)))
                                    #sys.stdout.write(time_string() + "Read DAC values" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    DAC_value = map(int, aux.split(";"))
                                    AUTO = False
                                    INTERPOL = False
                                    time.sleep(0.5)
                                    # Clear temp_tape vector to get new values with no mean calc
                                    temp_tape = [0.0]*CHANNELS
                                    time.sleep(1)
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write DAC values to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command p - Get temperature - Tape and Pt100
                            if msg == "p":
                                if (RW == "R"):
                                    con.send(";".join(map(str, temperature)))
                                    #sys.stdout.write(time_string() + "Read temperatures Tape and Pt100" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command R - Get resistance
                            if msg == "R":
                                if (RW == "R"):
                                    con.send(";".join(map(str, resistance)))
                                    #sys.stdout.write(time_string() + "Read current tape resistances" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command r - Initial resistance
                            if msg == "r":
                                if (RW == "R"):
                                    con.send(";".join(map(str, init_resistance)))
                                    #sys.stdout.write(time_string() + "Read Initial Resistances" + " \n")
                                    #sys.stdout.flush()
                                    continue
                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    init_resistance = map(float, aux.split(";"))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write Initial Resistances to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue
                                elif (RW == "G"):
                                    aux = con.recv(1024)
                                    channels = map(int, aux.split(";"))
                                    answer = measure.GetResistance(channels, 50)
                                    for index, chn in enumerate(channels):
                                        r0[chn] = answer[index]
                                        if chn in channel_active:
                                            init_resistance[channel_active.index(chn)] = answer[index]
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Get new initial resistances in channels: " + aux + " - New values: " + "%s" % answer + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command S - Write Setpoint
                            if msg == "S":
                                if (RW == "R"):
                                    con.send(";".join(map(str, PID_setpoint)))
                                    #sys.stdout.write(time_string() + "Read setpoints" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    PID_setpoint = map(float, aux.split(";"))
                                    AUTO = True
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write setpoints to: " + aux + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command T - Get temperature - Pt100
                            if msg == "T":
                                if (RW == "R"):
                                    con.send(";".join(map(str, temp_Pt100)))
                                    #sys.stdout.write(time_string() + "Read Pt100 temperature" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command t - Get temperature - Fita (Tape)
                            if msg == "t":
                                if (RW == "R"):
                                    con.send(";".join(map(str, temp_tape)))
                                    #sys.stdout.write(time_string() + "Read tape temperature" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command U - Get Voltage
                            if msg == "U":
                                if (RW == "R"):
                                    con.send(";".join(map(str, voltage)))
                                    #sys.stdout.write(time_string() + "Read current voltage" + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command V - Insert point at Setpoint vector
                            if msg == "V":
                                if (RW == "R"):
                                    con.send("NOT AVAILABLE")
                                    #sys.stdout.write(time_string() + "Read curve" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    vec = map(float, aux.split(";"))
                                    ch = int(vec.pop(0))
                                    chn = channel_active.index(ch)
                                    Curve[chn] = []
                                    for i in vec:
                                        Curve[chn].append(float(i))

                                    AUTO = True
                                    INTERPOL = True

                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write vector " + aux + " to channel " + str(ch) + " \n")
                                    sys.stdout.flush()
                                    continue

                            # Command W - Get Instant Power
                            if msg == "W":
                                if (RW == "R"):
                                    con.send(";".join(map(str, power)))
                                    #sys.stdout.write(time_string() + "Read current power " + " \n")
                                    #sys.stdout.flush()
                                    continue

                            # Command X - Return periods number
                            if msg == "X":
                                if (RW == "R"):
                                    con.send(";".join(map(str, np.array(periods)/UP_FREQ)))
                                    #sys.stdout.write(time_string() + "Read time" + " \n")
                                    #sys.stdout.flush()
                                    continue

                                elif (RW == "W"):
                                    aux = con.recv(1024)
                                    periods = np.array(map(int,[i*UP_FREQ for i in map(float, aux.split(";"))]))
                                    con.send(RW+msg+"OK")
                                    sys.stdout.write(time_string() + "Write seconds time to: " + aux + str(periods) + " \n")
                                    sys.stdout.flush()
                                    continue
                            else:
                                sys.stdout.write(time_string() + "Unknown message: " + RW + msg + " \n")
                                sys.stdout.flush()
                                continue
                        else:
                            # Disconnection
                            sys.stdout.write(time_string() + "Client " + client_info[0] + ":" + str(client_info[1]) + " disconnected.\n")
                            sys.stdout.flush()
                            # Reset channels
                            sys.stdout.write(time_string() + "Set all channels active and clear variables.\n")
                            sys.stdout.flush()
                            run_control.clear()
                            time.sleep(1)
                            channel_active = [0,1,2,3,4,5,6,7]
                            DAC.write([0]*8, channel_active)
                            AUTO = False
                            INTERPOL = False
                            resetVariables()

                            GPIO.output(LED, GPIO.LOW)
                            break


            except Exception:
                self.tcp.close()

                sys.stdout.write(time_string() + "Connection problem. TCP/IP server was closed. Error:\n\n")
                traceback.print_exc(file = sys.stdout)
                sys.stdout.write("\n")
                sys.stdout.flush()

                # Reset channels
                sys.stdout.write(time_string() + "Set all channels active and clear variables.\n")
                sys.stdout.flush()
                run_control.clear()
                time.sleep(1)
                channel_active = [0,1,2,3,4,5,6,7]
                DAC.write([0]*8, channel_active)
                AUTO = False
                INTERPOL = False
                resetVariables()

                time.sleep(5)



class measure:
    @staticmethod
    # Measure each tape and calculate the initial resistance
    def GetResistance(channels, avg = 1):

        DAC.write([MIN_DAC_RESISTANCE_MEASURE]*8, [i for i in range(8)])

        time.sleep(0.5)

        resistance = [0]*8
        for k in range(avg):
            ADC_value = ADC.read([i for i in range(8)])
            ADC2_value = ADC2.read([i for i in range(8)])
            current = (((np.array(ADC_value)/2.0**12*5.0)-0.5)/0.4)*current_gain_all + current_offset_all
            current = np.array(map(lambda x:x if x>0 else 0.001, current))
            voltage = ((np.array(ADC2_value)*5.0/4096.0))*voltage_gain_all + voltage_offset_all
            resistance += voltage / current

        resistance = resistance/float(avg)

        # Limiting initial resistance when channel is opened/high impedance
        resistance = map(lambda x:x if x<MAX_RESISTANCE else MAX_RESISTANCE, resistance)

        DAC.write([0]*8, [i for i in range(8)])

        answer = []
        for i in channels:
            answer.append(resistance[i])

        return answer


    @staticmethod
    # Request new initial temperatures to MBTemp board
    def GetTemperature(channels):

        new_t0 = MBTemp.ReadTemp_All()
        for i in range(8):
            new_t0[i] = ((-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-(0.388*new_t0[i] + 100.03))))))/(2*Pt100_A)) #(-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-res)))))/(2*Pt100_A))

        answer = []
        for i in channels:
            answer.append(new_t0[i])

        return answer

# --------------------- MAIN LOOP ---------------------
# -------------------- starts here --------------------

signal.signal(signal.SIGINT, signal_handler) # Ctrl-C signal

# Init devices and ensures alpha = 900
if PT100:
    MBTemp.begin(temperature_ID)
    if(MBTemp.ReadAlpha() != 900):
        MBTemp.WriteAlpha(900)


GPIO.setup(BUTTON, GPIO.IN)
GPIO.setup(LED, GPIO.OUT)
GPIO.output(LED, GPIO.LOW)


# ----- Get initial Temp
sys.stdout.write(time_string() + "Initializing MBTemp.\n")
sys.stdout.flush()
t0 = MBTemp.ReadTemp_All()
for i in range(8):
    t0[i] = ((-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-(0.388*t0[i] + 100.03))))))/(2*Pt100_A)) #(-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-res)))))/(2*Pt100_A))
sys.stdout.write(time_string() + "MBTemp initialized.\n")
sys.stdout.flush()


# ----- Get inital tape resistance
r0 = measure.GetResistance([i for i in range(8)], 50)


# ----- Clear and resize variables before init
sys.stdout.write(time_string() + "Reset variables.\n")
sys.stdout.flush()
resetVariables()


# ----- Create System Events
new_measure = Event()           # Set when beggin new channel period
run_control = Event()           # If set, run the control
end_control = Event()           # If set, exit program

# ----- Start communication thread
sys.stdout.write(time_string() + "Starting Communication Thread.\n")
sys.stdout.flush()
net = Communication(5000)
net.daemon = True
net.start()

# ----- Start thread to read/write ADC/DAC
#thread = IO_control(end_control, 1.0/UP_FREQ)
sys.stdout.write(time_string() + "Starting Control Thread.\n")
sys.stdout.flush()
thread = IO_control(run_control, 1.0/UP_FREQ)
thread.daemon = True
thread.start()

# If synchronized ramp, wait trigger
#run_control.set()
run_control.wait()
end_control.clear()


# Main loop
while not end_control.isSet():  # While end_flag is not set
    if run_control.isSet() and GPIO.input(BUTTON):     # If should run

        new_measure.wait()      # Wait until new measure arrives

        # Calculate voltage and current (values must be > 0)
        current = (((np.array(ADC_value)/2.0**12*5.0)-0.5)/0.4)*current_gain + current_offset
        current = np.array(map(lambda x:x if x>0 else 0.001, current))
        voltage = ((np.array(ADC2_value)*5.0/4096.0))*voltage_gain + voltage_offset
        voltage = np.array(map(lambda x:x if x>0 else 0.001, voltage))

        resistance = (voltage / current)
        # Limiting initial resistance when channel is opened/high impedance
        resistance = map(lambda x:x if x<MAX_RESISTANCE else MAX_RESISTANCE, resistance)

        power = current * voltage

        new_measure.clear()

#        print "Tensao: ", voltage
#        print "Corrente: ", current
#        print "Potencia: ", power
#        print "Resistencia ", resistance
#        print "Current limit: ", CurrentLimit
#        print "Temperatura inicial ", init_temperature
#        print "Resistencia inicial ", init_resistance
#        print(resistance_cable)

        # Temperature - Pt100
        res = MBTemp.ReadTemp_All()
        for index, channel in enumerate(channel_active):
            temp_Pt100[index] = (-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-(0.388*res[channel] + 100.03))))))/(2*Pt100_A) #(-Pt100_B + (np.sqrt(Pt100_B*Pt100_B - (4*Pt100_A*(Pt100_R0-res)))))/(2*Pt100_A);

        # Temperature - Tape
        temp_tape_read = (init_temperature + (((resistance - resistance_cable)/(init_resistance - resistance_cable)-1)*temp_coef))
        # ----- If temperature difference from three measures (count on temp_tape_difference) to previous one, skip mean value calc (it will take long to get to correct value) -----
        for index, channel in enumerate(channel_active):
            if abs(temp_tape_read[index] - temp_tape[index]) > 15:
                temp_tape_difference[index] += 1
                if temp_tape_difference[index] == 3:
                    temp_tape[index] = temp_tape_read[index]
                    temp_tape_difference[index] = 0
            else:
                temp_tape[index] = alpha_temp*temp_tape[index] + (1-alpha_temp)*temp_tape_read[index]
                temp_tape_difference[index] = 0
        temp_tape = np.array(map(lambda x:x if x>0 else 0.0, temp_tape))

        # Temperatures - According to configuration
        for index, channel in enumerate(channel_active):
            if channel in channel_via_Pt100:
                temperature[index] = temp_Pt100[index]
            else:
                temperature[index] = temp_tape[index]

#        print "Temp Pt100 ", temp_Pt100
#        print "Temp Fita ", temp_tape
#        print "Temperatura ", temperature

        # If PID control is set - PID_CV is the POWER applied to the tape
        if AUTO:
            PID_error[2] = PID_error[1]
            PID_error[1] = PID_error[0]
            PID_error[0] = PID_setpoint - temperature

            old_PID_CV = PID_CV
            PID_CV = PID_CV + np.dot(np.array(PID_value),np.array(PID_error)) # [PID_a, PID_b, PID_c] * [Error[ch0], Error[ch1]...]


            # Power limiting
            PID_CV = map(lambda x:x if x>0 else 0, PID_CV)
            PID_CV = map(lambda x:x if x<MAX_POWER else MAX_POWER, PID_CV)

            # Voltage limiting - If in MAX_VOLTAGE, it is not possible to increase output power
            for index, channel in enumerate(channel_active):
                if (voltage[index] > MAX_VOLTAGE) and (PID_CV[index] > old_PID_CV[index]):
                    PID_CV[index] = old_PID_CV[index]

            # Voltage limiting - If in MIN_VOLTAGE, it is not possible to decrease voltage
            for index, channel in enumerate(channel_active):
                if (voltage[index] < MIN_VOLTAGE) and (PID_CV[index] < old_PID_CV[index]):
                     PID_CV[index] = old_PID_CV[index]


            # Acts only on enabled channels (it is possible to turn off each one individually)
            for index, channel in enumerate(channel_active):
                if channel in EnabledChannels:
                    # Converts power into voltage (using current), and then into 0-5V DAC 12-bit value (0-4095)
                    if resistance[index] < 200.0:
                        power2DAC = (np.sqrt(PID_CV[index]*resistance[index])/9.6)*(4096/5)
                    else:
                        power2DAC = (np.sqrt(PID_CV[index]*init_resistance[index])/9.6)*(4096/5)

                    if power2DAC > MAX_DAC:
                        power2DAC = MAX_DAC

                    DAC_value[index] = int(np.rint(power2DAC))


                    # Adjusting DAC output to reasonable values
                    if DAC_value[index] < MIN_DAC:
                        DAC_value[index] = MIN_DAC
                        PID_CV[index] = np.power((48.0*MIN_DAC/4096.0),2)/resistance[index] # New PID_CV value, due to DAC_VALUE change.

                    # Limiting current output - MANDATORY OVER OTHER ADJUSTS - Adjusting new values for PID_CV and DAC_value if there is a limit for Iout
                    # If current exceeds and PID_CV increses, limit it.
                    # If current exceeds by more than 10% and PID_CV decreases, limit it.
                    if CurrentLimit[index] > 0.0 and ((current[index] > CurrentLimit[index] and PID_CV[index] > old_PID_CV[index]) or (current[index] > 1.1*CurrentLimit[index] and PID_CV[index] < old_PID_CV[index])):
                        PID_CV[index] = resistance[index]*CurrentLimit[index]*CurrentLimit[index]
                        DAC_value[index] = int(np.rint((np.sqrt(PID_CV[index]*resistance[index])/9.6)*(4096/5)))


                else:
                    # If not enabled, clear DAC and sets initial power
                    DAC_value[index] = int(0)
                    PID_CV[index] = np.power((48*MIN_DAC/4096.0),2)/init_resistance[index]

#            print "Current limit: ", CurrentLimit
#            print "Setpoint: ", PID_setpoint
#            print "PID_CV: ", PID_CV
#            print "Valor para o DAC: ", DAC_value

        # If Interpolation/Curve is set
        if INTERPOL:
            for channel in channel_active:
                channel = channel_active.index(channel)
                if len(Curve[channel]) >= 4:
                    # Get points
                    t1 = Curve[channel][0]
                    t2 = Curve[channel][2]
                    temp1 = Curve[channel][1]
                    temp2 = Curve[channel][3]
                    i = 2
                    while(t2 < periods[channel]/UP_FREQ):
                        if 2*i == len(Curve[channel]):
                            break
                        else:
                            t1 = t2
                            temp1 = temp2
                            t2 = Curve[channel][2*i]
                            temp2 = Curve[channel][2*i + 1]
                            i += 1

                    # Interpolation - interp(t, delta_t, delta_temp)
                    PID_setpoint[channel] = np.interp(periods[channel]/UP_FREQ, [t1, t2], [temp1, temp2])

    if GPIO.input(BUTTON) == 0:
        DAC.write([0]*8, channel_active)


# if Ctrl+C or end signal, finish the program
sys.stdout.write(time_string() + "Turning off outputs.\n")
sys.stdout.flush()

DAC.write([0]*8, channel_active)
GPIO.output(LED, GPIO.LOW)
