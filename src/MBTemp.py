if (__name__ == "__main__"):
    exit()

import serial
import time
global connection
global address
connection = None

def begin(addr, port = "/dev/ttyUSB0", spd = 115200):
    # Open serial communication to MBTemp
    global connection, address
    address = addr
    connection = serial.Serial(port, spd, timeout=0.2)

def ReadTemp(channel):
    global address
    msg = [address, 0x10, 0x00, 0x01]
    msg.append(channel)
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 7 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(7)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return (ord(message_received[4])*256 + ord(message_received[5]))/100.0

def ReadAlpha():
    global address
    msg = [address, 0x10, 0x00, 0x01, 0x08]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 7 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(7)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return (ord(message_received[4])*256 + ord(message_received[5]))

def ReadAngularCoef():
    global address
    msg = [address, 0x10, 0x00, 0x01, 0x09]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 7 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(7)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return (ord(message_received[4])*256 + ord(message_received[5]))/100.0

def ReadLinearCoef():
    global address
    msg = [address, 0x10, 0x00, 0x01, 0x0a]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 7 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(7)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return (ord(message_received[4])*256 + ord(message_received[5]))/100.0


def ReadTemp_All():
    global address
    msg = [address, 0x12, 0x00, 0x01, 0x01]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)


    # Send until a valid response is received
    while(len(message_received) != 21 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(21)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    answer = []
    for i in range(8):
        answer.append((ord(message_received[4+2*i])*256 + ord(message_received[5+2*i]))/100.0)
    return answer


def WriteAlpha(alpha):
    global address
    msg = [address, 0x20, 0x00, 0x03, 0x08]
    alpha_bytes = [alpha/256, alpha%256]
    msg += alpha_bytes
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 5 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(5)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return ReadAlpha()

def WriteLerADPuro(mode):
    global address
    msg = [address, 0x20, 0x00, 0x02, 11, mode]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 5 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(5)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return


def ReadLerADPuro():
    global address
    msg = [address, 0x10, 0x00, 0x01, 0x0b]
    message_to_send = ''
    message_received = ""
    cs = 0

    # Prepares message to be sent
    for i in msg:
        message_to_send += chr(i)
        cs += i
    message_to_send += chr((0x100 - (cs % 0x100)) & 0xFF)

    # Send until a valid response is received
    while(len(message_received) != 6 or (cs%0x100) != 0):
        message_received = ""
        cs = 0

        # Send the message to MBTemp
        connection.flushInput()
        connection.write("%s" % message_to_send)
        # Read the message from MBTemp
        message_received = connection.read(6)
        # Verify Checksum
        for i in message_received:
            cs += ord(i)

    return (ord(message_received[4]))

def close():
    connection.close()
