import time
import board
import busio
import math
import pulseio
import array
import sys
import struct
from adafruit_binascii import hexlify
import os
import microcontroller
import digitalio
import analogio


time.sleep(1)

analogin = analogio.AnalogIn(board.A2)

voltage = (analogin.value * 3.3) / 65536
if voltage > 2:
    print('not executing the code with USB connected')  #has to be also chnage in boot.py
                                                        #to execute the code in USB mode
    sys.exit()

RX = board.RX
TX = board.TX

uart_gps = busio.UART(TX, RX, baudrate=9600, timeout=1, receiver_buffer_size=128)

Change_uart_baudrate_38400 = bytes ([
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x54, #CFG-PRT
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x54, #CFG-PRT
    ])
uart_gps.write(Change_uart_baudrate_38400)

uart_gps.deinit()

uart_gps = busio.UART(TX, RX, baudrate=38400, timeout=1)

class UbxStream():
    def __init__(self, uart):
        # pyserial 3.x has min requirement python2.7
        # read() returns string in 2.7, bytes object otherwise
        self.buff  = bytearray(98)
        self._uart = uart
        self._ubox_synch = ['b5', '62']
        self.iTOW = self.year = self.month = self.day = self.hour = self.minute = ""
        self.second = self.valid = self.tAcc = self.nano = self.fixType = self.flags = ""
        self.flags2 = self.numSV = self.lon = self.lat = self.height = self.hMSL = self.hAcc = ""
        self.vAcc = self.velN = self.velE = self.velD = self.gSpeed = self.headMot = self.sAcc = ""
        self.headAcc = self.pDOP = self.headVeh = self.magDec = self.magAcc = ""

    def read(self, timeout=1, reset=True):
        if(reset):
            self._uart.reset_input_buffer()
            s1 = self._uart.read(1)
            if not s1 == None:
                s1 = hexlify(s1).decode('utf-8')
                if s1 == self._ubox_synch[0]:
                    s2 = self._uart.read(1)
                    s2 = hexlify(s2).decode('utf-8')
                    if s2 == self._ubox_synch[1]:
                        self._uart.readinto(self.buff)
                        ubx_class = hexlify(bytes([self.buff[0]])).decode('utf-8')
                        if ubx_class == '01':
                            ubx_id = hexlify(bytes([self.buff[1]])).decode('utf-8')
                            if ubx_id == '07':
                                return self.ubx_NAV_PVT()

    def ubx_NAV_PVT(self):
        if(self.validate_checksum(self.buff)):
            buff_cpy = self.buff[4:96]
            self.iTOW, self.year, self.month, self.day, self.hour, self.minute, self.second, self.valid, self.tAcc, self.nano, self.fixType, self.flags, self.flags2, self.numSV, self.lon, self.lat, self.height, self.hMSL, self.hAcc, self.vAcc, self.velN, self.velE, self.velD, self.gSpeed, self.headMot, self.sAcc, self.headAcc, self.pDOP, reserved11, reserved12, reserved13, reserved14, reserved15, reserved16,  self.headVeh, self.magDec, self.magAcc = struct.unpack('LH5BBLlB2BB4l2L5lLLH6BlhH', buff_cpy)
            return True
        else:
            return False

    def validate_checksum(self, buff):
        check1 = 0
        check2 = 0
        chk1 = buff[96]
        chk2 = buff[97]

        for i in range(0, len(buff)-2):
            check1 = (check1 + buff[i]) % 256
            check2 = (check1 + check2) % 256
        if chk1==check1 and chk2==check2:
            return True
        else:
            return False


uBX_nav_pvt_msg = UbxStream(uart_gps)
filefullpath = ''

############################################################
##################### START OF THE SETUP ###################
############################################################

def setup():

    uBX_nav_pvt_msg = UbxStream(uart_gps)
    global filefullpath

    Disable_NMEA = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, # GxGGA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, # GxGLL
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, # GxGSA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, # GxGSV
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, # GxRMC
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, # GxVTG
        ])
    uart_gps.write(Disable_NMEA)

    time.sleep(1)

    Disable_UBX = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, #NAV-POSLLH
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, #NAV-STATUS
        ])
    uart_gps.write(Disable_UBX)

    time.sleep(1)

    Enable_UBX = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, #NAV-PVT
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, #NAV-PVT
        ])
    uart_gps.write(Enable_UBX)

    time.sleep(1)

    commands2 = bytes ([
        #0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, #(10Hz)
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, #(5Hz)
        ])
    uart_gps.write(commands2)

    while True:
        if uBX_nav_pvt_msg.read():
            foldername = '/{:02d}-{:02d}-{:02d}'.format(uBX_nav_pvt_msg.year - 2000,uBX_nav_pvt_msg.month,uBX_nav_pvt_msg.day)
            filename = '{:02d}-{:02d}-{:02d}.csv'.format(uBX_nav_pvt_msg.hour,uBX_nav_pvt_msg.minute,uBX_nav_pvt_msg.second)
            try:
                os.mkdir(foldername)
                print('folder created')
            except:
                print('folder already exist or memory in read only')

            os.chdir(foldername)
            with open(filename, "a") as f:
                f.write('time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,heading,cAcc,gpsFix,numSV\n')
                f.write(',(deg),(deg),(m),(m/s),(m/s),(m/s),(m),(m),(m/s),(deg),(deg),,\n')
                filefullpath = '{}\{}'.format(foldername,filename)
                return

############################################################
##################### START OF THE LOOP ####################
############################################################
def loop():
    global filefullpath
    trigger = False
    writeInFileTrigger = True
    counter = 0
    with open(filefullpath, "a") as f:
        while True:
            if uBX_nav_pvt_msg.read():
                bufferMilliseconds = round(uBX_nav_pvt_msg.nano / 100000000)*10

                FlySightString = '{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:02}Z,{:010.7f},{:010.7f},{:.3f},{:.2f},{:.2f},{:.2f},{:.3f},{:.3f},{:.2f},{:.5f},{:.5f},{},'.format(
                    uBX_nav_pvt_msg.year,uBX_nav_pvt_msg.month,uBX_nav_pvt_msg.day,
                    uBX_nav_pvt_msg.hour,uBX_nav_pvt_msg.minute,uBX_nav_pvt_msg.second,
                    bufferMilliseconds,(uBX_nav_pvt_msg.lat/10000000),(uBX_nav_pvt_msg.lon/10000000),
                    (uBX_nav_pvt_msg.hMSL/1000),(uBX_nav_pvt_msg.velN/1000),(uBX_nav_pvt_msg.velE/1000),
                    (uBX_nav_pvt_msg.velD/1000),(uBX_nav_pvt_msg.hAcc/10000),(uBX_nav_pvt_msg.vAcc/10000),
                    (uBX_nav_pvt_msg.sAcc/10000),(uBX_nav_pvt_msg.headMot/100000),(uBX_nav_pvt_msg.headAcc/100000),
                    uBX_nav_pvt_msg.fixType)

                if uBX_nav_pvt_msg.numSV > 20:
                  FlySightString += "20\n"
                else:
                  FlySightString += '{}\n'.format(uBX_nav_pvt_msg.numSV)

                f.write(FlySightString)

                if int(uBX_nav_pvt_msg.velD/1000) in range (-2, 3):
                    counter += 1
                    if counter >20 and not trigger:
                        trigger = True
                        counter = 0
                if trigger:
                    if int(uBX_nav_pvt_msg.velD) <= 1000:
                        counter += 1
                        if counter >25:
                            f.close()
                            break



############################################################
################ STARTING THE FUNCTIONS ####################
############################################################
setup()
loop()