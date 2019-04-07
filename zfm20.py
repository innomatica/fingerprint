#!/usr/bin/env python3

#
# Command Packet: host to device
#
# (HDR:2)(ADDR:4)(PID:1)(PLEN:2)(DATA:x)(CSUM:2)
#
# HDR: header
# ADDR: module address
# PID: packet ID
# PLEN: data size plus checksum
# DATA: data
#

from gpiozero import DigitalInputDevice
import serial
import logging

# packet id
ZFM_PID_CMD = 0x01
ZFM_PID_DAT = 0x02
ZFM_PID_ACK = 0x07
ZFM_PID_EDT = 0x08

# Acknowledge Packet: device to host
#
# (HDR:2)(ADDR:4)(PID:1)(PLEN:2)(ACK:1)(CSUM2)
#
ZFM_ACK_CMD_COMPLETE    = 0x00
ZFM_ACK_ERR_RCVDATA     = 0x01
ZFM_ACK_NO_FINGER       = 0x02
ZFM_ACK_ENROLL_FAIL     = 0x03
ZFM_ACK_CHRGEN_FAIL1    = 0x06
ZFM_ACK_CHRGEN_FAIL2    = 0x07
ZFM_ACK_FNG_MISMATCH    = 0x08
ZFM_ACK_FNG_NOTFOUND    = 0x09
ZFM_ACK_CHRCMB_FAIL     = 0x0a
ZFM_ACK_WRONG_PAGEID    = 0x0b
ZFM_ACK_TMP_READ_ERR    = 0x0c
ZFM_ACK_TMP_UPLD_ERR    = 0x0d
ZFM_ACK_DAT_RECV_ERR    = 0x0e
ZFM_ACK_IMG_UPLD_ERR    = 0x0f
ZFM_ACK_TMP_ERAS_ERR    = 0x10
ZFM_ACK_LIB_ERAS_ERR    = 0x11
ZFM_ACK_IMGGEN_FAIL     = 0x15
ZFM_ACK_WRT_FLSH_ERR    = 0x18
ZFM_ACK_NO_DEF_ERR      = 0x19
ZFM_ACK_INV_REG_NUM     = 0x1a
ZFM_ACK_INV_REG_DAT     = 0x1b
ZFM_ACK_WRONG_PAGE      = 0x1c
ZFM_ACK_COM_PORT_FAIL   = 0x1d
ZFM_NACK                = 0xfe

class ZFM20:

    def __init__(self):

        # serial port
        self.__ser = None
        # touch sensor input
        self.__touch = None
        # device address
        self.__addr = [0xff,0xff,0xff,0xff]

    def __send_packet(self, pid, payload, nret):

        if self.__ser is None:
            return False

        # packet size
        plen = [((len(payload) + 2) >> 8) & 0xff, (len(payload) + 2) & 0xff]
        # checksum
        temp = pid
        for item in plen:
            temp = temp + item
        for item in payload:
            temp = temp + item
        csum = [(temp >> 8) & 0xff, temp & 0xff]

        packet = [0xef, 0x01]               # header
        packet.extend(self.__addr)          # address
        packet.append(pid)                  # packet ID
        packet.extend(plen)                 # packet size
        packet.extend(payload)              # payload
        packet.extend(csum)                 # checksum

        logging.debug('Sending {}'.format(bytes(packet)))
        # send packet
        self.__ser.write(bytes(packet))
        # read response
        res = self.__ser.read(nret + 11)
        logging.debug('Receiving {}'.format(res))

        if res is not None and len(res) == (nret + 11):
            return res[9:9+nret]
        else:
            return None


    def Open(self, pin, port, speed=57600):

        # touch sensor input
        self.__touch = DigitalInputDevice(pin, pull_up=None, active_state=False)

        # open serial port
        self.__ser = serial.Serial(port, speed, timeout=5)

        if self.__ser is not None:
            # check connection
            ret = self.__send_packet(ZFM_PID_CMD, b'\x17\x00', 1)
            logging.info('Open: {}'.format(ret))

            if ret is not None and ret[0] == 0x00:
                return True

        return False


    def Close(self):
        if self.__ser is not None:
            self.__ser.close()


    def CheckFinger(self):
        if self.__touch is None:
            return False

        return self.__touch.is_active


    def GetTemplateCount(self):

        ret = self.__send_packet(ZFM_PID_CMD, b'\x1d', 3)
        if ret is not None and ret[0] == 0x00:
            return ret[1] * 256 + ret[2]
        else:
            return 0

    def Capture(self):

        ret = self.__send_packet(ZFM_PID_CMD, b'\x01', 1)
        if ret == None:
            return ZFM_NACK
        else:
            return ret[0]

    def GenCharacteristic(self, buf):

        if buf == 1:
            # create a characteristic on the first buffer
            ret = self.__send_packet(ZFM_PID_CMD, b'\x02\x01', 1)
        else:
            # create a characteristic on the second buffer
            ret = self.__send_packet(ZFM_PID_CMD, b'\x02\x02', 1)

        if ret == None:
            return ZFM_NACK
        else:
            return ret[0]

    def GenTemplate(self):

        ret = self.__send_packet(ZFM_PID_CMD, b'\x05', 1)

        if ret == None:
            return ZFM_NACK
        else:
            return ret[0]

    def SaveTemplate(self, buf,  page):

        payload = bytes([0x06, (buf) & 0xff, (page>>8) & 0xff, (page) & 0xff])
        ret = self.__send_packet(ZFM_PID_CMD, payload, 1)

        if ret == None:
            return ZFM_NACK
        else:
            return ret[0]

    def ReadTemplate(self, page, buf):

        payload = bytes([0x07, (buf) & 0xff, (page>>8) & 0xff, (page) & 0xff])
        ret = self.__send_packet(ZFM_PID_CMD, payload, 1)

        if ret == None:
            return ZFM_NACK
        else:
            return ret[0]

    def MatchTemplate(self):
        ret = self.__send_packet(ZFM_PID_CMD, b'\x03', 3)

        if ret == None:
            return [ZFM_NACK, 0, 0]
        else:
            return ret


#------------------------------------------------------------------------------
if __name__ == "__main__":

    import time

    logging.basicConfig(level=logging.INFO)

    fscan = ZFM20()

    if fscan.Open(18, '/dev/ttyAMA0'):
        print("Device open")
    else:
        print("Failed to open the device")
        exit()

    tmp_count = fscan.GetTemplateCount()
    print('Template count: {}'.format(tmp_count))

    print("\n\n---------------------------------------------------")
    print("Enrol a new fingerprint")

    for index in range(2):
        print("Ready to capture image...Put your finger on the glass")

        while not fscan.CheckFinger():
            time.sleep(0.2)

        # insert delay to prevent immature scanning
        time.sleep(0.5)
        # scan
        ret = fscan.Capture()

        if ret != ZFM_ACK_CMD_COMPLETE:

            if ret == ZFM_ACK_NO_FINGER:
                print("\t<ERR>Timeout....")
            elif ret == ZFM_NACK:
                print("\t<ERR>Device communication error")
            elif ret == ZFM_ACK_ERR_RCVDATA:
                print("\t<ERR>Error during receiving data")
            elif ret == ZFM_ACK_ENROLL_FAIL:
                print("\t<ERR>Failed to capture fingerprint image")
            else:
                print("\t<ERR>Unknown error:{}".format(ret))
            # fail
            fscan.Close()
            exit()

        print("Fingerprint image captured...Lift the finger")

        while fscan.CheckFinger():
            time.sleep(0.2)

        # generate characteristic
        ret = fscan.GenCharacteristic(index+1)

        if ret != ZFM_ACK_CMD_COMPLETE:
            if ret == ZFM_ACK_ERR_RCVDATA:
                print("\t<ERR>Error during receiving data")
            elif ret == ZFM_ACK_CHRGEN_FAIL1:
                print("\t<ERR>Fingerprint image distorted")
            elif ret == ZFM_ACK_CHRGEN_FAIL2:
                print("\t<ERR>Fingerprint image too small")
            elif ret == ZFM_ACK_IMGGEN_FAIL:
                print("\t<ERR>Image generation error")
            else:
                print("\t<ERR>Unknown error:{}".format(ret))
            # fail
            fscan.Close()
            exit()

        print("Characteristic generated...")

    # generate the template from two characteristics
    print("Creating a template...")

    ret = fscan.GenTemplate()
    if ret != ZFM_ACK_CMD_COMPLETE:
        if ret == ZFM_ACK_ERR_RCVDATA:
            print("\t<ERR>Error during receiving data")
        elif ret == ZFM_ACK_CHRCMB_FAIL:
            print("\t<ERR>Template generation failed")
        else:
            print("\t<ERR>Unknown error:{}".format(ret))
        # fail
        fscan.Close()
        exit()

    # storing the template
    print("Storing a template to location:{}...".format(tmp_count))

    ret = fscan.SaveTemplate(1, tmp_count)
    if ret != ZFM_ACK_CMD_COMPLETE:
        if ret == ZFM_ACK_ERR_RCVDATA:
            print("\t<ERR>Error during receiving data")
        elif ret == ZFM_ACK_WRONG_PAGEID:
            print("\t<ERR>Wrong template page ID")
        elif ret == ZFM_ACK_WRT_FLSH_ERR:
            print("\t<ERR>Faile to write to FLASH")
        else:
            print("\t<ERR>Unknown error:{}".format(ret))
        # fail
        fscan.Close()
        exit()

    print("New fingerprint is enrolled successfully")

    # bring the template back to buffer 2
    print("\n\n---------------------------------------------------")
    print("Bring up the template from location:{}...".format(tmp_count))

    ret = fscan.ReadTemplate(tmp_count, 2)
    if ret != ZFM_ACK_CMD_COMPLETE:
        if ret == ZFM_ACK_ERR_RCVDATA:
            print("\t<ERR>Error during receiving data")
        elif ret == ZFM_ACK_WRONG_PAGEID:
            print("\t<ERR>Wrong template page ID")
        elif ret == ZFM_ACK_TMP_READ_ERR:
            print("\t<ERR>Error during reading template")
        else:
            print("\t<ERR>Unknown error:{}".format(ret))
        # fail
        fscan.Close()
        exit()

    print("Put your finger on the glass for match")

    while not fscan.CheckFinger():
        time.sleep(0.2)

    # insert delay to prevent immature scanning
    time.sleep(0.5)
    # scan
    ret = fscan.Capture()

    if ret != ZFM_ACK_CMD_COMPLETE:

        if ret == ZFM_ACK_NO_FINGER:
            print("\t<ERR>Timeout....")
        elif ret == ZFM_NACK:
            print("\t<ERR>Device communication error")
        elif ret == ZFM_ACK_ERR_RCVDATA:
            print("\t<ERR>Error during receiving data")
        elif ret == ZFM_ACK_ENROLL_FAIL:
            print("\t<ERR>Failed to capture fingerprint image")
        else:
            print("\t<ERR>Unknown error:{}".format(ret))
        # fail
        fscan.Close()
        exit()

    print("Fingerprint image captured...Lift the finger")

    while fscan.CheckFinger():
        time.sleep(0.2)

    # generate characteristic on the buffer 1
    ret = fscan.GenCharacteristic(1)

    if ret != ZFM_ACK_CMD_COMPLETE:
        if ret == ZFM_ACK_ERR_RCVDATA:
            print("\t<ERR>Error during receiving data")
        elif ret == ZFM_ACK_CHRGEN_FAIL1:
            print("\t<ERR>Fingerprint image distorted")
        elif ret == ZFM_ACK_CHRGEN_FAIL2:
            print("\t<ERR>Fingerprint image too small")
        elif ret == ZFM_ACK_IMGGEN_FAIL:
            print("\t<ERR>Image generation error")
        else:
            print("\t<ERR>Unknown error:{}".format(ret))
        # fail
        fscan.Close()
        exit()

    print("Characteristic generated...")

    # compare buffers
    ret = fscan.MatchTemplate()

    if ret[0] == ZFM_ACK_CMD_COMPLETE:
        print("\tFingerprint match: {}".format(ret[1]*256+ret[2]))
    elif ret[0] == ZFM_ACK_FNG_MISMATCH:
        print("\tFingerprint does not match: {}".format(ret[1]*256+ret[2]))
    elif ret[0] == ZFM_NACK:
        print("\t<ERR>Unknown err:{}".format(ret[0]))

    fscan.Close()
