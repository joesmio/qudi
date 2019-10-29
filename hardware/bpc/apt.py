from __future__ import division

import hardware.bpc.aptconsts as c

#import aptconsts as c

import ftd2xx
import time
from struct import pack, unpack, error

# In debug mode we print out all messages which are sent (in hex)
DEBUG_MODE = False


class MessageReceiptError(Exception): pass


class DeviceNotFoundError(Exception): pass


class AptDevice(object):
    """ Wrapper around the Apt protocol via the ftd2xx driver for USB communication with the FT232BM USB peripheral chip in the APT controllers.
   Below is a list of messages defined for all APT devices. Only a small portion of them necessary have been implemented so far taken from the spec
   http://www.thorlabs.com/software/apt/APT_Communications_Protocol_Rev_9.pdf
   The ftd2xx driver specification is also useful:
   http://www.ftdichip.com/Support/Documents/ProgramGuides/D2XX_Programmer's_Guide(FT_000071).pdf"""

    def __init__(self, hwser=None):
        # Find out how many ftd2xx devices are connected to the USB bus
        numDevices = ftd2xx.createDeviceInfoList()
        # Check each device to see if either the serial number matches (if given) or the description string is recognized as valid for the class type
        numMatchingDevices = 0
        for dev in range(numDevices):
            detail = ftd2xx.getDeviceInfoDetail(dev)
            #print(detail)
            #print("Devices include {0} {1}".format(detail['description'].decode(),detail['serial'].decode()))
            if hwser != None and detail["serial"] != "" and int(detail["serial"]) == hwser:
                # Get the first device which matches the serial number if given
                numMatchingDevices += 1
                self.device = device = ftd2xx.open(dev)
                print('Opening device with serial {0} ...'.format(hwser))
                break
            elif hwser == None and (detail["description"].decode() in self.deviceDescriptionStrings()):
                # Get the first device which is valid for the given class if no hwser
                numMatchingDevices += 1
                if numMatchingDevices == 1:
                    self.device = device = ftd2xx.open(dev)
                    print('Opening device')
            elif dev == numDevices - 1 and numMatchingDevices == 0:
                # Raise an exception if no devices were found
                if hwser != None:
                    errorStr = "Hardware serial number " + str(hwser) + " was not found"
                else:
                    errorStr = "No devices found matching class name " + type(
                        self).__name__ + ". Expand the definition of CLASS_STRING_MAPPING if necessary"
                raise DeviceNotFoundError(errorStr)
        # Print a warning message if no serial given and multiple devices were found which matched the class type
        if numMatchingDevices > 1 and hwser == None:
            print(str(numMatchingDevices) + " devices found matching " + type(
                self).__name__ + "; the first device was opened")
        # Inititalize the device according to FTD2xx and APT requirements
        device.setBaudRate(115200)
        device.setDataCharacteristics(ftd2xx.defines.BITS_8, ftd2xx.defines.STOP_BITS_1, ftd2xx.defines.PARITY_NONE)
        self.delay()
        device.purge()
        self.delay()
        device.resetDevice()
        device.setFlowControl(ftd2xx.defines.FLOW_RTS_CTS)
        device.setTimeouts(c.WRITE_TIMEOUT, c.READ_TIMEOUT)
        # Check first 2 digits of serial number to see if it's normal type or card/slot type, and build self.channelAddresses as list of (chanID,destAddress) tuples
        self.channelAddresses = []
        if device.serial[0:2] in c.BAY_TYPE_SERIAL_PREFIXES: # was just if True until 17/12, because piezo is bay type
            # Get the device info
            serNum, model, hwtype, firmwareVer, notes, hwVer, modState, numCh = \
            self.query(c.MGMSG_HW_REQ_INFO, c.MGMSG_HW_GET_INFO, destID=c.RACK_CONTROLLER_ID)[-1]
            # Check each bay to see if it's enabled and also request hardware info
            for bay in range(numCh):
                bayId = c.ALL_BAYS[bay]
                #bayId = 0x21
                self.writeMessage(c.MGMSG_HW_NO_FLASH_PROGRAMMING, destID=bayId)
                if self.BayUsed(bay):
                    bayInfo = self.query(c.MGMSG_HW_REQ_INFO, c.MGMSG_HW_GET_INFO, destID=bayId)[-1]
                    self.channelAddresses.append((c.CHANNEL_1, bayId))
        else:
            # Otherwise just build a list of the channel numbers


            # This message about no flash programming must be sent first
            self.writeMessage(c.MGMSG_HW_NO_FLASH_PROGRAMMING,destID=0x50)

            #print('querying about the device')
            serNum, model, hwtype, firmwareVer, notes, hwVer, modState, numCh = \
            self.query(c.MGMSG_HW_REQ_INFO, c.MGMSG_HW_GET_INFO,param1=0, destID=0x50) [-1]

            for channel in range(numCh):
                self.channelAddresses.append((c.ALL_CHANNELS[channel], c.GENERIC_USB_ID))
        # for channel in range(len(self.channelAddresses)):
        #     print('writing channel enable for channel {0}'.format(channel))
        #     self.writeMessage(c.MGMSG_MOD_SET_CHANENABLESTATE, 0, c.CHAN_ENABLE_STATE_ENABLED, c.RACK_CONTROLLER_ID)
        #     self.EnableHWChannel(channel)

        # chanstate = self.query(c.MGMSG_MOD_REQ_CHANENABLESTATE, c.MGMSG_MOD_GET_CHANENABLESTATE, param1=1, destID=0x21)
        #
        # print(chanstate)
        #
        # chanstate = self.query(c.MGMSG_MOD_REQ_CHANENABLESTATE, c.MGMSG_MOD_GET_CHANENABLESTATE, param1=2, destID=0x21)
        #
        # print(chanstate)
        #
        # chanstate = self.query(c.MGMSG_MOD_REQ_CHANENABLESTATE, c.MGMSG_MOD_GET_CHANENABLESTATE, param1=3, destID=0x21)
        #
        # print(chanstate)

        # Set the controller type
        self.controllerType = model.decode().replace("\x00", "").strip()
        # Print a message saying we've connected to the device successfuly
        print("Connected to %s with %d channel(s). Information: %s" % (
        model.decode().replace('\x00', ''), numCh, notes.decode().replace('\x00', '')))



    def __del__(self):
        self.device.close()

    def writeMessage(self, messageID, param1=0, param2=0, destID=c.RACK_CONTROLLER_ID, sourceID=c.HOST_CONTROLLER_ID,
                     dataPacket=None):
        """ Send message to device given messageID, parameters 1 & 2, destination and sourceID ID, and optional data packet,
        where dataPacket is an array of numeric values. The method converts all the values to hex according to the protocol
        specification for the message, and sends this to the device."""



        if dataPacket != None:
            # If a data packet is included then header consists of concatenation of: messageID (2 bytes),number of bytes in dataPacket (2 bytes), destination byte with MSB=1 (i.e. or'd with 0x80), sourceID byte

            dataPacketStr = pack(c.getPacketStruct(messageID), *dataPacket)
            #print('packed data to write {0}'.format(dataPacketStr))
            try:
                dataPacketStr = pack(c.getPacketStruct(messageID), *dataPacket)
                #print('packed data to write {0}'.format(dataPacketStr))
            except error as e:
                raise Exception( "Error packing message " + hex(
                    messageID) + "; probably the packet structure is recorded incorrectly in c.getPacketStruct()")
            message = pack(c.HEADER_FORMAT_WITH_DATA, messageID, len(dataPacketStr), destID | 0x80,
                           sourceID) + dataPacketStr
        else:
            # If no data packet then header consists of concatenation of: messageID (2 bytes),param 1 byte, param2 bytes,destination byte, sourceID byte
            message = pack(c.HEADER_FORMAT_WITHOUT_DATA, messageID, param1, param2, destID, sourceID)
        if DEBUG_MODE: self.disp(message, "TX:  ")
        numBytesWritten = self.device.write(message)

    def query(self, txMessageID, rxMessageID, param1=0, param2=0, destID=c.RACK_CONTROLLER_ID,
              sourceID=c.HOST_CONTROLLER_ID, dataPacket=None, waitTime=None):
        """ Sends the REQ query message given by txMessageID, and then retrieves the GET response message given by rxMessageID from the device.
        param1,param2,destID,and sourceID for the REQ message can also be specified if non-default values are required.
        The return value is a 7 element tuple with the first 6 values the messageID,param1,param2,destID,sourceID from the GET message header
        and the final value of the tuple is another tuple containing the values of the data packet, or None if there was no data packet.
        A wait parameter can also be optionally specified (in seconds) which introduces a waiting period between writing and reading """
        #print('Destination id {0}'.format(destID))
        self.writeMessage(txMessageID, param1, param2, destID, sourceID, dataPacket)
        if waitTime != None:
            # Keep reading the response until the query timeout is exceeded if wait flag specified
            t0 = time.time()
            while True:
                try:
                    response = self.readMessage()


                    break
                except MessageReceiptError:
                    if time.time() - t0 > waitTime / 1000: raise
        else:
            # Otherwise just wait for the ordinary read timeout
            response = self.readMessage()



        # Check that the received message is the one which was expected

        if response[0] != rxMessageID:
            # if response[0] is 0x1:
            #     #pass through this strange method that sometimes happens JS 09/01/19
            #     return response
            # raise MessageReceiptError("Error querying apt device when sending messageID " + hex(
            #      txMessageID) + ".... Expected to receive messageID " + hex(rxMessageID) + " but got " + hex(response[0]))
            print("Received message ID {0} instead of {1}, will read again...".format(hex(response[0]),hex(rxMessageID)))

            response = self.readMessage()

            if response[0] != rxMessageID:
                print("Received message ID {0} instead of {1}, will now query again...".format(hex(response[0]),
                                                                                          hex(rxMessageID)))
                self.query(txMessageID, txMessageID, rxMessageID, param1=param1, param2=param2,
                           destID=destID,sourceID=sourceID, dataPacket=dataPacket, waitTime=waitTime)
        return response

    def readMessage(self):
        """ Read a single message from the device and return tuple of messageID, parameters 1 & 2, destination and sourceID ID, and data packet
        (if included), where dataPacket is a tuple of all the message dependent parameters decoded from hex,
        as specified in the protocol documentation. Normally the user doesn't need to call this method as it's automatically called by query()"""
        # Read 6 byte header from device

        #print('Reading message')
        headerRaw = self.device.read(c.NUM_HEADER_BYTES)
        if headerRaw.decode(errors='replace') == "": raise MessageReceiptError("Timeout reading from the device")
        # Check if a data packet is attached (i.e. get the 5th byte and check if the MSB is set)

        isDataPacket = headerRaw[4] >> 7
        # Read data packet if it exists, and interpret the message accordingly
        if isDataPacket:
            header = unpack(c.HEADER_FORMAT_WITH_DATA, headerRaw)
            messageID = header[0]
            dataPacketLength = header[1]
            param1 = None
            param2 = None
            destID = header[2]
            sourceID = header[3]
            #print('Source of message {0}'.format(hex(sourceID)))


            destID = destID & 0x7F

            #print('Destination of message {0}'.format(destID))
            dataPacketRaw = self.device.read(dataPacketLength)
            if DEBUG_MODE: self.disp(headerRaw + dataPacketRaw, "RX:  ")
            try:
                dataPacket = unpack(c.getPacketStruct(messageID), dataPacketRaw)
            except error as e:
                # If an error occurs, it's likely due to a problem with the manual inputted data for packet structure in aptconsts
                raise
        else:
            if DEBUG_MODE: self.disp(headerRaw, "RX:  ")
            header = unpack(c.HEADER_FORMAT_WITHOUT_DATA, headerRaw)
            messageID = header[0]
            param1 = header[1]
            param2 = header[2]
            destID = header[3]
            sourceID = header[4]
            dataPacket = None
        # Return tuple containing all the message parameters
        return (messageID, param1, param2, destID, sourceID, dataPacket)

    def delay(self, delayTime=c.PURGE_DELAY):
        """ Sleep for specified time given in ms """
        time.sleep(delayTime / 1000)

    def disp(self, s, prefixStr="", suffixStr=""):
        """ Convenience method to give the hex for a raw string """
        #print(s)
        dispStr = prefixStr + str([hex(ord(c)) for c in s.decode(errors='replace') ]) + suffixStr
        print(dispStr)

    def BayUsed(self, bayId):
        """ Check if the specified bay is occupied """
        response = self.query(c.MGMSG_RACK_REQ_BAYUSED, c.MGMSG_RACK_GET_BAYUSED, bayId, 0, c.RACK_CONTROLLER_ID)
        state = response[2]
        if state == c.BAY_OCCUPIED:
            return True
        elif state == c.BAY_EMPTY:
            return False
        else:
            raise MessageReceiptError("Invalid response from MGMSG_RACK_REQ_BAYUSED")

    def EnableHWChannel(self, channel=0):
        """ Sent to enable the specified drive channel. """
        channelID, destAddress = self.channelAddresses[channel]
        self.writeMessage(c.MGMSG_MOD_SET_CHANENABLESTATE, channelID, c.CHAN_ENABLE_STATE_ENABLED, destAddress)

    def DisableHWChannel(self, channel=0):
        """ Sent to disable the specified drive channel. """
        channelID, destAddress = self.channelAddresses[channel]
        self.writeMessage(c.MGMSG_MOD_SET_CHANENABLESTATE, channelID, c.CHAN_ENABLE_STATE_DISABLED, destAddress)


class _AptMotor(AptDevice):
    """ Wrapper around the messages of the APT protocol specified for motor controller. The method names (and case) are set the same as in the Thor Labs ActiveX control for compatibility

    !!!! TODO: These are no longer directly compatible with ActiveX control due to the mapping of channel onto destId via self.channelAddresses, therefore it makes more sense to use a cleaner syntax here without
    worrying about compatibility, and if needed make a AptMotorWrapper(AptMotor) class which gives versions with identical names. This will prevent cluttering of the namespace as well"""

    def __init__(self, stageType=c.DEFAULT_STAGE_TYPE, *args, **kwargs):
        super(_AptMotor, self).__init__(*args, **kwargs)
        """
        ThorLabs APT ActiveX control does the following on initialization of DRV001 stage with BSC203 controller
        MGMSG_MOT_SET_VELPARAMS  -> 13,04,0E,00,A1,01,01,00,00,00,00,00,BC,8C,00,00,F0,EB,3D,05
        MGMSG_MOT_SET_JOGPARAMS  ->  16,04,16,00,A1,01,01,00,02,00,00,A0,00,00,F7,14,00,00,97,11,00,00,F8,F5,9E,02,02,00
        MGMSG_MOT_SET_LIMSWITCHPARAMS -> 23,04,10,00,A1,01,01,00,03,00,01,00,00,80,25,00,00,80,0C,00,01,00
        MGMSG_MOT_SET_POWERPARAMS  -> 26,04,06,00,A1,01,01,00,0F,00,1E,00
        MGMSG_MOT_SET_GENMOVEPARAMS  ->  3A,04,06,00,A1,01,01,00,00,20,00,00
        MGMSG_MOT_SET_HOMEPARAMS  ->  40,04,0E,00,A1,01,01,00,02,00,01,00,F8,F5,9E,02,00,80,25,00
        MGMSG_MOT_SET_MOVERELPARAMS  ->  45,04,06,00,A1,01,01,00,00,0A,00,00
        MGMSG_MOT_SET_MOVEABSPARAMS  -> 50,04,06,00,A1,01,01,00,00,00,00,00
        MGMSG_MOD_SET_CHANENABLESTATE -> 10,02,02,01,50,01
        # Repeat for channel 2
        """
        # TODO: bring stageType into the constructor
        self.stageType = stageType
        # Home each channel
        # for c in range(len(self.channelAddresses)):
        #    self.MoveHome(channel=c)

    def __del__(self):
        for ch in range(len(self.channelAddresses)):
            self.LLMoveStop(ch)
        return super(_AptMotor, self).__del__()

    def SetLimitSwitchParams(self, channel):
        channelID, destAddress = self.channelAddresses[channel]

        #MGMSG_MOT_SET_LIMSWITCHPARAMS -> 23,04,10,00,A1,01,01,00,03,00,01,00,00,80,25,00,00,80,0C,00,01,00

        '''
        23, 04, 10, 00, A1, 01, \ Header (channel 1)
        01, 00, \ Channel 1
        03, 00, \ CW Hard limit switch breaks
        01, 00, \ CCW Hard limit switch not present
        00, 80, 25, 00,\ CW Soft limit in steps
        00, 80, 0C, 00,\ CCW Soft limit in steps 
        01, 00  Soft limit mode: ignore limit
        '''

        # Difference in 6400 steps between soft limit and hard limit
        #Bit order is reversed?
        cw_hardlimit = 0x0003
        ccw_hardlimit = 0x0001
        cw_softlimit = 0x00258000 # 32 bit unsigned long bits in backwards
        ccw_softlimit = 0x000C8000 # 32 bit unsigned long 819200
        softlimit_mode = 0x0003 #0x0100

        packet = (channelID, cw_hardlimit, ccw_hardlimit, cw_softlimit, ccw_softlimit, softlimit_mode)

        print('for limitswitch {0}'.format(packet))

        print('expecting {0}'.format(c.MGMSG_MOT_SET_LIMSWITCHPARAMS))

        self.writeMessage(c.MGMSG_MOT_SET_LIMSWITCHPARAMS, dataPacket=packet,destID=destAddress)

    def getHomeParams(self):
        channelID, destAddress = self.channelAddresses[0]
        response = self.query(c.MGMSG_MOT_REQ_HOMEPARAMS, c.MGMSG_MOT_GET_HOMEPARAMS, channelID, destID=destAddress)
        decoded = self.decodeHomeParams(response)
        return decoded

    def decodeHomeParams(self,response):

        params = response[-1]

        decoded = {} #"['','','','']
        #(1, 3, 1, 2457600, 819200, 3)

        #packet = (channelID, home_dir, limit_switch, home_vel, offsetdist)
        if params[1] is 1:
            decoded['Home direction'] = 'forward (+ve)'
        if params[1] is 2:
            decoded['Home direction'] = 'reverse (-ve)'
        if params[2] is 1:
            decoded['Hardware limit switch'] = 'reverse'
        if params[2] is 4:
            decoded['Hardware limit switch'] = 'forwards'

        decoded['Home velocity'] = '{0}'.format(self._encToVelocity(params[3]))
        decoded['Home offset from limit'] = '{0}'.format(params[4])
        return decoded


    def decodeLimitParams(self,response):

        params = response[-1]

        decoded = {}
        #(1, 3, 1, 2457600, 819200, 3)

        #packet = (channelID, home_dir, limit_switch, home_vel, offsetdist)

        #cw_hardlimit, ccw_hardlimit, cw_softlimit, ccw_softlimit, softlimit_mode
        if params[1] is 1:
            decoded['CW switch'] = 'ignore'
        if params[1] is 2:
            decoded['CW switch'] ='make'
        if params[1] is 3:
            decoded['CW switch'] = 'break'
        if params[2] is 1:
            decoded['CCW switch'] = 'ignore'
        if params[2] is 2:
            decoded['CCW switch'] = 'make'
        if params[2] is 3:
            decoded['CCW switch'] = 'break'

        #decoded['CW software limit'] = '{0}'.format(self._encToPosition(params[3]))
        decoded['CW software limit'] = '{0}'.format(params[3]/134218)
        decoded['CW software limit (raw)'] = '{0}'.format(params[3])

        #decoded['CCW software limit'] = '{0}'.format(self._encToPosition(params[4]))
        decoded['CCW software limit'] = '{0}'.format(params[4] / 134218)
        decoded['CCW software limit (raw)'] = '{0}'.format(params[4])


        if params[5] is 1:
            decoded['At software limit'] = 'Ignore'
        if params[5] is 2:
            decoded['At software limit'] = 'Stop immediately'
        if params[5] is 3:
            decoded['At software limit'] = 'Profiled stop'

        return decoded


    def getLimitParams(self):
        channelID, destAddress = self.channelAddresses[0]
        response = self.query(c.MGMSG_MOT_REQ_LIMSWITCHPARAMS, c.MGMSG_MOT_GET_LIMSWITCHPARAMS, channelID, destID = destAddress)
        decoded = self.decodeLimitParams(response)
        return decoded

    def setBackslash(self):
        '''
        3A, 04, 06, 00, A1, 01, \
        01, 00,\
        00, 20, 00, 00
        '''

        #Set backslash distance to 32*2^16 = 2.56 mm

    def setHomeParams(self, channel):

        '''
        40,04,0E,00,A1,01,
        01,00,
        02,00, home direction
        01,00, limit switch
        F8,F5,9E,02, home velocity
        00,80,25,00 offset distance: this is the CW soft limit

        '''
        home_dir = 0x0002 # word 1 +ve 2 -VE
        limit_switch = 0x0001 # word

        # Values that are longer than a byte follow the Intel littleendian format.

        # Signed 32 bit integer(4 bytes) in 2’s compliment form
        #4 byte unsigned long


        home_vel = 0x029EF5F8 # 4 byte unsigned long value 43972088
        offsetdist = 0x0025800 # 4 byte signed integer 153600

        channelID, destAddress = self.channelAddresses[channel]

        packet = (channelID, home_dir, limit_switch, home_vel, offsetdist)
        #print('for homeparams {0}'.format(str(packet)))

       # print('expecting {0}'.format(c.MGMSG_MOT_SET_HOMEPARAMS))

        self.writeMessage(c.MGMSG_MOT_SET_HOMEPARAMS, dataPacket=packet,destID=destAddress)

    def GetVelocity(self,channel=0):
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_REQ_VELPARAMS, c.MGMSG_MOT_GET_VELPARAMS, channelID, destID=destAddress)
        velparam = response[-1][-1]
        #print('velocity params encoded as {0}'.format(response))
        return self._encToVelocity(velparam)

    def GetJogParams(self,channel =0):
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_REQ_JOGPARAMS, c.MGMSG_MOT_GET_JOGPARAMS, channelID, destID=destAddress)
        #print('jog params encoded as {0}'.format(response))

        datapacket = response[-1]
        #print('jog params encoded as {0}'.format(response))
        return datapacket

    def SetJogStep(self, dist, channel=0):

        params = list(self.GetJogParams(channel=channel))
        print('Current stepper jog {0} mm'.format(self._encToPosition(params[-5]), params[-5]))
        #print(self._positionToEnc(dist))
        params[-5] = self._positionToEnc(dist)

        self.SetJogParams(tuple(params),channel =channel)


    def SetJogParams(self,packet, channel = 0):

        channelID, destAddress = self.channelAddresses[channel]

        self.writeMessage(c.MGMSG_MOT_SET_JOGPARAMS, dataPacket=packet, destID=destAddress)



    def MoveHome(self, channel=0, wait=True):
        """ Home the specified channel and wait for the homed return message to be returned """
        channelID, destAddress = self.channelAddresses[channel]
        waitTime = c.QUERY_TIMEOUT if wait else None
        response = self.query(c.MGMSG_MOT_MOVE_HOME, c.MGMSG_MOT_MOVE_HOMED, channelID, destID=destAddress,
                              waitTime=waitTime)

    def MoveJog(self, channel=0, direction=c.MOTOR_JOG_FORWARD):
        """ Jog the specified channel in the specified direction and wait for the move completed message to be returned """
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_MOVE_JOG, c.MGMSG_MOT_MOVE_COMPLETED, channelID, direction,
                              destID=destAddress)

    def GetPosition(self, channel=0):
        """ Get the position in mm """
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_REQ_POSCOUNTER, c.MGMSG_MOT_GET_POSCOUNTER, channelID, destID=destAddress)

        if response is 0x01:
            response = self.query(c.MGMSG_MOT_REQ_POSCOUNTER, c.MGMSG_MOT_GET_POSCOUNTER, channelID, destID=destAddress)

        #print('response pos counter at channel ',channel, response)

        try:
            posParam = response[-1][-1]
            return self._encToPosition(posParam)
        except TypeError:
            print('Received response ',response)
            return 'error'

    def GetPositionRaw(self, channel=0):
        """ Get the position in mm """
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_REQ_POSCOUNTER, c.MGMSG_MOT_GET_POSCOUNTER, channelID, destID=destAddress)

        if response is 0x01:
            response = self.query(c.MGMSG_MOT_REQ_POSCOUNTER, c.MGMSG_MOT_GET_POSCOUNTER, channelID, destID=destAddress)

        # print('response pos counter at channel ',channel, response)

        try:
            posParam = response[-1][-1]
            return posParam #self._encToPosition(posParam)
        except TypeError:
            print('Received response ', response)
            return 'error'



    def UpdateEncoder(self, channel =0, position = 0):
        channelID, destAddress = self.channelAddresses[channel]

        waitTimeParam =  None
        posParam = self._positionToEnc(position)

        print('Asking to update encoder to (4 byte signed integer)', posParam)
        response = self.writeMessage(c.MGMSG_MOT_SET_POSCOUNTER,destID=destAddress,
                              dataPacket=(channelID, posParam))



    def MoveAbsoluteEnc(self, channel=0, positionCh1=0.0, positionCh2=0, waitTime=c.QUERY_TIMEOUT, wait=True):
        """ Move the specified channel to the specified absolute position and wait for the move completed message to be returned """
        channelID, destAddress = self.channelAddresses[channel]
        position = positionCh1
        waitTimeParam = waitTime if wait else None
        posParam = self._positionToEnc(position)

        print('Asking to move to position in encoder counts (4 byte signed integer)', posParam)
        response = self.query(c.MGMSG_MOT_MOVE_ABSOLUTE, c.MGMSG_MOT_MOVE_COMPLETED, 0x06, destID=destAddress,
                              dataPacket=(channelID, posParam), waitTime=waitTimeParam)



    def MoveAbsoluteEx(self, channel=0, positionCh1=0.0, positionCh2=0, wait=True):
        """ Wrapper for MoveAbsoluteEx """
        self.MoveAbsoluteEnc(channel, positionCh1, positionCh2, wait=wait)

    def GetStageAxisInfo(self, channel=0):
        """ Get the stage axis info... doesn't seem to be working right now """
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_MOT_REQ_PMDSTAGEAXISPARAMS, c.MGMSG_MOT_GET_PMDSTAGEAXISPARAMS, channelID,
                              destID=destAddress)
        dataPacket = response[-1]
        return dataPacket

    def LLMoveStop(self, channel=0):
        """ Send the stop signal... c.MGMSG_MOT_MOVE_COMPLETED may be returned here if the stage was moving """
        # TODO: deal with the return message
        channelID, destAddress = self.channelAddresses[channel]
        self.writeMessage(c.MGMSG_MOT_MOVE_STOP, channelID, destID=destAddress)
        pass

    def _positionToEnc(self, position):
        """ convert between position in mm (or angle in degrees where applicable) and appropriate encoder units"""

        #print('asked for position', position)
        #print('scaling factors', c.getMotorScalingFactors(self.controllerType, self.stageType)["position"])
        return round(position * c.getMotorScalingFactors(self.controllerType, self.stageType)["position"])

    def _encToVelocity(self,enc):
        return enc / c.getMotorScalingFactors(self.controllerType, self.stageType)["velocity"]

    def _encToPosition(self, enc):
        """ convert between position in mm (or angle in degrees where applicable) and appropriate encoder units"""
        return enc / c.getMotorScalingFactors(self.controllerType, self.stageType)["position"]

    def _velocityToEnc(self, velocity):
        """ convert between velocity in mm/s (angular in degrees/s where applicable) and appropriate encoder units"""
        return round(velocity * c.getMotorScalingFactors(self.controllerType, self.stageType)["velocity"])

    def _accelerationToEnc(self, acceleration):
        """ convert between acceleration in mm/s/s (angular in degrees/s/s where applicable) and appropriate encoder units"""
        return round(acceleration * c.getMotorScalingFactors(self.controllerType, self.stageType)["acceleration"])

import numpy as np

class _AptPiezo(AptDevice):
    """ Wrapper around the messages of the APT protocol specified for piezo controller. The method names (and case) are set the same as in the Thor Labs ActiveX control for compatibility

    !!!! TODO: These are no longer directly compatible with ActiveX control due to the mapping of channel onto destId via self.channelAddresses, therefore it makes more sense to use a cleaner syntax here without
    worrying about compatibility, and if needed make a AptPiezoWrapper(AptPiezo) class which gives versions with identical names. This will prevent cluttering of the namespace as well"""

    def __init__(self, hwser=None):
        super(_AptPiezo, self).__init__(hwser)

        self.maxVoltage = 75 #self.GetMaxOPVoltage()  # for some unknown reason our device isn't responding to self.GetMaxOPVoltage()
        self.maxExtension = 20 #self.GetMaxTravel()
        print('Max extension is {0}'.format(self.GetMaxTravel()))
        for ch in range(len(self.channelAddresses)):
           self.SetControlMode(ch,c.PIEZO_CLOSED_LOOP_MODE)
           self.SetPosOutput(ch)
           self.initializeConstants(ch)
            #If we wanna receive status update messages then we need to send MGMSG_HW_START_UPDATEMSGS
            #We would additionally need to send server alive messages every 1s, e.g. MGMSG_PZ_ACK_PZSTATUSUPDATE for Piezo
            #However if we don't need broadcasting of the position etc we can just fetch the status via GET_STATUTSUPDATES



    def initializeConstants(self, channel=0):
        """ Hack to initialize the pieze controller with constants defined explicitly in aptconsts.
        # TO DO : Make the constants model specific """
        channelID, destAddress = self.channelAddresses[channel]
        # self.writeMessage(c.MGMSG_MOD_SET_DIGOUTPUTS , 0, 0x59,destAddress)
        #  Thor Labs are doing this, but I have no idea if it's necessary, or what 0x59 is since this is supposed to be 0
        self.writeMessage(c.MGMSG_PZ_SET_NTMODE, 0x01)
        self.writeMessage(c.MGMSG_PZ_SET_INPUTVOLTSSRC, 0x04, destID=destAddress,
                          dataPacket=(channelID, c.PIEZO_INPUT_VOLTS_SRC_SW))
        self.writeMessage(c.MGMSG_PZ_SET_PICONSTS, 0x06, destID=destAddress,
                          dataPacket=(channelID, c.PIEZO_PID_PROP_CONST, c.PIEZO_PID_INT_CONST))
        self.writeMessage(c.MGMSG_PZ_SET_IOSETTINGS, 0x0A, destID=destAddress, dataPacket=(
        channelID, c.PIEZO_AMP_CURRENT_LIM, c.PIEZO_AMP_LP_FILTER, c.PIEZO_AMP_FEEDBACK_SIGNAL,
        c.PIEZO_AMP_BNCMODE_LVOUT))

    def SetControlMode(self, channel=0, controlMode=c.PIEZO_OPEN_LOOP_MODE):
        """ When in closed-loop mode, position is maintained by a feedback signal from the piezo actuator.
        This is only possible when using actuators equipped with position sensing.
        This method sets the control loop status The Control Mode is specified in the Mode parameter as per the main documentation """
        channelID, destAddress = self.channelAddresses[channel]
        self.writeMessage(c.MGMSG_PZ_SET_POSCONTROLMODE, channelID, controlMode, destID=destAddress)

    def GetControlMode(self, channel=0):
        """ Get the control mode of the APT Piezo device"""
        response = self.query(c.MGMSG_PZ_REQ_POSCONTROLMODE, c.MGMSG_PZ_GET_POSCONTROLMODE, channelID,
                              destID=destAddress)
        assert response[1] == channelID, "inconsistent channel in response message from piezocontroller"
        return response[2]

    def SetVoltOutput(self, channel=0, voltOutput=0.0):
        """ Used to set the output voltage applied to the piezo actuator.
        This command is applicable only in Open Loop mode. If called when in Closed Loop mode it is ignored."""
        channelID, destAddress = self.channelAddresses[channel]
        voltParam = self._voltageAsFraction(voltOutput)
        self.writeMessage(c.MGMSG_PZ_SET_OUTPUTVOLTS, destID=destAddress, dataPacket=(channelID, voltParam))

    def GetVoltOutput(self, channel=0):
        """ Get the output voltage of the APT Piezo device. Only applicable when in open-loop mode """
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_PZ_REQ_OUTPUTVOLTS, c.MGMSG_PZ_GET_OUTPUTVOLTS, channelID, destID=destAddress)
        dataPacket = response[-1]
        assert dataPacket[0] == channelID, "inconsistent channel in response message from piezocontroller"
        return self._fractionAsVoltage(dataPacket[1])

    def SetPosOutput(self, channel=0, posOutput=10.0):
        """ Used to set the output position of piezo actuator. This command is applicable only in Closed Loop mode.
        If called when in Open Loop mode it is ignored.
        The position of the actuator is relative to the datum set for the arrangement using the ZeroPosition method."""
        channelID, destAddress = self.channelAddresses[channel]
        posParam = int(self._positionAsFraction(posOutput))
        #print('Requesting fraction {0} from pos {1}'.format(posParam, posOutput))
        self.writeMessage(c.MGMSG_PZ_SET_OUTPUTPOS, destID=destAddress, dataPacket=(channelID, posParam))

    def GetPosOutput(self, channel=0):
        """ Get the current position of the APT Piezo device. Only applicable when in closed-loop mode"""
        channelID, destAddress = self.channelAddresses[channel]
        #print('at get pos output channel {0}, destination {1}'.format(channelID, hex(destAddress)))
        #destAddress = 0x21
        response = self.query(c.MGMSG_PZ_REQ_OUTPUTPOS, c.MGMSG_PZ_GET_OUTPUTPOS, channelID, destID=destAddress)
        dataPacket = response[-1]
        #print('data packet {0}'.format(dataPacket))
        #print('channel id {0} and intended {1}'.format(dataPacket[0],channelID))
        assert dataPacket[0] == channelID, "inconsistent channel in response message from piezocontroller"
        #print('GetPosOutput says {0}'.format(dataPacket[1]))
        return self._fractionAsPosition(dataPacket[1])

    def ZeroPosition(self, channel=0):
        """ This function applies a voltage of zero volts to the actuator associated with the channel specified by the lChanID parameter, and then reads the position.
        This reading is then taken to be the zero reference for all subsequent position readings.
        This routine is typically called during the initialisation or re-initialisation of the piezo arrangement. """
        channelID, destAddress = self.channelAddresses[channel]
        self.writeMessage(c.MGMSG_PZ_SET_ZERO, channelID, destID=destAddress)

    def GetMaxTravel(self, channel=0):
        """ In the case of actuators with built in position sensing, the Piezoelectric Control Unit can detect the range of travel of the actuator
        since this information is programmed in the electronic circuit inside the actuator.
        This function retrieves the maximum travel for the piezo actuator associated with the channel specified by the Chan Ident parameter,
        and returns a value (in microns) in the Travel parameter."""
        channelID, destAddress = self.channelAddresses[channel]
        #print(hex(destAddress))
        destAddress = 0x21
        response = self.query(c.MGMSG_PZ_REQ_MAXTRAVEL, c.MGMSG_PZ_GET_MAXTRAVEL, channelID, destID=destAddress)
        dataPacket = response[-1]
        assert dataPacket[0] == channelID, "inconsistent channel in response message from piezocontroller"
        return dataPacket[1] * c.PIEZO_TRAVEL_STEP

    def GetMaxOPVoltage(self, channel=0):
        """ The piezo actuator connected to the unit has a specific maximum operating voltage range: 75, 100 or 150 V.
        This function gets the maximum voltage for the piezo actuator associated with the specified channel."""
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_PZ_REQ_OUTPUTMAXVOLTS, c.MGMSG_PZ_GET_OUTPUTMAXVOLTS, channelID,
                              destID=destAddress)
        dataPacket = response[-1]
        assert dataPacket[0] == channelID, "inconsistent channel in response message from piezocontroller"
        return dataPacket[1] * c.PIEZO_VOLTAGE_STEP

    def LLGetStatusBits(self, channel=0):
        """ Returns a number of status flags pertaining to the operation of the piezo controller channel specified in the Chan Ident parameter.
        These flags are returned in a single 32 bit integer parameter and can provide additional useful status information for client application development.
        The individual bits (flags) of the 32 bit integer value are described in the main documentaton."""
        channelID, destAddress = self.channelAddresses[channel]
        response = self.query(c.MGMSG_PZ_REQ_PZSTATUSBITS, c.MGMSG_PZ_GET_PZSTATUSBITS, channelID, destID=destAddress)
        dataPacket = response[-1]
        assert dataPacket[0] == channelID, "inconsistent channel in response message from piezocontroller"
        return dataPacket[1]

    # Helper methods for the above main methods. Change to mixed case since no need for compatibility with ActiveX control
    def _voltageAsFraction(self, voltage):
        """ specify voltage as short representing fraction of max voltage"""
        return round(c.PIEZO_MAX_VOLT_REPR * voltage / self.maxVoltage)

    def _fractionAsVoltage(self, voltFraction):
        """ convert voltage from short representing fraction of max voltage"""
        return voltFraction / c.PIEZO_MAX_VOLT_REPR * self.maxVoltage

    def _positionAsFraction(self, position):
        """ specify position as short representing fraction of max displacement. Apparently the max value depends on the unit though :( it might be 0xFFFF"""
        return round(c.PIEZO_MAX_POS_REPR * position / self.maxExtension)

    def _fractionAsPosition(self, positionFraction):
        """ convert position from short representing fraction of max displacement"""
        #print('With fraction {0}'.format(positionFraction))
        return positionFraction / c.PIEZO_MAX_POS_REPR * self.maxExtension

    def setLUTparam(self, mode, num_cycles, cycle_length, dwell_time, trigger_width, channel, start_dwell=100, end_dwell=100):
        '''
        Channel: 0/1/2
        Mode: fixed cycle, triggers output ever move, starts on trigger input
        continuousCycleLength: 40
        NumCycles: 20
        DelayTime: 10
        PreCycleRest: 0
        PostCycleRest: 0
        OPTrigStart: 0 # start output trigger at sample 0
        OPTrigWidth: 6 # let trigger last for 6 ms
        TrigRepeatCycle: 0 # every sample triggers
        '''
        #number_cycles = 2000 # how many cycles
        #cycle_length = 200 #how many samples will be output in each cycle of the waveform, less or equal to loaded
        #dwell_time = 800 # in units 1 ms
        #trigger_width = 1
        # construct packet tuple
        #print((channel, mode, cycle_length, int(number_cycles), dwell_time, 0, 0, 1, trigger_width, 0))

        channelID, destAddress = self.channelAddresses[channel]

        # number of cycles is a word i.e. 10 is 16
        #LSB = number_cycles % 256

        # 8 is 128
        # 6 is 96
        # 36 is 576
        # 80 is 1280
        # 40 is 640
        # 1 is 16
        # ----0x10 is 256
        # 10 is 160

        #print('cycle length was')
        #print(cycle_length)

        #cycle_length =  262 #132 #133 #133 # n + 2
        #num_cycles = 5

        #trigger_width = 10
        #dwell_time = 20
        #1 = 16 triggers

        packet = (channelID, mode, cycle_length, num_cycles, dwell_time,  start_dwell, end_dwell, 1, trigger_width, 1)

        self.writeMessage(c.MGMSG_PZ_SET_OUTPUTLUTPARAMS, dataPacket=packet,destID=destAddress)


    def setLUTmode(self, fixed=True, triggered_start=True, trigger_out=True):

        '''
        0x01 : Continuous
        0x02 : Fixed
        0x04 : Output triggering
        0x08 : Input triggering

        0x10 : Rising edge
        0x20 : Falling edge
        0x40 : Gated
        0x80 : Trigger repeated
        '''

        if fixed is True:
            char1 = 2
        else:
            char1 = 1  # continuous waveform output

        if triggered_start is True:
            char1 += 8 # ENnables input triggering

        if trigger_out is True:
            char1 += 4 # Generates a pulse on waveform output

        # TODO: 2nd char conditions not needed or included yet, see APT manual for implementation

        # Except it is!
        # 0x80 repeat triggering during single cycle
        # 0x10 output rising edge trigger
        word = char1

        word += 128

        word += 16

        return word

    def setLUTval(self, index, pos, channel=0):
        pos = 20+pos
        #print('LUT Setting position {0} at index {1} on channel {2} of max {3}'.format(pos,index,channel,self.maxExtension))
        posfrac = self._positionAsFraction(pos)
        channelID, destAddress = self.channelAddresses[channel]
        #print(self.channelAddresses)
        #print('index was {0}'.format(index))
        #print('pos frac was {0}'.format(posfrac))
        self.writeMessage(c.MGMSG_PZ_SET_OUTPUTLUT, dataPacket=(channelID, index, posfrac),destID=destAddress)


class AptMotor(_AptMotor):
    """ This class contains higher level methods not provided in the Thor Labs ActiveX control, but are very useful nonetheless """

    def deviceDescriptionStrings(self):
        # Mapping dictionary between class names and the description string given by the device
        return ['APT Stepper Motor Controller']

    def setPosition(self, channel=0, position=0):
        print('Moving channel {0} to {1}'.format(channel,position))
        self.MoveAbsoluteEnc(channel, position)

    def getPosition(self, channel=0):
        return self.GetPosition(channel)

    def getPositionRaw(self, channel=0):
        return self.GetPositionRaw(channel)

    def zero(self, channel=0):
        self.MoveHome(channel)

    def jogforward(self, channel=0):
        self.MoveJog(channel, direction=c.MOTOR_JOG_FORWARD)

    def jogback(self, channel =0):
        self.MoveJog(channel, direction=c.MOTOR_JOG_REVERSE)

    def jog_step(self, dist, channel =0):
        #convert to motor units (mm) here from SI
        self.SetJogStep(dist*1e3, channel =channel)







class AptPiezo(_AptPiezo):
    """ This class contains higher level methods not provided in the Thor Labs ActiveX control, but are very useful nonetheless """

    def deviceDescriptionStrings(self):
        """ Return a list of strings for which the device description is compatible with this class """
        return ["APT Piezo Controller"]

    def getEnableState(self, channel):
        response = self.query(c.MGMSG_MOD_REQ_CHANENABLESTATE, c.MGMSG_MOD_GET_CHANENABLESTATE, channel)
        assert response[1] == channel
        assert response[2] == c.CHAN_ENABLE_STATE_ENABLED or response[
                                                                 2] == c.CHAN_ENABLE_STATE_DISABLED, "Unrecognized enable state received"
        return response[2] == c.CHAN_ENABLE_STATE_ENABLED

    def isZeroing(self, channel):
        """ Check to see if the piezo controller is in the middle of zeroing (6th bit True)"""
        StatusBits = self.LLGetStatusBits(channel)
        return (StatusBits >> 5) & 1

    def setPosition(self, channel, position):
        """ Move to specified position if valid, and wait for the measured position to stabilize """

        print('channel {0} pos {1}'.format(channel,position))

        if position >= 0 and position <= self.maxExtension:
            self.SetPosOutput(channel, position)
            #print("setting {0}".format(position))
            t0 = time.time()

            while abs(position - self.GetPosOutput(channel)) > 1.01 * c.PIEZO_POSITION_ACCURACY:
                #print(position - self.GetPosOutput(channel))
                if (time.time() - t0) > c.PIEZO_MOVE_TIMEOUT:
                    print("Timeout error moving to " + str(position) + 'um on channel ' + str(channel) +", at "+str(self.GetPosOutput(channel)))
                    break
                else:
                    time.sleep(10e-3)

    def getPosition(self, channel):
        """ Get the position of the piezo. This is simply a wrapper for GetPosOutput using mixedCase """
        return self.GetPosOutput(channel)

    def zero(self, channel):
        """ Call the zero method and wait for it to finish """
        self.ZeroPosition(channel)
        t0 = time.time()
        while self.isZeroing(channel):
            if (time.time() - t0) > c.PIEZO_ZERO_TIMEOUT:
                print("Timeout error zeroing channel " + str(channel))
                break
            else:
                time.sleep(500e-3)

    def moveabsolute(self, posx, posy, posz):
        #pass

        self.setPosition(0,20+1e6*posx)
        self.setPosition(1,20+1e6*posy)
        self.setPosition(2,20+1e6*posz)

    def updateencoders(self, x, y, z):

        self.UpdateEncoder(0)
        self.UpdateEncoder(1)
        self.UpdateEncoder(2)

    def moveToCenter(self, channel):
        """ Moves the specified channel to half of its maximum extension"""
        self.setPosition(channel, self.maxExtension / 2)


    def set1Dscanrange(self, zmin, zmax, dwell, res = 130, channel = 0):
        mode_hex = self.setLUTmode(fixed=True, triggered_start=False, trigger_out=True)


        dwell = 10
        number_cycles = int(res/2)
        trigger_width = 6

        length_cycle = 4*res +2

        dwell= int(dwell)

        #zmin = 0.75*zmin*1e6 #in microns
        #zmax = -5 #0.75*zmax*1e6

        zmin =zmin * 1e6  # in microns
        zmax = zmax*1e6


        res = int(res)

        #trigger_width = 5  #  about right for 8 ms period, TODO: should be flexible

        #print(mode_hex, number_cycles, length_cycle, dwell, trigger_width, 0)
        self.setLUTparam(mode_hex, number_cycles, length_cycle, dwell, trigger_width, channel)

        # populate z matrix

        val = zmin
        delta = (zmax - zmin) / res
        #print('delta')
        #print(delta)
        #print(res)
        #print(res)


        #for i in range(0,7999):
        #    self.setLUTval(i, val, channel=channel)

        self.setLUTval(0, val, channel=channel)
        self.setLUTval(1, val, channel=channel)
        #self.setLUTval(2, val, channel=channel)

        #backscan = int(res/10)
        #
        # for i in range(2, res+2):
        #     val = val + delta
        #     #print(val)
        #     self.setLUTval(i, val, channel=channel)
        #
        # val = val + delta  # do last value twice
        #
        #
        # for i in range(res+2, 2*res+3):
        #     val = val - delta
        #     #print(val)
        #     self.setLUTval(i, val, channel=channel)

        for i in range(2, 2*res + 2,2):
            val = val + delta
            # print(val)
            self.setLUTval(i, val, channel=channel)
            self.setLUTval(i+1, val, channel=channel)

        val = val + delta  # do last value twice

        for i in range(2*res + 2, 4 * res + 3,2):
            val = val - delta
            # print(val)
            self.setLUTval(i, val, channel=channel)
            self.setLUTval(i+1, val, channel=channel)



    def setxyscanrange(self, xmin, xmax, ymin, ymax, dwell, res=160):

        # For x, channel 0
        # length of cycle is default the resolution  *2 (forward and back)
        # output triggering on each value
        # dwell is in units of BPC 1 ms

        dwell = 10
        trigger_width = 7 # approx 8 ms

        xmin = xmin*1e6
        xmax = xmax*1e6
        ymin = ymin *1e6
        ymax = ymax*1e6

        mode_hex = self.setLUTmode(fixed=True, triggered_start=False, trigger_out=True)
        number_cycles = int(res / 2)
        length_cycle = int(res * 4 + 2)
        dwell_bpc_x = int(dwell)

        res = int(res)


        print('in apt setxyscanrange res is ', res)


        self.setLUTparam(mode_hex, number_cycles, length_cycle, dwell_bpc_x, trigger_width, 0)

        # For y, channel 1
        # length of cycle is default the resolution
        # output triggering on each value
        # dwell is in units of BPC 1 ms * res, moves forward on each half cycle of x

        mode_hex = self.setLUTmode(fixed=True, triggered_start=False, trigger_out=False)
        number_cycles = 1
        length_cycle = res
        dwell_bpc_y = int(dwell_bpc_x * (2*res+1+100)*0.95)


        self.setLUTparam(mode_hex, number_cycles, length_cycle, dwell_bpc_y, trigger_width, 1, start_dwell=100)

        # populate x matrix

        delta = (xmax-xmin) / res
        val = xmin #+ delta/2

        self.setLUTval(0, val, channel=0)
        self.setLUTval(1, val, channel=0)

        for i in range(2, 2*res+2,2):
            val = val + delta
            self.setLUTval(i, val, channel=0)
            self.setLUTval(i+1, val, channel=0)

        val = val + delta  # do last value twice

        for i in range(2*res+2, 4*res+3,2):
            val = val - delta
            self.setLUTval( i, val,channel=0)
            self.setLUTval(i+1, val, channel=0)

        # populate y matrix


        delta = (ymax - ymin) / res
        val = ymin # + delta/2

        for i in range(0, res+1,1):
            val = val + delta
            self.setLUTval(i, val, channel=1)
            #self.setLUTval(i+1, val, channel=1)

        #self.setLUTval(res+1, (ymax - ymin)/2, channel=1)


    def startxyscan(self):

        #Arm channels and start scan
        for chan in range(0,2):
            channelID, destAddress = self.channelAddresses[chan]
            self.writeMessage(c.MGMSG_PZ_START_LUTOUTPUT, channelID, destID=destAddress)

        #TODO: tell arduino to pulse ??


    def startxscan(self):
        chan = 0
        channelID, destAddress = self.channelAddresses[chan]
        self.writeMessage(c.MGMSG_PZ_START_LUTOUTPUT, channelID, destID=destAddress)

    def startyscan(self):
        chan = 1
        channelID, destAddress = self.channelAddresses[chan]
        self.writeMessage(c.MGMSG_PZ_START_LUTOUTPUT, channelID, destID=destAddress)

    def startzscan(self):
        chan = 2
        channelID, destAddress = self.channelAddresses[chan]
        self.writeMessage(c.MGMSG_PZ_START_LUTOUTPUT, channelID, destID=destAddress)

    def changePI(self, P, I):
        P = int(P)
        I = int(I)
        for chan in range(0, 3):
            channelID, destAddress = self.channelAddresses[chan]
            self.writeMessage(c.MGMSG_PZ_SET_PICONSTS, channelID, destID=destAddress,
                              dataPacket=(channelID, P, I))



    def stopscan(self):

        for chan in range(0,3):
            channelID, destAddress = self.channelAddresses[chan]
            self.writeMessage(c.MGMSG_PZ_STOP_LUTOUTPUT, channelID, destID=destAddress)

    def abortscan(self):
        self.stopscan()



'''
Application note Joe Smith 07/01/19

Homing the nanomax stages with the kcube

This is successful, despite what Thorlabs say, but not trivial

Auto on the machine will reach one of the limits and complain by default.

getLimitsParams

Out[16]: (1061, None, None, 1, 80, (1, 2, 2, 1228800, 409600, 1))

getHomeparams

Out[21]: (1090, None, None, 1, 80, (1, 2, 1, 21987328, 204800))

SetLimitswitchParams(0)

setHomeParams(0)

'''

if __name__ == '__main__':
    p = AptPiezo()
    for ch in range(0,3):
        p.SetControlMode(ch, c.PIEZO_CLOSED_LOOP_MODE)
        p.zero(ch)
        #print('moving to centre')
        #p.moveToCenter(ch)
        #p.setPosition(ch,15)
        #p.stopscan()

        # dwell time seems to be in units 0.5 ms?

        #p.setxyscanrange(0,20,0,20,16,res=130)
        #p.startxyscan()
        p.stopscan()
    pass





