# -*- coding: utf-8 -*-

"""
This module contains the Qudi interface file for confocal scanner.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import numpy as np
import re
import time
import json
import serial
import socket
import logging
import os

from core.module import Base, ConfigOption
#from interface.slow_counter_interface import SlowCounterInterface
#from interface.slow_counter_interface import SlowCounterConstraints
#from interface.slow_counter_interface import CountingMode
#from interface.odmr_counter_interface import ODMRCounterInterface
from interface.confocal_scanner_interface import ConfocalScannerInterface

class FastScanner(Base, ConfocalScannerInterface):
    """ This is the Interface class to define the controls for the arduino scanner and FPGA hardware.
    """

    _modtype = 'FastScanner'
    _modclass = 'hardware'

    def on_activate(self):
        """ Start up TimeTagger interface
            This is where the channels and coincidences are decided
        """



        config = self.getConfiguration()

        if 'timetagger_channel_apd_0' in config.keys():
            self._channel_apd_0 = config['timetagger_channel_apd_0']
            self._chan_list = [config['timetagger_channel_apd_0']]
        else:
            self.log.error('No parameter "timetagger_channel_apd_0" configured.\n')


        if 'averager' in config.keys():
            self._chan_list.append(2)
            self._chan_list.append(3)
            self._chan_list.append(4)
            #self._chan_list.append(2)

        self._channel_apd_1 = None
        if 'timetagger_channel_apd_1' in config.keys():
            self._channel_apd_1 = config['timetagger_channel_apd_1']
            self._chan_list.append(config['timetagger_channel_apd_1'])

        if 'coincidence' in config.keys():
            self._coincidence = config['coincidence']
        else:
            self._coincidence = []

        self.TCP_IP = 'localhost'
        self.TCP_PORT = 5001
        window = 256
        #self.log.info('coincidence is {0}'.format(self._coincidence))
       # self.log.info('channel list is {0}'.format(self._chan_list))

        # should be -0.8 for picoharp pulsed laser sync

        # bias should be 1.1 V for IRNAS board to trigger

        self.ms = mysocket(sock=None, channels=self._chan_list, biases=[1.2,1.2,1.2,1.5,1.1,1.1,1.1,1.1], delays=[0,128,0,0,0], coincidences=self._coincidence,
                           window=window,
                           histogram_channels=[1], histogram_windows_ns=50)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused connection, killing Qudi')
            os._exit(-1)

        self.ms.update_timing_window(0)
        self._channel_apd = 1
        message = 'setup'
        self.ms.send(message)

        self.n_connects = 0

        self._count_frequency = 50  # Hz

        self.res = 130
        # scan resolution


        self.scanner_lock = False

        if 'timetagger_sum_channels' in config.keys():
            self._sum_channels = config['timetagger_sum_channels']
        else:
            #self.log.warning('No indication whether or not to sum apd channels for timetagger. Assuming false.')
            self._sum_channels = False
            self._channel_apd = 0

        if self._sum_channels and ('timetagger_channel_apd_1' in config.keys()):
            self.log.error('Cannot sum channels when only one apd channel given')

        if 'coincidence_window' in config.keys():
            self._coin_window = config[ 'coincidence_window']
        else:
            self._coin_window = 400e-9
            #self.log.info('No coincidence window set. Choosing 400 ns')

        ## self._mode can take 3 values:
        # 0: single channel, no summing
        # 1: single channel, summed over apd_0 and apd_1
        # 2: dual channel for apd_0 and apd_1
        if self._sum_channels:
            self._mode = 1
        elif self._channel_apd_1 is None:
            self._mode = 0
        else:
            self._mode = 2

        #self.ms.sock.shutdown(socket.SHUT_RDWR)


        # This is the handshake to be performed to prevent an error after requesting setup
        data = self.ms.sock.recv(2048)  # receive back again
        self.ms.sock.send(data)
        self.ms.sock.recv(2048)


        self.ms.close()

        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0

        try:
            self.arduino = FastController( "COM13", baudrate = 115200)
        except TypeError:
            self.log.error('Killing qudi')
            os._exit(-1)


    def on_deactivate(self):
        pass
    
    def reset_hardware(self):
        """ Resets the hardware, so the connection is lost and other programs
            can access it.

        @return int: error code (0:OK, -1:error)
        """
        return 0

    
    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [4][2]: array of 4 ranges with an array containing lower
                              and upper limit
        """
        return [[-2e-5, 0e-5], [-2e-5, 0e-5], [-2e-5, 0e-5], [-2e-5, 0e-5]]

    
    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.

        @param float [4][2] myrange: array of 4 ranges with an array containing
                                     lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        return 0

    
    def set_voltage_range(self, myrange=None):
        """ Sets the voltage range of the NI Card.

        @param float [2] myrange: array containing lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        return 0

    
    def get_scanner_axes(self):
        """ Find out how many axes the scanning device is using for confocal and their names.

        @return list(str): list of axis names

        Example:
          For 3D confocal microscopy in cartesian coordinates, ['x', 'y', 'z'] is a sensible value.
          For 2D, ['x', 'y'] would be typical.
          You could build a turntable microscope with ['r', 'phi', 'z'].
          Most callers of this function will only care about the number of axes, though.

          On error, return an empty list.
        """
        return ['x', 'y', 'z']

    
    def get_scanner_count_channels(self):
        """ Returns the list of channels that are recorded while scanning an image.

        @return list(str): channel names

        Most methods calling this might just care about the number of channels.
        """
        channels = ['Singles']

        return channels

    
    def set_up_scanner_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.

        @param float clock_frequency: if defined, this sets the frequency of the
                                      clock
        @param str clock_channel: if defined, this is the physical channel of
                                  the clock

        @return int: error code (0:OK, -1:error)
        """
        return 0

    
    def set_up_scanner(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None, res = 130, xrange = [0, 1e-5], yrange=[0, 1e-5]):
        """ Configures the actual scanner with a given clock.

        @param str counter_channels: if defined, these are the physical conting devices
        @param str sources: if defined, these are the physical channels where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)

        """

        self.counts_out = []
        self.counts_even = False

        self.flip = False

        self.ms.channels = counter_channels
        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)

        # Catch if Waterloobox not on to avoid hard crash
        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1



        # https://stackoverflow.com/questions/6439790/sending-a-reset-in-tcp-ip-socket-connection
        l_onoff = 1
        l_linger = 0
        import struct
        self.ms.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                     struct.pack('ii', l_onoff, l_linger))


        self.ms.sock.send(bytes('setup', 'ascii'))



        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('utf8').replace('\x00', ''))
        decrypted["poll_time"] = 0 #1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'scanner'
        decrypted["histogram_channels"] = res
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] =  [1.2,1.2,1.2,1.5,1.1,1.1,1.1,1.1]
        if self._coincidence:
            decrypted["coincidence_channels"] = self._coincidence
            decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)

        #then instruct arduino to scan

        self.res = res

        self.arduino.setscanrange(xrange[0],xrange[1],yrange[0],yrange[1],res)

        time.sleep(0.2)

        self.n = -1
        self.count_release = False

        value = self.arduino.startscan()

        if "S" in value:
            self.scanner_lock = True
            self.log.info('Scanner reported starting')
            return 0

        else:
            self.log.error('No word from scanner on start')
            return -1


    
    def scanner_set_position(self, x=None, y=None, z=None, a=None):
        """Move stage to x, y, z, a (where a is the fourth voltage channel).

        @param float x: postion in x-direction (volts)
        @param float y: postion in y-direction (volts)
        @param float z: postion in z-direction (volts)
        @param float a: postion in a-direction (volts)

        @return int: error code (0:OK, -1:error)
        """

        if self.scanner_lock is True:
            self.log.info('Cannot move whilst scanner scanning')
            return 0

        else:

            if x is not None:
                self.curr_x = x
            if y is not None:
                self.curr_y = y
            if z is not None:
                self.curr_z = z

            value = self.arduino.moveabsolute(self.curr_x, self.curr_y, self.curr_z)

            if 'M' in value:
                self.log.info('Scanner moved ({0:.2f}, {1:.2f}, {2:.2f}) um'.format(self.curr_x*1e6, self.curr_y*1e6, self.curr_z*1e6))
                return 0
            else:
                self.log.error('No movemement reported from scanner')
                return -1


    
    def get_scanner_position(self):
        """ Get the current position of the scanner hardware.

        @return float[n]: current position in (x, y, z, a).
        """
        #print('from scanner {0}, {1}, {2}'.format(self.curr_x,self.curr_y,self.curr_z))
        return [self.curr_x,self.curr_y,self.curr_z]

    def scan_image(self):
        found = 0
        samples = 1
        counts_out = np.empty(
            (65, 260),
            dtype=np.uint32)

        deltatime = 0
        counts0 = 0

        while found < samples:
            self.ms.sock.send(bytes('counts\x00', 'utf8'))
            raw_data = self.ms.sock.recv(262144)  # 2048
            if not raw_data:
                break

            start = bytes(raw_data).find(bytes('{'.encode('utf8')))
            end = bytes(raw_data).find(bytes('}'.encode('utf8')), start)
            json_str = raw_data[start:end + 1]
            try:
                decrypted = json.loads(json_str)
            except json.decoder.JSONDecodeError:
                break
            if 'counts' in decrypted['type']:
                counts_out = decrypted['histogram_counts']
                #print(counts_out)

                if not all(int(value) == 0 for value in counts_out[1]):
                    found = found + 1

        try:
            counts_out = np.array(counts_out, dtype=np.uint64)

        except ValueError:
            self.log.warning('Value error: {0}'.format(counts_out))
            counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32) * -1
        except OverflowError:
            self.log.warning('Overflow error: {0}'.format(counts_out))
            counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32) * -1

        if counts_out is None:
            counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32) * -1

        self.counts_out = counts_out.reshape((130, 130))
        self.counts_even = True
        self.count_release = True
    #print(self.counts_out[:, :])
        return self.counts_out[:, :].reshape(130,130, 1)

    def scan_line(self):
        found = 0
        samples = 1
        counts_out = np.empty(
            (self.res, 1),
            dtype=np.uint32)

        deltatime = 0
        counts0 = 0

        now =  time.time()
        while found < samples:
            self.ms.sock.send(bytes('counts\x00', 'ascii'))
            #self.ms.sock.send(bytes('counts\x00', 'utf8'))

            # block and wait for data
            #raw_data = self.ms.sock.recv(32768)  # 2048
            self.ms.sock.settimeout(2.0)
            try:
                raw_data = self.ms.sock.recv(2048)
            except socket.timeout:
                continue
            except BlockingIOError:
                continue
                #self.ms.sock.send(bytes('counts\x00', 'ascii'))
                #raw_data = self.ms.sock.recv(2048)

            self.ms.sock.settimeout(None)
            if not raw_data:
                continue

            start = bytes(raw_data).find(bytes('{'.encode('utf8')))
            end = bytes(raw_data).find(bytes('}'.encode('utf8')), start)
            json_str = raw_data[start:end + 1]
            try:
                decrypted = json.loads(json_str)
            except json.decoder.JSONDecodeError:
                break
            if 'counts' in decrypted['type']:
                counts_out = decrypted['histogram_counts']
                #print(counts_out)
                channel_c = decrypted['counts']
                if channel_c[-1] is not 0:
                    # look that sync channel running now, hence scan started
                    found = found + 1

            elif time.time() - now > 1:
                #only wait for 3 seconds
                found = found +1
                counts_out = np.zeros((self.res, samples), dtype=np.uint32)


        try:
            #print(counts_out)
            counts_out = np.array(counts_out, dtype=np.uint64)

        except ValueError:
            self.log.warning('Value error: {0}'.format(counts_out))
            counts_out = np.ones((self.res, samples), dtype=np.uint32) * -1
        except OverflowError:
            self.log.warning('Overflow error: {0}'.format(counts_out))
            counts_out = np.ones((self.res, samples), dtype=np.uint32) * -1

        if counts_out is None:
            counts_out = np.ones((self.res, samples), dtype=np.uint32) * -1

        self.counts_out = counts_out.reshape((self.res, 1))
        #self.counts_even = True
        #self.count_release = True

        # flip if odd
        if self.flip:
            self.counts_out = np.flip(self.counts_out, axis=0)

        self.flip = not self.flip

        return self.counts_out
    
    def scan_lineold(self, line_path=None, pixel_clock=False):
        """ Scans a line and returns the counts on that line.

        @param float[k][n] line_path: array k of n-part tuples defining the pixel positions
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[k][m]: the photon counts per second for k pixels with m channels
        """
        #get tag from waterloo box with poll time 0

        #print('scanning line {0}'.format(self.n))
        #self.n = self.n+1
        #print('{0},{1}'.format(self.counts_even, self.count_release))
        #print(self.counts_out)

        #there are four types of line: for1 back1 for2 back2 and we only do something for for1


        if self.counts_even is True:
            #This is the rule for return lines
            self.counts_even = False
            #print(self.counts_out[:,0,50].reshape(130,1))
            return self.counts_out[self.n,:].reshape(130,1)

        if self.count_release is True:
            self.counts_even = True

            #if self.n is 0 :
            #    self.counts_out = np.flip(self.counts_out,1)
            time.sleep(0.1)
            self.n = self.n + 1

            #print('{0}'.format(self.counts_out[:,self.n%2,int(self.n/2-1)].reshape(130,1)))
            #return self.counts_out[:, 0, 50]
            return self.counts_out[self.n,:].reshape(130,1)


        if self.counts_even is False and self.count_release is False:
            found = 0
            samples=1
            counts_out = np.empty(
                (65,260),
                dtype=np.uint32)

            deltatime = 0
            counts0 = 0

            while found < samples:
                self.ms.sock.send(bytes('counts\x00', 'utf8'))
                raw_data = self.ms.sock.recv(262144) #2048
                if not raw_data:
                    break

                start = bytes(raw_data).find(bytes('{'.encode('utf8')))
                end = bytes(raw_data).find(bytes('}'.encode('utf8')), start)
                json_str = raw_data[start:end + 1]
                try:
                    decrypted = json.loads(json_str)
                except json.decoder.JSONDecodeError:
                    break
                if 'counts' in decrypted['type']:
                    counts_out = decrypted['histogram_counts']

                    if not all(int(value)==0 for value in counts_out[1]):
                        found = found + 1

            try:
                counts_out = np.array(counts_out, dtype=np.uint64)

            except ValueError:
                self.log.warning('Value error: {0}'.format(counts_out))
                counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32)*-1
            except OverflowError:
                self.log.warning('Overflow error: {0}'.format(counts_out))
                counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32)*-1

            if counts_out is None:
                counts_out = np.ones((len(self.get_counter_channels()), samples), dtype=np.uint32) * -1

            self.counts_out = counts_out.reshape((130,130))
            self.counts_even = True
            self.count_release = True

            return self.counts_out[0,:].reshape(130,1)


    def close_scanner(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        self.ms.close()
        #
        # for i in range(0,5):
        #     try:
        #         self.ms.sock.send(bytes('disconnect\x00', 'utf8'))
        #         time.sleep(0.2)
        #     except OSError:
        #         break

        #time.sleep(1)
        #self.log.info('closing scanner')

        print('Asking arduino to abort')

        value = self.arduino.abortscan()
        print('Value')
        print(value)
        if 'X' in value:
            print('Scanner stopping in fast irnas')
            self.log.info('Scanner reported stopping')
            self.scanner_lock = False
            return 0
        else:
            self.log.error('Scanner did not report stopping')
            self.log.info('{0}'.format(value))
            return -1

    
    def close_scanner_clock(self, power=0):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        return 0


    def start_optimiser(self):

        #Send optimise command to arduino

        #Receive back the optimal value

        self.counts_out = []
        self.counts_even = False

        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        self.ms.sock.send(bytes('setup', 'utf8'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('utf8').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'stabilise'
        decrypted["histogram_channels"] = self.optim_res
        decrypted["edge_inversion_channels"] = 65536
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]
        if self._coincidence:
            decrypted["coincidence_channels"] = self._coincidence
            decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        #data = self.ms.sock.recv(2048)  # receive back again

        value = self.arduino.optimise(self.curr_x,self.curr_y,self.curr_z)
        if "O" in value:
            print('optimise step complete')
            #self.log.info("optimise successful")
        return 0

    def update_optimise_parameters(self, res = 5e-8, points = 7):

        self.optim_res = points

        #convert to voltage.
        resolution = res

        value = self.arduino.setoptimiserange(points, resolution)
        #print('Sent to arduino {0}, p {1}'.format(resolution, points))


        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        self.ms.sock.send(bytes('setup', 'utf8'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('utf8').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'stabilise'
        decrypted["histogram_channels"] = self.optim_res

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)


        #print('Sent to waterloo box p {0}'.format(points))

        if "P" in value:
            self.scanner_lock = True
            self.log.info('Optimise range changed {0} point, {1} res'.format(points, resolution))
            return 0

    def next_optimiser(self):
        found = 0
        samples = 1
        counts_out = np.empty(
            (self.optim_res, 1),
            dtype=np.uint32)

        deltatime = 0
        counts0 = 0

        now = time.time()
        while found < samples:
            self.ms.sock.send(bytes('counts\x00', 'ascii'))
            raw_data = self.ms.sock.recv(32768)  # 2048
            if not raw_data:
                break

            start = bytes(raw_data).find(bytes('{'.encode('utf8')))
            end = bytes(raw_data).find(bytes('}'.encode('utf8')), start)
            json_str = raw_data[start:end + 1]
            #print(json_str)
            try:
                decrypted = json.loads(json_str)

            except json.decoder.JSONDecodeError:
                break
            if 'counts' in decrypted['type']:
                counts_out = decrypted['histogram_counts']

                #print(type(counts_out))
                try:
                    if int(counts_out[0][0]) is not 0:
                        found = found + 1
                except IndexError:
                    pass

            elif time.time() - now > 3:
                # only wait for three seconds
                found = found + 1
                counts_out = np.zeros((self.optim_res, 3), dtype=np.uint32)

        try:
            counts_out = np.array(counts_out, dtype=np.uint64)

        except ValueError:
            self.log.warning('Value error: {0}'.format(counts_out))
            counts_out = np.ones((self.optim_res, 3), dtype=np.uint32) * -1
        except OverflowError:
            self.log.warning('Overflow error: {0}'.format(counts_out))
            counts_out = np.ones((self.optim_res, 3), dtype=np.uint32) * -1

        if counts_out is None:
            counts_out = np.ones((self.optim_res, 3), dtype=np.uint32) * -1

        self.counts_out = counts_out.reshape((3, self.optim_res))
        self.counts_out = np.transpose( self.counts_out)
        # self.counts_even = True
        # self.count_release = True

        print('asking to optimise at position {0}, {1}, {2}'.format(self.curr_x, self.curr_y, self.curr_z))

        value = self.arduino.optimise(self.curr_x, self.curr_y, self.curr_z)
        if "O" in value:
            self.log.info("optimise successful")
        return self.counts_out


    def set_up_zscan(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None, res=130, xrange=[-2e-5, 0], yrange=[-2e-5, 0],inttime=8):
        """ Configures the actual scanner with a given clock.

        @param str counter_channels: if defined, these are the physical conting devices
        @param str sources: if defined, these are the physical channels where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)

        """

        self.counts_out = []
        self.counts_even = False

        self.flip = False

        self.ms.channels = counter_channels
        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        # https://stackoverflow.com/questions/6439790/sending-a-reset-in-tcp-ip-socket-connection
        l_onoff = 1
        l_linger = 0
        import struct
        self.ms.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                struct.pack('ii', l_onoff, l_linger))

        self.ms.sock.send(bytes('setup', 'ascii'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('ascii').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'scanner'
        decrypted["histogram_channels"] = res
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]
        if self._coincidence:
            decrypted["coincidence_channels"] = self._coincidence
            decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)

        # then instruct arduino to scan

        self.res = res

        self.arduino.setscanrangez(xrange[0], xrange[1], yrange[0], yrange[1], res, inttime=inttime)

        time.sleep(0.2)

        self.n = -1
        self.count_release = False

        value = self.arduino.startzscan()

        if "Z" in value:
            self.scanner_lock = True
            self.log.info('Z scanner reported starting')
            return 0

        else:
            self.log.error('No word from z scanner on start')
            return -1




    def set_up_xscan(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None, res=130, xrange=[-2e-5, 0], yrange=[-2e-5, 0],inttime=8):
        """ Configures the actual scanner with a given clock.

        @param str counter_channels: if defined, these are the physical conting devices
        @param str sources: if defined, these are the physical channels where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)

        """

        self.counts_out = []
        self.counts_even = False

        self.flip = False

        self.ms.channels = counter_channels
        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        # https://stackoverflow.com/questions/6439790/sending-a-reset-in-tcp-ip-socket-connection
        l_onoff = 1
        l_linger = 0
        import struct
        self.ms.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                struct.pack('ii', l_onoff, l_linger))

        self.ms.sock.send(bytes('setup', 'ascii'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('ascii').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'scanner'
        decrypted["histogram_channels"] = res
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]
        if self._coincidence:
            decrypted["coincidence_channels"] = self._coincidence
            decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)


        # can throw OSError, in which case open socket again

        # then instruct arduino to scan

        self.res = res

        self.arduino.setscanrange(xrange[0], xrange[1], yrange[0], yrange[1], res, inttime=inttime)

        time.sleep(0.2)

        self.n = -1
        self.count_release = False

        value = self.arduino.startxscan()

        if "X" in value:
            self.scanner_lock = True
            self.log.info('X scanner reported starting')
            return 0

        else:
            self.log.error('No word from x scanner on start')
            return -1

    def set_up_yscan(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None, res=130, xrange=[-2e-5, 0], yrange=[-2e-5, 0],inttime=8):
        """ Configures the actual scanner with a given clock.

        @param str counter_channels: if defined, these are the physical conting devices
        @param str sources: if defined, these are the physical channels where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)

        """

        self.counts_out = []
        self.counts_even = False

        self.flip = False

        self.ms.channels = counter_channels
        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        # https://stackoverflow.com/questions/6439790/sending-a-reset-in-tcp-ip-socket-connection
        l_onoff = 1
        l_linger = 0
        import struct
        self.ms.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                struct.pack('ii', l_onoff, l_linger))

        self.ms.sock.send(bytes('setup', 'ascii'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('ascii').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'scanner'
        decrypted["histogram_channels"] = res
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]
        if self._coincidence:
            decrypted["coincidence_channels"] = self._coincidence
            decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)

        # then instruct arduino to scan

        self.res = res

        self.arduino.setscanrange(xrange[0], xrange[1], yrange[0], yrange[1], res, inttime=inttime)

        time.sleep(0.2)

        self.n = -1
        self.count_release = False

        value = self.arduino.startyscan()

        if "Y" in value:
            self.scanner_lock = True
            self.log.info('Y scanner reported starting')
            return 0

        else:
            self.log.error('No word from y scanner on start')
            return -1



    def set_up_3dscan(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None, delta_res = 150e-9, xydelta=3e-6, xyres = 30, zres = 10, zdelta=1e-6,inttime=8):
        """ Configures the actual scanner with a given clock.

        @param str counter_channels: if defined, these are the physical conting devices
        @param str sources: if defined, these are the physical channels where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)

        """

        # xy range in square scan, around current location

        #print('3D scanner has current location ')
        xrange = [self.curr_x - xydelta / 2, self.curr_x + xydelta / 2]

        # yrange in square scan, around current location
        yrange = [self.curr_y - xydelta / 2, self.curr_y + xydelta / 2]

        # self.res = res

        zrange = [self.curr_z - zdelta / 2, self.curr_z + zdelta / 2]

        if (xrange[0] < -20e-6):
            # diff = xrange[0] - yrange[0]
            self.log.error('Proposed x-min {0} out of range'.format(xrange[0]))
            return -1
            xrange[0] = -20e-6

        if (yrange[0] < -20e-6):
            self.log.error('Proposed y-min {0} out of range'.format(yrange[0]))
            return -1
            yrange[0] = -20e-6

        if (zrange[0] < -20e-6):
            self.log.error('Proposed z-min {0} out of range'.format(zrange[0]))
            return -1
            zrange[0] = -20e-6

        if (xrange[1] > 0e-6):
            self.log.error('Proposed x-max {0} out of range'.format(xrange[1]))
            return -1
            xrange[1] = 0e-6

        if (yrange[1] > 0e-6):
            self.log.error('Proposed y-max {0} out of range'.format(yrange[1]))
            return -1
            yrange[1] = 0e-6

        if (zrange[1] > 0e-6):
            self.log.error('Proposed z-max {0} out of range'.format(zrange[1]))
            return -1
            zrange[1] = 0e-6




        self.counts_out = []
        self.counts_even = False

        self.ms.channels = counter_channels
        # tick = time.perf_counter()
        window = 0
        # (1 / self._count_frequency)
        self.ms.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused the connection. Is the CQP server turned on?')
            return -1
        # https://stackoverflow.com/questions/6439790/sending-a-reset-in-tcp-ip-socket-connection
        l_onoff = 1
        l_linger = 0
        import struct
        self.ms.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                struct.pack('ii', l_onoff, l_linger))

        self.ms.sock.send(bytes('setup', 'ascii'))
        data = self.ms.sock.recv(2048)
        # print(data)
        decrypted = json.loads(data.decode('ascii').replace('\x00', ''))
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = '3dscanner'
        decrypted["histogram_channels"] = int(xydelta/delta_res) # slightly dodgy calculating array size on hardware too
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]

        decrypted["slide_co_width"] = int(zdelta/delta_res) # proxy for z res

        # if self._coincidence:
        #     decrypted["coincidence_channels"] = self._coincidence
        #     decrypted["coincidence_windows_ns"] = [self._coin_window * 1e9]

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)

        # then instruct arduino to scan

        self.log.info('Setting up 3D scan with ({0:.2f}, {1:.2f}) ({2:.2f}, {3:.2f}),({4:.2f}, {5:.2f}) um '.format(1e6*xrange[0], 1e6*xrange[1], 1e6*yrange[0],
                                                                                                                    1e6*yrange[1], 1e6*zrange[0], 1e6*zrange[1]))
        self.arduino.set3dscanrange(xrange[0], xrange[1], yrange[0], yrange[1], zrange[0], zrange[1], xyres, zres, delta_res, inttime=inttime)

        time.sleep(0.2)

        self.n = -1
        self.count_release = False

        self.res = int(xydelta/delta_res)

        value = self.arduino.start3dscan()

        if "D" in value:
            self.scanner_lock = True
            self.log.info('3D scanner reported starting')
            return 0

        else:
            self.log.error('No word from 3D scanner on start')
            return -1



import logging

class FastController():
    '''
    Python class for controlling Arduino
    '''

    def __init__(self, port, baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate=115200, timeout = 0.5)
            self.serial.write(b'99')
        except serial.serialutil.SerialException:
            logging.error('Cannot connect to Arduino, is the port in use?')
            return -1

    def __str__(self):
        return "Arduino is on port %s at %d baudrate" % (self.serial.port, self.serial.baudrate)

    def setoptimiserange(self, points, increment):
        self.__sendData('1')
        self.__sendData(self.d2vint(increment))
        self.__sendData(int(points))

        print('optim range ',increment, self.d2vint(increment), points)
        return self.__getData()

    def setscanrange(self, xmin, xmax, ymin, ymax, resolution, inttime = 10):
        self.__sendData('2')

        # Important note, send parameters back to front
        print('setting range ', xmin, xmax, ymin, ymax, resolution, inttime)
        print('setting volt ', self.d2vint(xmax), self.d2vint(xmin), self.d2vint(ymax), self.d2vint(ymin))
        self.__sendData(int(inttime))
        self.__sendData(int(resolution))
        self.__sendData(self.d2vint(ymax))
        self.__sendData(self.d2vint(ymin))
        self.__sendData(self.d2vint(xmax))
        self.__sendData(self.d2vint(xmin))

        return self.__getData()

    def setscanrangez(self, xmin, xmax, ymin, ymax, resolution, inttime = 10):
        self.__sendData('2')

        # Important note, send parameters back to front
        print('setting range ', xmin, xmax, ymin, ymax, resolution, inttime)
        print('setting volt ', self.d2vintz(xmax), self.d2vintz(xmin), self.d2vintz(ymax), self.d2vintz(ymin))
        self.__sendData(int(inttime))
        self.__sendData(int(resolution))
        self.__sendData(self.d2vintz(ymax))
        self.__sendData(self.d2vintz(ymin))
        self.__sendData(self.d2vintz(xmax))
        self.__sendData(self.d2vintz(xmin))

        return self.__getData()

    def set3dscanrange(self, xmin, xmax, ymin, ymax,  zmin, zmax, xres, zres, delta, inttime = 10):
        self.__sendData('11')

        # Important note, send parameters back to front
        #print('setting range ', xmin, xmax, ymin, ymax, resolution, inttime)
        print('setting volt ', self.d2vint(xmin), self.d2vint(xmax), self.d2vint(ymin), self.d2vint(ymax),self.d2vintz(zmin),
              self.d2vintz(zmax),int(xres),int(zres),self.d2vint(delta),int(inttime))
        self.__sendData(int(inttime))
        self.__sendData(self.d2vint(delta))
        self.__sendData(int(zres))
        self.__sendData(int(xres))
        self.__sendData(self.d2vintz(zmax))
        self.__sendData(self.d2vintz(zmin))
        self.__sendData(self.d2vint(ymax))
        self.__sendData(self.d2vint(ymin))
        self.__sendData(self.d2vint(xmax))
        self.__sendData(self.d2vint(xmin))

    def startscan(self):
        self.__sendData('3')
        return self.__getData()

    def startzscan(self):
        self.__sendData('7')
        return self.__getData()

    def startxscan(self):
        self.__sendData('8')
        return self.__getData()

    def startyscan(self):
        self.__sendData('9')
        return self.__getData()

    def start3dscan(self):
        self.__sendData('10')
        print('10')
        return self.__getData()

    def moveabsolute(self, xvalue,yvalue,zvalue):
        #print('moveabsolute {0}, {1}, {2} '.format(xvalue,yvalue,zvalue))
        #print('{0}, {1}, {2} V'.format(self.d2vint(xvalue), self.d2vint(yvalue), self.d2vint(zvalue)))
        self.__sendData('4')
        self.__sendData(self.d2vintz(zvalue))
        self.__sendData(self.d2vint(yvalue))
        self.__sendData(self.d2vint(xvalue))
        return self.__getData()

    def optimise(self, startx, starty, startz):
        self.__sendData('5')

        #self.__sendData(self.d2vint(startz))
        #self.__sendData(self.d2vint(starty))
        #self.__sendData(self.d2vint(startx))

        self.__sendData('{0},{1},{2}'.format(self.d2vint(startx),self.d2vint(starty),self.d2vint(startz)))
        print('optimise {0}, {1}, {2}'.format(self.d2vint(startx),self.d2vint(starty),self.d2vint(startz)))
        return self.__getData()

    def abortscan(self):
        #self.__sendData('6')

        serial_data = str('6').encode('utf-8')
        self.serial.write(serial_data)
        value = self.__getData()
        while 'X' not in value:
            print('value')
            print(value)
            value = self.__getData()
        return value

    def __sendData(self, serial_data):
        while ( "w" not in self.__getData() ):
            pass
        serial_data = str(serial_data).encode()
        self.serial.write(serial_data)

    def __getData(self):
        input_string = self.serial.readline()
        input_string = input_string.decode('utf-8')
        return input_string.rstrip('\r\n')

    def close(self):
        self.serial.close()
        return True

    def d2vint(self, dist):
        value = int(self.dist2volt(dist))

        if value < 0:
            value = 0
            #logging.warning('voltage out of range {0} from dist {1}'.format(value, dist))
        if value > 65535:
            #logging.warning('voltage out of range {0} from dist {1}'.format(value,dist))
            value = 65535
        return value

    def d2vintz(self, dist):
        value = int(abs(dist ) / ((20 / 45219) * 1e-6))

        if value < 0:
            value = 0
            #logging.warning('voltage out of range {0} from dist {1}'.format(value, dist))
        if value > 65535:
            #logging.warning('voltage out of range {0} from dist {1}'.format(value,dist))
            value = 65535
        return value


    def vol2dist(self, voltage):

        self.dist = -1*(20 / 65535) * 1e-6 * voltage

        # - 13.8 u is the actual -20 u in the old version
        # we want the range to be less

        return self.dist

    def dist2volt(self, dist):

        # no longer volt but 0 to 65535 on DAC

        self.volt = abs(dist ) / ((20 / 65535) * 1e-6)

        # resolution is 3 mV
        #self.volt = round(self.volt, 5)

        return self.volt




class mysocket:
    '''demonstration class only
      - coded for clarity, not efficiency
    '''

    def __init__(self, sock=None, channels=[1], biases=[], delays=[], coincidences=[], window=0, histogram_channels=[],
                 histogram_windows_ns=[], n = 0):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.need_setup = 1
        self.channels = channels
        self.biases = biases
        self.delays = delays
        self.coincidences = coincidences
        self.window = 0
        self.histogram_channels = histogram_channels
        self.histogram_windows_ns = histogram_windows_ns
        self.name = "imaging_pc" +str(n)

    def send_setup(self):
        #logging.info("------------------- sending setup ----------------------")
        #        print self.current_setup
        message = json.dumps(self.current_setup)
        self.send(message)
        #logging.info(message)
        #logging.info("--------------------------------------------------------")

    def connect(self, host, port):
        #print('connecting')
        self.sock.connect((host, port))

    def send(self, msg):
        #msg should be bytes
        totalsent = 0
        while totalsent < len(msg):
            #logging.info(type(msg))
            #logging.info(msg.type)
            #logging.info(msg[totalsent:])

            sent = self.sock.send(bytes(msg[totalsent:].encode("ascii")))
            if sent == 0:
                logging.error("Socket connection to Waterloo broken")
            totalsent = totalsent + sent

    def recv(self, msglen):
        chunks = []
        bytes_recd = 0

        while bytes_recd < msglen:
            chunk = self.sock.recv(min(msglen - bytes_recd, 2048))
            #            print '\'' + chunk + '\''
            if chunk == '':
                logging.error("socket connection broken")
            done = 0
            end = 0
            items_found = 0
            # Look for { ... } pairs, and consider them to be complete messages
            while not done:
                start = bytes(chunk).find(bytes('{'.encode('utf8')), end);
                if (start >= 0):
                    end = bytes(chunk).find(bytes('}'.encode('utf8')), start)
                    if (end >= 0):
                        json_str = chunk[start:end + 1]
                        json_data = json.loads(json_str)
                        #print(json_data)
                        if json_data["type"] == "setup":
                            self.handle_setup(json_data)
                        items_found += 1
                        chunks.append(json_data)
                    else:
                        done = 1
                else:
                    done = 1
            bytes_recd = bytes_recd + len(chunk)
        # print items_found
        return chunks

    def close(self):
        #print('connecting')
        self.send('disconnect')
        #self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()

    def set_channels(self, channels, input_threshold_volts, delay_ns):
        active_channel_bits = sum(1 << (chan - 1) for (chan) in channels)

        #        active_channel_bits = 0xffff
        #        input_threshold_volts = -2.0

        self.current_setup["active_channels"] = active_channel_bits
        index = 0
        #logging.info(channels)
        for chan in channels:
            self.current_setup["input_threshold_volts"][chan - 1] = input_threshold_volts[index]
            self.current_setup["channel_delay_ns"][chan - 1] = delay_ns[index]
            #            self.current_setup["input_threshold_volts"][chan - 1] = input_threshold_volts[index]
            index += 1

    def add_coincidence(self, channels, coincidence_window_ns):
        co_channel_mask = sum(1 << (chan - 1) for (chan) in channels)
        #logging.info('co_channel_mask = {}'.format(co_channel_mask))
        self.current_setup["coincidence_channels"].append(co_channel_mask)
        self.current_setup["coincidence_windows_ns"].append(coincidence_window_ns)

    def update_timing_window(self, integration_time):
        if self.need_setup:
            self.recv(1024)
        else:
            self.current_setup['poll_time'] = integration_time
            self.send_setup()

    ###### Here's where you se the number of channels, etc.
    def handle_setup(self, setup_object):
        self.current_setup = setup_object

        if (self.need_setup):
            self.current_setup["user_name"] = self.name
            self.current_setup["user_platform"] = "python"
            self.current_setup["poll_time"] = 0
            self.need_setup = 0

            # "I want data from channels 7, 15 and 16, with voltages -0.1 and delay 0ns"
            self.set_channels(self.channels, self.biases, self.delays)
            self.current_setup["coincidence_channels"] = []
            # "I want coincidences on channel 7 and 15, with 4000ns window"
            self.add_coincidence(self.channels, 4000)

            # "I ALSO want coincidences on channel 7 and 15, with 4000ns window"
            # self.add_coincidence([15, 16], 4000)
            self.send_setup()

    def singles(self, channels, inttime=1.0, msgbytes=2048):
        '''
        Integrate the singles counts per second for an arbitrary number of
        channels
        --------
        :channels:
                list or tuple of ints, or an int specifying the channel(s) to
                integrate singles for
        :inttime:
                positive float - total time to integrate for
        :msgbytes:
                positive int - size of message in bytes to recieve from websocket
        '''
        # Check for correct datatypes
        #logging.info(channels)
        if isinstance(channels, (list, tuple)):
            num_channels = len(channels)
            singles = [0] * num_channels
        elif isinstance(channels, int):
            num_channels = 1
            singles = [0]
            channels = channels
        else:
            raise TypeError('channels arg must be a list, tuple or int')

        if not isinstance(msgbytes, int):
            try:
                msgbytes = int(msgbytes)
            except:
                raise TypeError('msgbytes arg must be an int')

        reltime = 0.
        attempts = 0
        MAX_ATTEMPTS = 10

        while (reltime < inttime):

            # Attempt to handle errors from the socket not responding
            try:
                # get our data from the socket
                data = self.recv(msgbytes)
                #logging.info(data)
                if not data:
                    continue
                attempts = 0
            except:
                attempts += 1
                if attempts < MAX_ATTEMPTS:
                    print('An error occurred... trying again (attempt {}/{})'.format(attempts, MAX_ATTEMPTS))
                else:
                    print('Maximum number of tries reached... quitting')
                    self.__del__()
                    raise
                continue

            # loop through the data acquired
            for i in range(1, len(data) - 1):

                # check the datatype
                if data[i]['type'] == 'counts':
                    #logging.info(data[i]['counts'])
                    # now loop through every channel and add the number of singles
                    for j in range(num_channels):
                        channel = 0 #TODO : fix
                        singles[j] += data[i]['counts'][channel]
                        #logging.info(singles[j])
                        #logging.info(channel)

                    # add to our total time integrated for
                    reltime += data[i]['span_time']
                    #logging.info(reltime)
                    #logging.info(inttime)
                    if (reltime >= inttime):
                        break

                        #               print ('singles:',singles, 'integration time:', reltime, 'counts/s',[single/reltime for single in singles])
        #logging.info(singles)
        return [single / reltime for single in singles]

    def integrate(ms, msgbytes, channels, inttime):
        singles = [0, 0]
        coincidence = 0.
        reltime = 0.00000001

        #       message = 'setup'
        #       ms.send(message)

        # print('Recieving data')
        # data = ms.recv(int(msgbytes/4))
        # print (len(data))
        #       print('reltime\t\tsingles[0]\tsingles[1]\tcoincidence')

        while (reltime < inttime):
            data = ms.recv(msgbytes)
            datlen = len(data)
            #               print (len(data))
            # while len(data) < 8: #ensure that we actually got something
            # print('Recieving data')
            # data = ms.recv(msgbytes)
            # print (len(data))

            for i in range(1, datlen - 1):
                if data[i]['type'] == 'counts':
                    if len(data[i]['coincidence']) == 1:
                        singles[0] += data[i]['counts'][channels[0] - 1]
                        singles[1] += data[i]['counts'][channels[1] - 1]
                        coincidence += data[i]['coincidence'][0]
                        reltime += data[i]['span_time']

                        #               print('{0: 6.2f}\t\t{1: 6.0f}\t\t{2: 6.0f}\t\t{3: 6.2f}' .format(reltime, singles[0]/reltime,  singles[1]/reltime,  coincidence/reltime))

        return [reltime, singles[0] / reltime, singles[1] / reltime, coincidence / reltime]

    def __del__(self):
        try:
            self.close()
        except:
            pass