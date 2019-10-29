# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware dummy for fast counting devices.

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

import time
import numpy as np
import re
import json
import socket
import logging
import os

from core.module import Base, ConfigOption
from core.util.modules import get_main_dir
from interface.fast_counter_interface import FastCounterInterface


class FastCounterWaterloo(Base, FastCounterInterface):
    """This is the Interface class to define the controls for the simple
    microwave hardware.
    """
    _modclass = 'fastcounterinterface'
    _modtype = 'hardware'

    # config option
    _gated = ConfigOption('gated', False, missing='warn')

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        self.log.debug('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.info('{0}: {1}'.format(key,config[key]))



    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self.statusvar = 0
        self._binwidth = 1
        self._gate_length_bins = 8192

        self.TCP_IP = 'localhost'
        self.TCP_PORT = 5001

        self._chan_list = [1]

        self.ms = mysocket(sock=None, channels=self._chan_list, biases=[1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1],
                           delays=[0, 0, 0, 0, 0],histogram_channels=[1], histogram_windows_ns=50)

        try:
            self.ms.connect(self.TCP_IP, self.TCP_PORT)
        except ConnectionRefusedError:
            self.log.error('Waterloo box refused connection, killing Qudi')
            os._exit(-1)

        self.ms.update_timing_window(0)
        self._channel_apd = 1
        message = 'setup'
        self.ms.send(message)

        self._count_data = np.zeros((self._gate_length_bins,),dtype=np.uint32)


        return

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self.statusvar = -1
        return

    def get_constraints(self):
        """ Retrieve the hardware constrains from the Fast counting device.

        @return dict: dict with keys being the constraint names as string and
                      items are the definition for the constaints.

         The keys of the returned dictionary are the str name for the constraints
        (which are set in this method).

                    NO OTHER KEYS SHOULD BE INVENTED!

        If you are not sure about the meaning, look in other hardware files to
        get an impression. If still additional constraints are needed, then they
        have to be added to all files containing this interface.

        The items of the keys are again dictionaries which have the generic
        dictionary form:
            {'min': <value>,
             'max': <value>,
             'step': <value>,
             'unit': '<value>'}

        Only the key 'hardware_binwidth_list' differs, since they
        contain the list of possible binwidths.

        If the constraints cannot be set in the fast counting hardware then
        write just zero to each key of the generic dicts.
        Note that there is a difference between float input (0.0) and
        integer input (0), because some logic modules might rely on that
        distinction.

        ALL THE PRESENT KEYS OF THE CONSTRAINTS DICT MUST BE ASSIGNED!
        """

        constraints = dict()

        # the unit of those entries are seconds per bin. In order to get the
        # current binwidth in seonds use the get_binwidth method.
        constraints['hardware_binwidth_list'] = [10e-9] #, 2/950e6, 4/950e6, 8/950e6]

        return constraints

    def configure(self, bin_width_s, record_length_s, number_of_gates = 0):
        """ Configuration of the fast counter.

        @param float bin_width_s: Length of a single time bin in the time trace
                                  histogram in seconds.
        @param float record_length_s: Total length of the timetrace/each single
                                      gate in seconds.
        @param int number_of_gates: optional, number of gates in the pulse
                                    sequence. Ignore for not gated counter.

        @return tuple(binwidth_s, gate_length_s, number_of_gates):
                    binwidth_s: float the actual set binwidth in seconds
                    gate_length_s: the actual set gate length in seconds
                    number_of_gates: the number of gated, which are accepted
        """
        self._binwidth = int(np.rint(bin_width_s * 1e9 ))
        self._gate_length_bins = int(np.rint(record_length_s / bin_width_s))
        actual_binwidth = self._binwidth / 1e9
        actual_length = self._gate_length_bins * actual_binwidth
        self.statusvar = 1
        return actual_binwidth, actual_length, number_of_gates


    def get_status(self):
        """ Receives the current status of the Fast Counter and outputs it as
            return value.

        0 = unconfigured
        1 = idle
        2 = running
        3 = paused
        -1 = error state
        """
        return self.statusvar

    def start_measure(self):

        if self.statusvar is 2:
            self.log.error('Another fast counter is already running, close this one '
                           'first.')
            return -1


        if self.statusvar is 1:
            self._count_data = np.zeros((self._gate_length_bins,), dtype=np.uint32)

        self.statusvar = 2


        self.ms.channels = 0

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
        decrypted["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3
        decrypted["user_name"] = 'imaging_pc0'
        decrypted["user_platform"] = 'fast'
        decrypted["histogram_channels"] = self._gate_length_bins
        decrypted["edge_inversion_channels"] = 32768
        decrypted["input_threshold_volts"] = [1.2, 1.2, 1.2, 1.5, 1.1, 1.1, 1.1, 1.1]

        decrypted["coincidence_windows_ns"] = [int(self.get_binwidth()*1e9)]

        self.current_setup = decrypted

        send_dat = json.dumps(decrypted) + '\x00'
        self.ms.sock.send(bytes(send_dat, 'utf8'))
        time.sleep(0.2)

        # self.ms.sock.send(bytes('counts', 'utf8'))

        data = self.ms.sock.recv(2048)  # receive back again
        time.sleep(0.2)

        self.n = -1
        self.count_release = False


        return 0

    def pause_measure(self):
        """ Pauses the current measurement.

        Fast counter must be initially in the run state to make it pause.
        """
        #time.sleep(1)
        self.statusvar = 3

        self.ms.close()

        return 0

    def stop_measure(self):
        """ Stop the fast counter. """

        #time.sleep(1)
        if self.statusvar is not 3:
            self.ms.close()

        self.statusvar = 1

        return 0

    def continue_measure(self):
        """ Continues the current measurement.

        If fast counter is in pause state, then fast counter will be continued.
        """

        if self.statusvar is 3:
            self.start_measure()

        self.statusvar = 2
        return 0

    def is_gated(self):
        """ Check the gated counting possibility.

        @return bool: Boolean value indicates if the fast counter is a gated
                      counter (TRUE) or not (FALSE).
        """

        return self._gated

    def get_binwidth(self):
        """ Returns the width of a single timebin in the timetrace in seconds.

        @return float: current length of a single bin in seconds (seconds/bin)
        """
        width_in_seconds = self._binwidth * 1/1e9
        return width_in_seconds

    def get_data_trace(self):
        """ Polls the current timetrace data from the fast counter.

        Return value is a numpy array (dtype = int64).
        The binning, specified by calling configure() in forehand, must be
        taken care of in this hardware class. A possible overflow of the
        histogram bins must be caught here and taken care of.
        If the counter is NOT GATED it will return a 1D-numpy-array with
            returnarray[timebin_index]
        If the counter is GATED it will return a 2D-numpy-array with
            returnarray[gate_index, timebin_index]
        """

        if self.statusvar is not 2:
            self.log.error('Count not running')
            return -1

        try:
            self.ms.sock.send(bytes('counts\x00', 'ascii'))
        except ConnectionResetError:
            self.start_measure()
            self.ms.sock.send(bytes('counts\x00', 'ascii'))
            #return self._count_data


        found = 0
        counts_out = np.zeros(
            (self._gate_length_bins,),
            dtype=np.uint32)

        flag = 0

        counter = None # Keeps track of how many sweeps have been peformed

        while found < 1:
            #try:
            self.ms.sock.send(bytes('counts\x00', 'ascii'))
            #except ConnectionResetError:
             #   return self._count_data

            self.ms.sock.settimeout(60.0)
            try:
                raw_data = self.ms.sock.recv(65536)  # 2048
            except socket.timeout:

                if flag is 1:
                    print('Rescanning line')
                    # self.bpc.startxscan()
                    flag = 0
                else:

                    print('Socket timed out, attempting to pull what we have')

                    self.current_setup["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3

                    send_dat = json.dumps(self.current_setup) + '\x00'
                    self.ms.sock.send(bytes(send_dat, 'utf8'))
                    self.ms.sock.recv(2048)  # receive setup back again
                    raw_data = self.ms.sock.recv(2048)
                    self.current_setup["poll_time"] = 0  # 1 / self._count_frequency  # 20e-3

                    send_dat = json.dumps(self.current_setup) + '\x00'
                    self.ms.sock.send(bytes(send_dat, 'utf8'))

                    self.ms.sock.recv(2048)  # receive setup back again

                    flag = 1

            except BlockingIOError:
                print('Socket blocking error')
                continue

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

            if 'counts' in decrypted['type']:  # and 'scanner' in decrypted['user_platform']:
                counts_out = decrypted['histogram_counts']

                counter = decrypted['counter']

                #if counts_out[0] >= 0:
                found = found + 1

            try:
                counts_out = np.array(counts_out, dtype=np.uint32)

            except ValueError:
                self.log.warning('Value error: {0}'.format(counts_out))
                counts_out = np.zeros((self._gate_length_bins, ), dtype=np.uint32)
            except OverflowError:
                self.log.warning('Overflow error: {0}'.format(counts_out))
                counts_out = np.zeros((self._gate_length_bins, ), dtype=np.uint32)

        if counts_out is None:
            print('No counts out')
            found = 0
            counts_out = np.zeros((self._gate_length_bins, ), dtype=np.uint32)

        #self._count_data = np.empty((self._gate_length_bins,1))

        try:
            self._count_data += counts_out
        except ValueError:
            pass

        print(counts_out)


        info_dict = {'elapsed_sweeps': counter,
                     'elapsed_time': None}

        return self._count_data, info_dict

    def get_frequency(self):
        return 1000.




import logging

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