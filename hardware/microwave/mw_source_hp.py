# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control HP Device.

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

Parts of this file were developed from a PI3diamond module which is
Copyright (C) 2009 Helmut Rathgen <helmut.rathgen@gmail.com>

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import visa
import numpy as np
import time

from core.module import Base, ConfigOption
from interface.microwave_interface import MicrowaveInterface
from interface.microwave_interface import MicrowaveLimits
from interface.microwave_interface import MicrowaveMode
from interface.microwave_interface import TriggerEdge


class MicrowaveHP(Base, MicrowaveInterface):
    """ Hardware file for HP. Tested for the model 8341B. """

    _modclass = 'MicrowaveInterface'
    _modtype = 'hardware'

    _gpib_address = ConfigOption('gpib_address', missing='error')
    _gpib_timeout = ConfigOption('gpib_timeout', 10, missing='warn')

    # Indicate how fast frequencies within a list or sweep mode can be changed:
    _FREQ_SWITCH_SPEED = 0.009  # Frequency switching speed in s (acc. to specs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        # trying to load the visa connection to the module
        self.rm = visa.ResourceManager()
        try:
            self._gpib_connection = self.rm.open_resource(self._gpib_address)
        except:
            self.log.error('This is MWHP: could not connect to the GPIB address >>{}<<.'
                           ''.format(self._gpib_address))
            raise
        #self._gpib_connection.write('*RST')

        self.model =  self._gpib_connection.query('OI')
        self.log.info('MWHP initialised and connected to hardware.')

        # Settings must be locally saved because the SCPI interface of that device is too bad to
        # query those values.
        self._freq_list = list()
        self._list_power = -144
        self._cw_power = -144
        self._cw_frequency = 2870.0e6

        self._current_mode = 'cw'

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self._gpib_connection.close()
        self.rm.close()

    def get_limits(self):
        """Limits of Gigatronics 2400/2500 microwave source series.

          return MicrowaveLimits: limits of the particular  MW source model
        """
        limits = MicrowaveLimits()
        limits.supported_modes = (MicrowaveMode.CW, MicrowaveMode.SWEEP)

        limits.min_frequency = 100e3
        limits.max_frequency = 20e9

        limits.min_power = -144
        limits.max_power = 10

        limits.list_minstep = 1e5
        limits.list_maxstep = 1e9
        limits.list_maxentries = 1000

        limits.sweep_minstep = 1e5
        limits.sweep_maxstep = 1e9
        limits.sweep_maxentries = 1000

        return limits



    def off(self):
        """
        Switches off any microwave output.
        Must return AFTER the device is actually stopped.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write('RF0')

        return 0

    def get_status(self):
        """
        Gets the current status of the MW source, i.e. the mode (cw, list or sweep) and
        the output state (stopped, running)

        @return str, bool: mode ['cw', 'list', 'sweep'], is_running [True, False]
        
        
        Byte 5 indicates mode
        
        Byte 7, bit 5 indicates if RF off or not
        
        """

        is_running = self._gpib_connection.query('OM')[6] is " "

        mode = self._current_mode
        return mode, is_running

    def get_power(self):
        """
        Gets the microwave output power.

        @return float: the power set at the device in dBm
        """
        mode, dummy = self.get_status()
        if mode == 'list':
            return self._list_power
        else:
            return float(self._gpib_connection.query('OR'))

    def get_frequency(self):
        """
        Gets the frequency of the microwave output.
        Returns single float value if the device is in cw mode.
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of frequencies if the device is in list mode.

        @return [float, list]: frequency(s) currently set for this device in Hz
        """
        mode, is_running = self.get_status()
        if 'cw' in mode:
            return_val = float(self._gpib_connection.query('OK'))
        elif 'sweep' in mode:
            return_val = self._freq_list
        else:
            return_val = -1
        return return_val

    def cw_on(self):
        """ Switches on any preconfigured microwave output.

        @return int: error code (0:OK, -1:error)
        """
        mode, is_running = self.get_status()
        if is_running:
            if mode == 'cw':
                return 0
            else:
                self.off()

        if mode != 'cw':
            self.set_cw()

        self._gpib_connection.write('RF1')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_cw(self, frequency=None, power=None):
        """
        Configures the device for cw-mode and optionally sets frequency and/or power

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm

        @return float, float, str: current frequency in Hz, current power in dBm, current mode
        """
        mode, is_running = self.get_status()
        if is_running:
            self.off()

        self._current_mode = 'cw'

        if frequency is not None:
            self._gpib_connection.write('IPCW{0:e}GZ'.format(frequency/1e9))
        else:
            self._gpib_connection.write('IPCW{0:e}GZ'.format(self._cw_frequency/1e9))

        if power is not None:
            self._gpib_connection.write('PL{0:f}DB'.format(power))
        else:
            self._gpib_connection.write('PL{0:f}DB'.format(self._cw_power))

        mode, dummy = self.get_status()
        self._cw_frequency = self.get_frequency()
        self._cw_power = self.get_power()
        return self._cw_frequency, self._cw_power, mode

    def list_on(self):
        """
        Switches on the list mode microwave output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """

        return -1

    def set_list(self, frequency=None, power=None):
        """
        Configures the device for list-mode and optionally sets frequencies and/or power

        @param list frequency: list of frequencies in Hz
        @param float power: MW power of the frequency list in dBm

        @return list, float, str: current frequencies in Hz, current power in dBm, current mode
        """

        return -1, -1, -1, -1, ''

    def reset_listpos(self):#
        """ Reset of MW List Mode position to start from first given frequency

        @return int: error code (0:OK, -1:error)
        """

        return -1

    def set_ext_trigger(self, pol=TriggerEdge.RISING):
        """ Set the external trigger for this device with proper polarization.

        @param TriggerEdge pol: polarisation of the trigger (basically rising edge or
                        falling edge)

        @return int: error code (0:OK, -1:error)
        """
        return TriggerEdge.RISING

    def sweep_on(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write('RF1')
        self._current_mode = 'sweep'



        return 0

    def set_sweep(self, start=None, stop=None, step=None, power=None):
        """
        Configures the device for sweep-mode and optionally sets frequency start/stop/step
        and/or power

        @return float, float, float, float, str: current start frequency in Hz,
                                                 current stop frequency in Hz,
                                                 current frequency step in Hz,
                                                 current power in dBm,
                                                 current mode
        """
        num = round((stop - start) / step)
        self._gpib_connection.write('FA{0}GZFB{1}GZSN{2}'.format(start/1e9, stop/1e9, num))

      #  return self.mw_start_freq, self.mw_stop_freq, self.mw_step_freq, self.mw_sweep_power, \
       #        'sweep'

        return start, stop, step, power, \
           'sweep'

    def reset_sweeppos(self):
        """
        Reset of MW sweep mode position to start (start frequency)

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write('RS')

        return 0

    def trigger(self):
        """ Trigger the next element in the list or sweep mode programmatically.

        @return int: error code (0:OK, -1:error)

        Ensure that the Frequency was set AFTER the function returns, or give
        the function at least a save waiting time.
        """

        # WARNING:
        # The manual trigger functionality was not tested for this device!
        # Might not work well! Please check that!

        self._gpib_connection.write('IF')
        time.sleep(self._FREQ_SWITCH_SPEED)  # that is the switching speed
        return 0

