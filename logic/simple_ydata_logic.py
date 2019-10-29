# -*- coding: utf-8 -*-
"""
Buffer for simple data

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

from core.module import Connector
from logic.generic_logic import GenericLogic
from qtpy import QtCore


class SimpleDataLogic(GenericLogic):
    """ Logic module agreggating multiple hardware switches.
    """
    _modclass = 'smple_data'
    _modtype = 'logic'

    scanner = Connector(interface='ConfocalLogic')

    sigRepeat = QtCore.Signal()

    def on_activate(self):
        """ Prepare logic module for work.
        """
        self._data_logic = self.get_connector('scanner')
        self.stopRequest = False
        self.bufferLength = 10000
        self.sigRepeat.connect(self.measureLoop, QtCore.Qt.QueuedConnection)


        # The z scan values are just cribbed from the xy scan with the limits set by the y values
        self.zmin = -10e-6
        self.zmax = 10e-6
        self.res = self._data_logic.res

        self.inttime = 8

        #print('activating zdata logic')


    def on_deactivate(self):
        """ Deactivate modeule.
        """
        self.stopMeasure()

    def startMeasure(self):
        """ Start measurement: zero the buffer and call loop function."""
        self.window_len = 50
        self.n = 1
        self.buf = np.zeros(self.res)
        self.smooth = np.zeros(self.res)
        self.module_state.lock()

        # Zscan start


        #print(self.zmin, self.zmax)
        self._data_logic.start_yscan(zrange =[self.zmin, self.zmax], res = self.res, inttime=self.inttime)

        self.sigRepeat.emit()

    def stopMeasure(self):
        """ Ask the measurement loop to stop. """
        self.stopRequest = True

    def measureLoop(self):
        """ Measure 10 values, add them to buffer and remove the 10 oldest values.
        """
        if self.stopRequest:
            self._data_logic.kill_z()
            self.stopRequest = False
            self.module_state.unlock()
            return

        data = (np.asarray(self._data_logic.z_line()).T)[0,0:]
        self.buf[0:] = data
        self.smooth = ((self.n-1)*self.smooth + self.buf) / self.n
        self.n = self.n +1
        self.sigRepeat.emit()

