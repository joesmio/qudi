# -*- coding: utf-8 -*
"""
This file contains the Qudi logic class for optimizing scanner position.

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

from qtpy import QtCore
import numpy as np
import time

from logic.generic_logic import GenericLogic
from core.module import Connector, ConfigOption, StatusVar
from core.util.mutex import Mutex


class flLogic(GenericLogic):

    """This is the Logic class for tracking bright features.
    """

    _modclass = 'fllogic'
    _modtype = 'logic'

    # declare connectors
    counter = Connector(interface='SlowCounterInterface')
    fitlogic = Connector(interface='FitLogic')

    # declare status vars
    opt_channel = StatusVar('optimization_channel', 0)

    # "private" signals to keep track of activities here in the optimizer logic
    _sigScanNextXyLine = QtCore.Signal()
    _sigScanZLine = QtCore.Signal()
    _sigCompletedXyOptimizerScan = QtCore.Signal()
    _sigDoNextOptimizationStep = QtCore.Signal()
    _sigFinishedAllOptimizationSteps = QtCore.Signal()

    # public signals
    sigImageUpdated = QtCore.Signal()
    sigRefocusStarted = QtCore.Signal(str)
    sigRefocusXySizeChanged = QtCore.Signal()
    sigRefocusZSizeChanged = QtCore.Signal()
    sigRefocusFinished = QtCore.Signal(str, list)
    sigClockFrequencyChanged = QtCore.Signal(int)
    sigPositionChanged = QtCore.Signal(float, float, float)

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        # locking for thread safety
        self.threadlock = Mutex()

        self.stopRequested = False
        self.is_crosshair = True

        # Keep track of who called the refocus
        self._caller_tag = ''

    def on_activate(self):
        """ Initialisation performed during activation of the module.

        @return int: error code (0:OK, -1:error)
        """
        self._counting_device = self.get_connector('counter')
        self._fit_logic = self.get_connector('fitlogic')

        # Reads in the maximal scanning range. The unit of that scan range is micrometer!
        self.x_range = [0, 800e-9]

        self._initial_pos_x = 0.

        self.optim_pos_x = 0.

        self.points = 640

        binsize = 0.5e-9

        self.sample = 64 * binsize / 1e-9


        self.mode = 'None'

        # Initialization of optimization sequence step counter
        self._optimization_step = 0

        # Sets connections between signals and functions
        self._sigScanZLine.connect(self.do_optimization, QtCore.Qt.QueuedConnection)

        self._sigDoNextOptimizationStep.connect(self.do_optimization, QtCore.Qt.QueuedConnection)
        self._sigFinishedAllOptimizationSteps.connect(self.finish_refocus)

        self._initialize_z_refocus_image()
        return 0

    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        return 0


    def set_refocus_Z_size(self, size):
        """ Set the number of values for Z refocus

            @param int size: number of values for Z refocus
        """
        self.refocus_Z_size = size
        self.sigRefocusZSizeChanged.emit()


    def start_refocus(self, initial_pos=None, caller_tag='unknown', tag='logic'):
        """ Starts the optimization scan around initial_pos

            @param list initial_pos: with the structure [float, float, float]
            @param str caller_tag:
            @param str tag:
        """
        # checking if refocus corresponding to crosshair or corresponding to initial_pos




        # Keep track of where the start_refocus was initiated
        self._caller_tag = caller_tag

        self.module_state.lock()

        self.optim_pos_x = self._initial_pos_x
        self.optim_sigma_x = 0.

        #
        self._xy_scan_line_count = 0
        self._optimization_step = 0

        self.n = 1

        scanner_status = self._counting_device.set_up_counter()
        if scanner_status < 0:
            self.sigRefocusFinished.emit(
                self._caller_tag,
                [self.optim_pos_x, self.optim_pos_y, self.optim_pos_z, 0])
            return


        self.sigRefocusStarted.emit(tag)
        self._sigDoNextOptimizationStep.emit()

    def stop_refocus(self):
        """Stops refocus."""
        with self.threadlock:
            self.stopRequested = True


    def _initialize_z_refocus_image(self):
        """Initialisation of the z refocus image."""

        # Take optim pos as center of refocus image, to benefit from any previous
        # optimization steps that have occurred.

        xmin = 0
        xmax = 800e-9

        self._x_values = np.linspace(xmin, xmax, num=640) #self.points/self.sample)

        self.x_track_line = np.zeros(
                len(self._x_values))

        self.smooth_x = np.zeros(
            len(self._x_values))


        self.n = 1

    def clear(self):
        self.x_track_line = np.zeros(
            len(self._x_values))

    def do_optimization(self):
        """ Do the z axis optimization."""
        # z scanning

        #self._counting_device.update_optimise_parameters(res =self.res , points=self.points )

        if self.stopRequested is True:
            self.module_state.unlock()
            self.stopRequested = False
            self._sigFinishedAllOptimizationSteps.emit()
            return

        #binsize = 2e-9

        #sample = 1# 64*binsize/1e-9

        matrix = self._counting_device.get_counter()
       # get optim position

        #print(matrix)

        if matrix:
            #matrix.reshape(sample, self.points / sample).sum(axis=1)
            self.x_track_line += matrix

        xmin = 0
        xmax = 800e-9

        self._x_values = np.linspace(xmin, xmax, num=640) #self.points/sample)


        self.sigImageUpdated.emit()
        self._sigDoNextOptimizationStep.emit()

    def finish_refocus(self):
        """ Finishes up and releases hardware after the optimizer scans."""
        #self._counting_device.kill_scanner()
        pass

    def _do_next_optimization_step(self):
        """Handle the steps through the specified optimization sequence
        """

        # Read the next step in the optimization sequence
        this_step = self.optimization_sequence[self._optimization_step]

        # Increment the step counter
        self._optimization_step += 1

        # Launch the next step
        self._initialize_z_refocus_image()
        self._sigScanZLine.emit()


    def set_position(self, tag, x=None, y=None, z=None, a=None):
        """ Set focus position.
            @param str tag: sting indicating who caused position change
            @param float x: x axis position in m
            @param float y: y axis position in m
            @param float z: z axis position in m
            @param float a: a axis position in m
        """
        if x is not None:
            self._current_x = x
        if y is not None:
            self._current_y = y
        if z is not None:
            self._current_z = z
        self.sigPositionChanged.emit(self._current_x, self._current_y, self._current_z)
