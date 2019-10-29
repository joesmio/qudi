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


class TrackingLogic(GenericLogic):

    """This is the Logic class for tracking bright features.
    """

    _modclass = 'trackinglogic'
    _modtype = 'logic'

    # declare connectors
    confocalscanner1 = Connector(interface='ConfocalScannerInterface')
    simplecounter = Connector(interface='CounterInterface')
    fitlogic = Connector(interface='FitLogic')

    # declare status vars
    _clock_frequency = StatusVar('clock_frequency', 50)
    return_slowness = StatusVar(default=20)
    refocus_XY_size = StatusVar('xy_size', 0.6e-6)
    optimizer_XY_res = StatusVar('xy_resolution', 10)
    refocus_Z_size = StatusVar('z_size', 2e-6)
    optimizer_Z_res = StatusVar('z_resolution', 30)
    hw_settle_time = StatusVar('settle_time', 0.1)
    optimization_sequence = StatusVar(default=['XY', 'Z'])
    do_surface_subtraction = StatusVar('surface_subtraction', False)
    surface_subtr_scan_offset = StatusVar('surface_subtraction_offset', 1e-6)
    opt_channel = StatusVar('optimization_channel', 0)

    # "private" signals to keep track of activities here in the optimizer logic
    _sigScanNextXyLine = QtCore.Signal()
    _sigScanZLine = QtCore.Signal()
    _sigCompletedXyOptimizerScan = QtCore.Signal()
    _sigDoNextOptimizationStep = QtCore.Signal()
    _sigFinishedAllOptimizationSteps = QtCore.Signal()

    # public signals
    sigImageUpdated = QtCore.Signal(str) # str is tag
    sigRefocusStarted = QtCore.Signal(str)
    sigRefocusXySizeChanged = QtCore.Signal()
    sigRefocusZSizeChanged = QtCore.Signal()
    sigRefocusFinished = QtCore.Signal(str, list)
    sigClockFrequencyChanged = QtCore.Signal(int)
    sigPositionChanged = QtCore.Signal(float, float, float)

    sigTimeUpdate = QtCore.Signal()

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
        self._scanning_device = self.get_connector('confocalscanner1')
        self._fit_logic = self.get_connector('fitlogic')
        self._simple_counter = self.get_connector('simplecounter')

        # Reads in the maximal scanning range. The unit of that scan range is micrometer!
        self.x_range = self._scanning_device.get_position_range()[0]
        self.y_range = self._scanning_device.get_position_range()[1]
        self.z_range = self._scanning_device.get_position_range()[2]

        self._initial_pos_x = 0.
        self._initial_pos_y = 0.
        self._initial_pos_z = 0.

        self.optim_pos_x = 0.
        self.optim_pos_y = 0.
        self.optim_pos_z = 0.

        self.wait_time = 60

        self.remaining_time = 0

        self.mode = 'None'

        # Initialization of optimization sequence step counter
        self._optimization_step = 0

        self.res = 67e-9
        self.points = 30

        self.active_tracking = True

        self.do_fit = 0

        self.av = 3

        self.just_track_z = False

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

    def check_optimization_sequence(self):
        """ Check the sequence of scan events for the optimization.
        """

        # Check the supplied optimization sequence only contains 'XY' and 'Z'
        if len(set(self.optimization_sequence).difference({'XY', 'Z'})) > 0:
            self.log.error('Requested optimization sequence contains unknown steps. Please provide '
                           'a sequence containing only \'XY\' and \'Z\' strings. '
                           'The default [\'XY\', \'Z\'] will be used.')
            self.optimization_sequence = ['XY', 'Z']



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


        if isinstance(initial_pos, (np.ndarray,)) and initial_pos.size >= 3:
            self._initial_pos_x, self._initial_pos_y, self._initial_pos_z = initial_pos[0:3]
        elif isinstance(initial_pos, (list, tuple)) and len(initial_pos) >= 3:
            self._initial_pos_x, self._initial_pos_y, self._initial_pos_z = initial_pos[0:3]
        elif initial_pos is None:
            scpos = self._scanning_device.get_scanner_position()[0:3]
            self._initial_pos_x, self._initial_pos_y, self._initial_pos_z = scpos
        else:
            pass  # TODO: throw error

        # Keep track of where the start_refocus was initiated
        self._caller_tag = caller_tag

        self.module_state.lock()

        self.optim_pos_x = self._initial_pos_x
        self.optim_pos_y = self._initial_pos_y
        self.optim_pos_z = self._initial_pos_z
        self.optim_sigma_x = 0.
        self.optim_sigma_y = 0.
        self.optim_sigma_z = 0.
        #
        self._xy_scan_line_count = 0
        self._optimization_step = 0

        self.n = 1

        self.check_optimization_sequence()




        self.clear_plots()

        self._scanning_device.update_optimise_parameters(res=self.res, points=self.points)

        scanner_status = self._scanning_device.start_optimiser()
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
        x0 = self.optim_pos_x
        y0 = self.optim_pos_y
        z0 = self.optim_pos_z

        self.refocus_Z_size = self.res * self.points

        xmin = np.clip(x0 - 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])
        xmax = np.clip(x0 + 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])
        ymin = np.clip(y0 - 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])
        ymax = np.clip(y0 + 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])
        zmin = np.clip(z0 - 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])
        zmax = np.clip(z0 + 0.5 * self.refocus_Z_size, self.z_range[0], self.z_range[1])


        self._x_values = np.linspace(xmin, xmax, num=self.points+1)
        self._y_values = np.linspace(ymin, ymax, num=self.points+1)
        self._z_values = np.linspace(zmin, zmax, num=self.points+1)

        self.x_track_line = np.zeros(
                len(self._x_values)-1)


        self.y_track_line = np.zeros(
            len(self._y_values)-1)

        self.z_track_line = np.zeros(
            len(self._z_values)-1)


        self.n = 1


    def ploddershittracker(self):

        '''
         dirty horrible tracker that is slow but should function
         '''

        x = self._scanning_device.curr_x
        y = self._scanning_device.curr_y
        z = self._scanning_device.curr_z
        matrix = np.zeros((self.points,3))

        self._scanning_device.scanner_lock = False


        for point in range(0,self.points):
            self._scanning_device.scanner_set_position(self._x_values1[point],y,z)
            matrix[point, 0] = self._simple_counter.get_counter()[[0]]

        for point in range(0,self.points):
            self._scanning_device.scanner_set_position(x,self._y_values1[point],z)
            matrix[point, 1] = self._simple_counter.get_counter()[[0]]

        for point in range(0,self.points):
            self._scanning_device.scanner_set_position(x,y,self._z_values1[point])
            matrix[point, 2] = self._simple_counter.get_counter()[[0]]

        self._scanning_device.scanner_set_position(x, y, z)

        return matrix



    def do_optimization(self):
        """ Do the z axis optimization."""
        # z scanning

        #self._scanning_device.update_optimise_parameters(res =self.res , points=self.points )

        # At the end fo the sequence, finish the optimization
        if self._optimization_step == len(self.optimization_sequence):
            self._sigFinishedAllOptimizationSteps.emit()
            return


        if self.stopRequested is True:
            self.module_state.unlock()
            self._scanning_device.scanner_lock = False
            self.stopRequested = False
            self._sigFinishedAllOptimizationSteps.emit()
            return


        while self.remaining_time > 0.1:
            time.sleep(0.1)
            self.remaining_time = self.remaining_time - 0.1
            self.sigTimeUpdate.emit()

            if self.stopRequested is True:
                self.module_state.unlock()
                self._scanning_device.scanner_lock = False
                self.stopRequested = False
                self._sigFinishedAllOptimizationSteps.emit()
                return


        self.remaining_time = self.wait_time
            # print('Next optimiser in {0:.1f} s'.format(self.remaining_time))



        xmin = self._scanning_device.curr_x - 0.5*self.res*self.points
        xmax = self._scanning_device.curr_x + 0.5*self.res*self.points

        ymin = self._scanning_device.curr_y  - 0.5*self.res*self.points
        ymax = self._scanning_device.curr_y + 0.5*self.res*self.points

        zmin = self._scanning_device.curr_z  - 0.5*2*self.res*self.points
        zmax = self._scanning_device.curr_z + 0.5*2*self.res*self.points

        print('xmin,xmax')
        print(xmin,xmax)

        #Find maximum
        #Set to new location

        #Find if values have shifted and by how many points from last time

        shiftx = int((self._x_values[0] - xmin)/(self.res))
        shifty = int((self._y_values[0] - ymin)/(self.res))
        shiftz = int((self._z_values[0] - zmin)/(self.res))

        # New values

        self._x_values = np.linspace(xmin, xmax, num=self.points+1)
        self._y_values = np.linspace(ymin, ymax, num=self.points+1)
        self._z_values = np.linspace(zmin, zmax, num=self.points+1)

        self._x_values1 = np.linspace(xmin, xmax, num=self.points)
        self._y_values1 = np.linspace(ymin, ymax, num=self.points)
        self._z_values1 = np.linspace(zmin, zmax, num=self.points)

        self._simple_counter.set_up_counter()
        print('Start value {0}'.format(self._simple_counter.get_counter()[[0]]))

        plodders = True

        if plodders is True:
            matrix = self.ploddershittracker()

            self.x_track_line = matrix[:self.points, 0]
            self.y_track_line = matrix[:self.points, 1]
            self.z_track_line = matrix[:self.points, 2]

            if self.stopRequested is True:
                self.module_state.unlock()
                self._scanning_device.scanner_lock = False
                self.stopRequested = False
                self._sigFinishedAllOptimizationSteps.emit()
                return


        else:
            matrix = self._scanning_device.next_optimiser()

            for i in range(1, self.av):
                matrix += self._scanning_device.next_optimiser()

                self.x_track_line = matrix[:self.points, 0] + np.flip(matrix[self.points:2 * self.points, 0], axis=0)
                self.y_track_line = matrix[:self.points, 1] + np.flip(matrix[self.points:2 * self.points, 1], axis=0)
                self.z_track_line = matrix[:self.points, 2] + np.flip(matrix[self.points:2 * self.points, 2], axis=0)
                self.sigImageUpdated.emit('tracking')

                if self.stopRequested is True:
                    self.module_state.unlock()
                    self._scanning_device.scanner_lock = False
                    self.stopRequested = False
                    self._sigFinishedAllOptimizationSteps.emit()
                    return


        max_val = 10e-6 - 0.5 * self.res * self.points

        min_val = -10e-6 +  0.5 * self.res * self.points

        # algorithm too susceptive to noise

        # reverting to gaussian update

        xcenter = self._scanning_device.curr_x
        ycenter = self._scanning_device.curr_y
        zcenter = self._scanning_device.curr_z

        print('z track line')

        print(self.z_track_line)

        if self.just_track_z is False:

            result = self._fit_logic.make_gaussianlinearoffset_fit(
                x_axis=self._x_values1,
                data=self.x_track_line,
                units='m',
                estimator=self._fit_logic.estimate_gaussianlinearoffset_peak
            )


            if result.success is True:
                print('X gaussian fit success')
                if result.best_values['center'] >= min_val and result.best_values['center'] <= max_val:
                    xcenter = result.best_values['center']
                # Change centroid

            result = self._fit_logic.make_gaussianlinearoffset_fit(
                x_axis=self._y_values1,
                data=self.y_track_line,
                units='m',
                estimator=self._fit_logic.estimate_gaussianlinearoffset_peak
            )


            if result.success is True:
                print('Y gaussian fit success')
                #print(result.result['center'].stderr)

                if result.best_values['center'] >= min_val and result.best_values['center'] <= max_val:
                    ycenter = result.best_values['center']
                    # Change centroid




        adjusted_param = {'offset': {
            'value': 1e-12,
            'min': -self.z_track_line.max(),
            'max': self.z_track_line.max()
        }}

        result = self._fit_logic.make_gaussianlinearoffset_fit(
            x_axis=self._z_values1,
            data=self.z_track_line,
            units='m',
            estimator = self._fit_logic.estimate_gaussianlinearoffset_peak
        )


        if result.success is True:
            print('Z gaussian fit success')
            if result.best_values['center'] >= min_val and result.best_values['center'] <= max_val:
                zcenter = result.best_values['center']
                # Change centroid

        print("new center now {0:.2f}, {1:.2f},{2:.2f} um".format(xcenter*1e6,ycenter*1e6,zcenter*1e6))

        # clear fits

        if self.active_tracking is True:

            old_x =  self._scanning_device.curr_x
            old_y = self._scanning_device.curr_y
            old_z = self._scanning_device.curr_z

            oldcounts = self._simple_counter.get_counter()[[0]]

            self._scanning_device.curr_x = xcenter
            self._scanning_device.curr_y = ycenter
            self._scanning_device.curr_z = zcenter

            if plodders is True:
                self._scanning_device.scanner_lock = False
                self._scanning_device.scanner_set_position(xcenter, ycenter,zcenter)


            else:
                self._scanning_device.update_optimiser_location()

            newcounts = self._simple_counter.get_counter()[[0]]

            print('new counts {0} old counts {1}'.format(newcounts,oldcounts))
            if newcounts < 0.9*oldcounts :
                print('New Counts < 0.9 * Old counts : Fail')
                self._scanning_device.curr_x = old_x
                self._scanning_device.curr_y = old_y
                self._scanning_device.curr_z = old_z
                self._scanning_device.update_optimiser_location()

        self._simple_counter.close_counter()
        self.sigImageUpdated.emit('tracking')
        self._sigDoNextOptimizationStep.emit()


    def finish_refocus(self):
        """ Finishes up and releases hardware after the optimizer scans."""
        #self._scanning_device.kill_scanner()
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


    def clear_plots(self):

        self.x_track_line = np.zeros(
            self.points)

        # self.smooth_x = np.zeros(
        #     len(self._x_values))

        self.y_track_line = np.zeros(
            self.points)

        self.z_track_line = np.zeros(
            self.points)