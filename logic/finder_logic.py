# -*- coding: utf-8 -*-
"""
This module operates a confocal microsope.

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


from collections import OrderedDict

from core import config

import os

from copy import copy
import time
import datetime
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from io import BytesIO

from logic.generic_logic import GenericLogic
from core.util.mutex import Mutex
from core.module import Connector, ConfigOption, StatusVar

from datetime import datetime



class OldConfigFileError(Exception):
    """ Exception that is thrown when an old config file is loaded.
    """
    def __init__(self):
        super().__init__('Old configuration file detected. Ignoring confocal history.')


class ConfocalHistoryEntry(QtCore.QObject):
    """ This class contains all relevant parameters of a Confocal scan.
        It provides methods to extract, restore and serialize this data.
    """

    def __init__(self, confocal):
        """ Make a confocal data setting with default values. """
        super().__init__()

        self.depth_scan_dir_is_xz = True
        self.depth_img_is_xz = True

        self.xy_line_pos = 0
        self.depth_line_pos = 0

        # Reads in the maximal scanning range. The unit of that scan range is meters!
        self.x_range = [0e-3, 1e-3] #confocal._motor.get_position_range()[0]
        self.y_range = [0e-3, 1e-3] #confocal._motor.get_position_range()[1]

        self.z_range = [-20e-6, 0e-6] #confocal._motor.get_position_range()[2]


        self.motor_range =  [0e-3, 6e-3] #confocal._motor.get_position_range()[1]
        self.piezo_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[2]

        # Sets the current position to the center of the maximal scanning range
        self.current_x = (self.motor_range[1]) / 2
        self.current_y = (self.motor_range[1]) / 2
        self.current_z = (self.motor_range[1]) / 2
        self.current_a = 0.0

        self._current_xp = (self.piezo_range[1]) / 2
        self._current_yp = (self.piezo_range[1]) / 2
        self._current_zp = (self.piezo_range[1]) / 2

        # Sets the size of the image to the maximal scanning range
        self.image_x_range = [-10e-6, 10e-6]
        self.image_y_range = [-10e-6, 10e-6]
        self.image_z_range = self.z_range

        self.spx_size = 130
        self.spx_overlap = 0

        # Default values for the resolution of the scan
        self.xy_resolution = 15
        self.z_resolution = 50

        # Initialization of internal counter for scanning
        self.xy_line_position = 0
        self.depth_line_position = 0

        # Variable to check if a scan is continuable
        self.scan_counter = 0
        self.xy_scan_continuable = False
        self.depth_scan_continuable = False

        # tilt correction stuff:
        self.tilt_correction = False
        # rotation point for tilt correction
        self.tilt_reference_x = 0.5 * (self.x_range[0] + self.x_range[1])
        self.tilt_reference_y = 0.5 * (self.y_range[0] + self.y_range[1])
        # sample slope
        self.tilt_slope_x = 0
        self.tilt_slope_y = 0
        # tilt correction points
        self.point1 = np.array((0, 0, 0))
        self.point2 = np.array((0, 0, 0))
        self.point3 = np.array((0, 0, 0))
        self.tilt_correction = True
        self.tilt_slope_x = 0
        self.tilt_slope_y = 0
        self.tilt_reference_x = 0
        self.tilt_reference_y = 0

        self.psf_deltaxy = 3e-6
        self.psf_deltaz = 1e-6
        self.psf_deltares = 100e-9
        self.current_pointer = 'History'
        self.xy_image = OrderedDict()
        self.xy_image[self.current_pointer] = OrderedDict()
        self.xy_image[self.current_pointer]['image'] =np.zeros((
            self.spx_size,
            self.spx_size,
                3 + 1
            ))


    def restore(self, confocal):
        """ Write data back into confocal logic and pull all the necessary strings """
        confocal._current_x = self.current_x
        confocal._current_y = self.current_y
        confocal._current_z = self.current_z
        confocal._current_a = self.current_a
        confocal.image_x_range = np.copy(self.image_x_range)
        confocal.image_y_range = np.copy(self.image_y_range)
        confocal.image_z_range = np.copy(self.image_z_range)
        confocal.spx_overlap = self.spx_overlap
        confocal.spx_size = self.spx_size
        confocal.xy_resolution = self.xy_resolution
        confocal.z_resolution = self.z_resolution
        confocal.depth_img_is_xz = self.depth_img_is_xz
        confocal.depth_scan_dir_is_xz = self.depth_scan_dir_is_xz
        confocal._xy_line_pos = self.xy_line_position
        confocal._depth_line_pos = self.depth_line_position
        confocal._xyscan_continuable = self.xy_scan_continuable
        confocal._zscan_continuable = self.depth_scan_continuable
        confocal._scan_counter = self.scan_counter
        confocal.point1 = np.copy(self.point1)
        confocal.point2 = np.copy(self.point2)
        confocal.point3 = np.copy(self.point3)
        confocal._scanning_device.tilt_variable_ax = self.tilt_slope_x
        confocal._scanning_device.tilt_variable_ay = self.tilt_slope_y
        confocal._scanning_device.tilt_reference_x = self.tilt_reference_x
        confocal._scanning_device.tilt_reference_y = self.tilt_reference_y
        confocal._scanning_device.tiltcorrection = self.tilt_correction

        confocal.initialize_image()

        confocal.xy_image = self.xy_image.copy()

        # try:
        #     if confocal.xy_image.shape == self.xy_image.shape:
        #
        # except AttributeError:
        #     self.xy_image = np.copy(confocal.xy_image)


        confocal._zscan = False

    def snapshot(self, confocal):
        """ Extract all necessary data from a confocal logic and keep it for later use """
        self.current_x = confocal._current_x
        self.current_y = confocal._current_y
        self.current_z = confocal._current_z
        self.current_a = confocal._current_a
        self.image_x_range = np.copy(confocal.image_x_range)
        self.image_y_range = np.copy(confocal.image_y_range)
        self.image_z_range = np.copy(confocal.image_z_range)
        self.spx_size = confocal.spx_size
        self.spx_overlap = confocal.spx_overlap
        self.depth_scan_dir_is_xz = confocal.depth_scan_dir_is_xz
        self.depth_img_is_xz = confocal.depth_img_is_xz
        self.xy_line_position = confocal._xy_line_pos
        self.depth_line_position = confocal._depth_line_pos
        self.xy_scan_continuable = confocal._xyscan_continuable
        self.depth_scan_continuable = confocal._zscan_continuable
        self.scan_counter = confocal._scan_counter
        self.tilt_correction = confocal._scanning_device.tiltcorrection
        self.tilt_slope_x = confocal._scanning_device.tilt_variable_ax
        self.tilt_slope_y = confocal._scanning_device.tilt_variable_ay
        self.tilt_reference_x = confocal._scanning_device.tilt_reference_x
        self.tilt_reference_y = confocal._scanning_device.tilt_reference_y
        self.point1 = np.copy(confocal.point1)
        self.point2 = np.copy(confocal.point2)
        self.point3 = np.copy(confocal.point3)
        self.xy_image = np.copy(confocal.xy_image)

    def serialize(self):
        """ Give out a dictionary that can be saved via the usual means """
        serialized = dict()
        serialized['focus_position'] = [self.current_x, self.current_y, self.current_z, self.current_a]
        serialized['x_range'] = list(self.image_x_range)
        serialized['y_range'] = list(self.image_y_range)
        serialized['z_range'] = list(self.image_z_range)
        serialized['spx_size'] = self.spx_size
        serialized['spx_overlap'] = self.spx_overlap
        serialized['depth_img_is_xz'] = self.depth_img_is_xz
        serialized['depth_dir_is_xz'] = self.depth_scan_dir_is_xz
        serialized['xy_line_position'] = self.xy_line_position
        serialized['depth_line_position'] = self.depth_line_position
        serialized['xy_scan_cont'] = self.xy_scan_continuable
        serialized['depth_scan_cont'] = self.depth_scan_continuable
        serialized['scan_counter'] = self.scan_counter
        serialized['tilt_correction'] = self.tilt_correction
        serialized['tilt_point1'] = list(self.point1)
        serialized['tilt_point2'] = list(self.point2)
        serialized['tilt_point3'] = list(self.point3)
        serialized['tilt_reference'] = [self.tilt_reference_x, self.tilt_reference_y]
        serialized['tilt_slope'] = [self.tilt_slope_x, self.tilt_slope_y]
        try:
            serialized['image_number'] = self._spx_counter
        except AttributeError:
            serialized['image_number'] = 0

        serialized['xy_images'] = self.xy_image
        return serialized

    def deserialize(self, serialized):
        """ Restore Confocal history object from a dict """
        if 'focus_position' in serialized and len(serialized['focus_position']) == 4:
            self.current_x = serialized['focus_position'][0]
            self.current_y = serialized['focus_position'][1]
            self.current_z = serialized['focus_position'][2]
            self.current_a = serialized['focus_position'][3]
        if 'x_range' in serialized and len(serialized['x_range']) == 2:
            self.image_x_range = serialized['x_range']
        if 'y_range' in serialized and len(serialized['y_range']) == 2:
            self.image_y_range = serialized['y_range']
        if 'z_range' in serialized and len(serialized['z_range']) == 2:
            self.image_z_range = serialized['z_range']
        if 'spx_size' in serialized:
            self.spx_size = serialized['spx_size']
        if 'spx_overlap' in serialized:
            self.spx_overlap = serialized['spx_overlap']
        if 'depth_img_is_xz' in serialized:
            self.depth_img_is_xz = serialized['depth_img_is_xz']
        if 'depth_dir_is_xz' in serialized:
            self.depth_scan_dir_is_xz = serialized['depth_dir_is_xz']
        if 'tilt_correction' in serialized:
            self.tilt_correction = serialized['tilt_correction']
        if 'tilt_reference' in serialized and len(serialized['tilt_reference']) == 2:
            self.tilt_reference_x = serialized['tilt_reference'][0]
            self.tilt_reference_y = serialized['tilt_reference'][1]
        if 'tilt_slope' in serialized and len(serialized['tilt_slope']) == 2:
            self.tilt_slope_x = serialized['tilt_slope'][0]
            self.tilt_slope_y = serialized['tilt_slope'][1]
        if 'tilt_point1' in serialized and len(serialized['tilt_point1']) == 3:
            self.point1 = np.array(serialized['tilt_point1'])
        if 'tilt_point2' in serialized and len(serialized['tilt_point2']) == 3:
            self.point2 = np.array(serialized['tilt_point2'])
        if 'tilt_point3' in serialized and len(serialized['tilt_point3']) == 3:
            self.point3 = np.array(serialized['tilt_point3'])
        if 'image_number' in serialized:
            self._spx_counter = serialized['image_number']
        #if 'xy_images' in serialized:
            #self.xy_image = serialized['xy_images']
            # if isinstance(serialized['xy_image'], np.ndarray):
            #     self.xy_image = serialized['xy_image']
            # else:
            #     raise OldConfigFileError()
        if 'depth_image' in serialized:
            if isinstance(serialized['depth_image'], np.ndarray):
                self.depth_image = serialized['depth_image'].copy()
            else:
                raise OldConfigFileError()


class ConfocalLogic(GenericLogic):
    """
    This is the Logic class for confocal scanning.
    """
    _modclass = 'confocallogic'
    _modtype = 'logic'

    # declare connectors
    confocalscanner1 = Connector(interface='ConfocalScannerInterface')
    savelogic = Connector(interface='SaveLogic')
    fitlogic = Connector(interface='FitLogic')
    motor = Connector(interface = 'MotorInterface')
    rot = Connector(interface = 'MotorInterface')

    # status vars
    _clock_frequency = StatusVar('clock_frequency', 500)
    return_slowness = StatusVar(default=50)
    max_history_length = StatusVar(default=10)

    # signals
    signal_start_scanning = QtCore.Signal(str)
    signal_continue_scanning = QtCore.Signal(str)
    signal_stop_scanning = QtCore.Signal()
    signal_scan_lines_next = QtCore.Signal()
    signal_xy_image_updated = QtCore.Signal()
    signal_depth_image_updated = QtCore.Signal()
    signal_change_position = QtCore.Signal(str)
    signal_xy_data_saved = QtCore.Signal()
    signal_depth_data_saved = QtCore.Signal()
    signal_tilt_correction_active = QtCore.Signal(bool)
    signal_tilt_correction_update = QtCore.Signal()
    signal_draw_figure_completed = QtCore.Signal()
    signal_position_changed = QtCore.Signal()

    signal_spx_next = QtCore.Signal()

    signal_3d_next = QtCore.Signal()

    sigImageXYInitialized = QtCore.Signal()
    sigImageDepthInitialized = QtCore.Signal()

    signal_history_event = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

        # counter for scan_image
        self._scan_counter = 0
        self._zscan = False
        self.stopRequested = False
        self.depth_scan_dir_is_xz = True
        self.depth_img_is_xz = True
        self.permanent_scan = False

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._scanning_device = self.get_connector('confocalscanner1')
        self._save_logic = self.get_connector('savelogic')
        self._fit_logic = self.get_connector('fitlogic')

        self._motor = self.get_connector('motor')
        self._rot  = self.get_connector('rot')

        self.dynamic_updating_status = 0

        self.lock = Mutex(recursive=True)

        # Reads in the maximal scanning range. The unit of that scan range is micrometer!
        # Reads in the maximal scanning range. The unit of that scan range is meters!
        self.x_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[0]
        self.y_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[1]
        self.z_range = [0e-6, 6e-3]  # confocal._motor.get_position_range()[2]

        self.scan_x_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[0]
        self.scan_y_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[1]

        self.spx_grid = [(0,0)]
        self._spx_counter = 0

        self.dynamic_z = False

        self.stitch_range = 160e-6

        self.stopsuperRequested = False


        #self.current_pointer = self.generate_pointer()

        # TODO: Make these variables from history
        self.spx_size = 130
        self.spx_overlap = 0
        self.spx_x_range = [-10e-6, 10e-6]
        self.spx_y_range = [-10e-6, 10e-6]

        self.xy_image = OrderedDict()
        try:
            self.xy_image[self.current_pointer] = OrderedDict()
        except AttributeError:
            self.current_pointer = 'History'
            self.xy_image[self.current_pointer] = OrderedDict()

        self.xy_image[self.current_pointer]['image'] = np.zeros((
            self.spx_size,
            self.spx_size,
                3 + len(self.get_scanner_count_channels())
            ))

        self.psf_deltaxy = 3e-6
        self.psf_deltaz = 1e-6
        self.psf_deltares = 100e-9

        self.psf_xyres = int(self.psf_deltaxy / self.psf_deltares)
        self.psf_zres = int(self.psf_deltaz / self.psf_deltares)

        self.psf_data = np.ones((self.psf_xyres,self.psf_xyres,self.psf_zres))

        self.motor_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[1]
        self.piezo_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[2]

        #self.psf_data[i,:,:,:] =i

        # #Reds
        # self.psf_data[:,:,:,0] = 0
        # #Greens
        # self.psf_data[:, :, :, 1] = 0
        # #Blues
        # self.psf_data[:, :, :, 2] = 0
        # #Alphas
        # self.psf_data[:, :, :, 3] = 4#0.4#*255

        self.initial_jog_x_step = 10e-6
        self.initial_jog_y_step = 10e-6
        self.initial_jog_z_step = 10e-6

        self.change_jog_step({'x':self.initial_jog_x_step, 'y':self.initial_jog_y_step, 'z':self.initial_jog_z_step })


        self.res = 130

        self.superdict = self.get_motor_position()

        self.piezo_x_range = [-20e-6, 0]
        self.piezo_y_range =  [-20e-6, 0]

        self.singlemode = False

        self.tilt_reference_x = 0
        self.tilt_variable_ax = 0
        self.tilt_reference_y = 0
        self.tilt_variable_ay = 0



        self.pois = {}



        # restore here ...
        self.history = []
        for i in reversed(range(1, self.max_history_length)):
            try:
                new_history_item = ConfocalHistoryEntry(self)
                new_history_item.deserialize(
                    self._statusVariables['history_{0}'.format(i)])
                self.history.append(new_history_item)
            except KeyError:
                pass
            except OldConfigFileError:
                self.log.warning(
                    'Old style config file detected. History {0} ignored.'.format(i))
            except:
                self.log.warning(
                        'Restoring history {0} failed.'.format(i))
        try:
            new_state = ConfocalHistoryEntry(self)
            new_state.deserialize(self._statusVariables['history_0'])
            new_state.restore(self)
        except:
            new_state = ConfocalHistoryEntry(self)
            new_state.restore(self)
        finally:
            self.history.append(new_state)

        self.history_index = len(self.history) - 1

        # Sets connections between signals and functions
        self.signal_scan_lines_next.connect(self._scan_line, QtCore.Qt.QueuedConnection)
        self.signal_start_scanning.connect(self.start_scanner, QtCore.Qt.QueuedConnection)
        self.signal_continue_scanning.connect(self.continue_scanner, QtCore.Qt.QueuedConnection)

        self.signal_stop_scanning.connect(self.after_piezo_scan, QtCore.Qt.QueuedConnection)

        #self._change_position('activation')

        self.signal_spx_next.connect(self._scan_super_px, QtCore.Qt.QueuedConnection)

        self.signal_3d_next.connect(self.next_3d_line, QtCore.Qt.QueuedConnection)

    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        closing_state = ConfocalHistoryEntry(self)
        closing_state.snapshot(self)
        self.history.append(closing_state)
        histindex = 0
        for state in reversed(self.history):
            self._statusVariables['history_{0}'.format(histindex)] = state.serialize()
            histindex += 1

        if self._scanning_device.module_state.current is 'locked':
            self._scanning_device.module_state.unlock()

        if self.module_state.current is 'locked':
            self.module_state.unlock()



        self.signal_scan_lines_next.disconnect(self._scan_line)
        self.signal_start_scanning.disconnect(self.start_scanner)
        self.signal_continue_scanning.disconnect(self.continue_scanner)

        self.signal_stop_scanning.disconnect(self.after_piezo_scan)

        self.signal_3d_next.disconnect(self.next_3d_line)


        return 0

    def switch_hardware(self, to_on=False):
        """ Switches the Hardware off or on.

        @param to_on: True switches on, False switched off

        @return int: error code (0:OK, -1:error)
        """
        if to_on:
            return self._scanning_device.activation()
        else:
            return self._scanning_device.reset_hardware()

    def set_clock_frequency(self, clock_frequency):
        """Sets the frequency of the clock

        @param int clock_frequency: desired frequency of the clock

        @return int: error code (0:OK, -1:error)
        """
        self._clock_frequency = int(clock_frequency)
        #checks if scanner is still running
        if self.module_state() == 'locked':
            return -1
        else:
            return 0

    def start_scanning(self, zscan = False, tag='logic'):
        """Starts scanning

        @param bool zscan: zscan if true, xyscan if false

        @return int: error code (0:OK, -1:error)
        """
        # TODO: this is dirty, but it works for now
        #while self.module_state() == 'locked':
        #    time.sleep(0.01)
        self._scan_counter = 0
        self._zscan = zscan
        if self._zscan:
            self._zscan_continuable = True
        else:
            self._xyscan_continuable = True

        self.signal_start_scanning.emit(tag)
        return 0

    def continue_scanning(self,zscan,tag='logic'):
        """Continue scanning

        @return int: error code (0:OK, -1:error)
        """
        self._zscan = zscan
        if zscan:
            self._scan_counter = self._depth_line_pos
        else:
            self._scan_counter = self._xy_line_pos
        self.signal_continue_scanning.emit(tag)
        return 0

    def stop_scanning(self):
        """Stops the scan

        @return int: error code (0:OK, -1:error)
        """

        # with self.threadlock:
        #     if self.module_state() == 'locked':
        #         print('module state is locked so we are requesting a stop')
        #         self.stopRequested = True
        self.signal_stop_scanning.emit()

        return 0

    def initialize_image(self):
        """Initalization of the image.

        @return int: error code (0:OK, -1:error)
        """


        if self.singlemode is True:

            print('SINGLE MODE ACTIVATED')

            current_pos = self.get_piezo_position()

            xmin = current_pos[0] + self.scan_x_range[0]
            xmax = current_pos[0] + self.scan_x_range[1]
            ymin = current_pos[1] + self.scan_y_range[0]
            ymax = current_pos[1] + self.scan_y_range[1]

            xmin = np.clip(xmin, *self.piezo_x_range)
            xmax = np.clip(xmax, *self.piezo_x_range)
            ymin = np.clip(ymin, *self.piezo_y_range)
            ymax = np.clip(ymax, *self.piezo_y_range)

            x1, x2 = self.spx_grid[self._spx_counter][0]+xmin, self.spx_grid[self._spx_counter][0]+xmax
            y1, y2 = self.spx_grid[self._spx_counter][1]+ymin, self.spx_grid[self._spx_counter][1]+ymax

        else:


            # x1: x-start-value, x2: x-end-value
            try:
                x1, x2 = self.spx_grid[self._spx_counter][0]+self.spx_x_range[0],  self.spx_grid[self._spx_counter][0]+self.spx_x_range[1]
                # y1: x-start-value, y2: x-end-value
                y1, y2 =  self.spx_grid[self._spx_counter][1]+self.spx_y_range[0],  self.spx_grid[self._spx_counter][1]+self.spx_y_range[1]
                # z1: x-start-value, z2: x-end-value
               # z1, z2 = self.image_z_range[0], self.image_z_range[1]
            except IndexError:

                # Must only want 1 scan
                self.log.info('Requested single scan')
                xmin = self.scan_x_range[0]
                xmax = self.scan_x_range[1]
                ymin = self.scan_y_range[0]
                ymax = self.scan_y_range[1]

                xmin = np.clip(xmin, *self.piezo_x_range)
                xmax = np.clip(xmax, *self.piezo_x_range)
                ymin = np.clip(ymin, *self.piezo_y_range)
                ymax = np.clip(ymax, *self.piezo_y_range)

                x1, x2 = self.spx_grid[self._spx_counter][0] + xmin, self.spx_grid[self._spx_counter][0] + xmax
                y1, y2 = self.spx_grid[self._spx_counter][1] + ymin, self.spx_grid[self._spx_counter][1] + ymax

                #self.log.error('Scan range given is incorrect')
                return -1
            #print([x1, x2])
            #print([y1, y2])

        # Checks if the x-start and x-end value are ok
        if x2 < x1:
            self.log.error(
                'x1 must be smaller than x2, but they are '
                '({0:.3f},{1:.3f}).'.format(x1, x2))
            return -1


        # Checks if the y-start and y-end value are ok
        if y2 < y1:
            self.log.error(
                'y1 must be smaller than y2, but they are '
                '({0:.3f},{1:.3f}).'.format(y1, y2))
            return -1


        self._X = np.linspace(x1, x2, self.spx_size)
        self._Y = np.linspace(y1, y2, self.spx_size)
        #
        # # prevents distortion of the image
        # if (x2 - x1) >= (y2 - y1):
        #     self._X = np.linspace(x1, x2, max(self.spx_size, 2))
        #     self._Y = np.linspace(y1, y2, max(int(self.spx_size*(y2-y1)/(x2-x1)), 2))
        # else:
        #     self._Y = np.linspace(y1, y2, max(self.spx_size, 2))
        #     self._X = np.linspace(x1, x2, max(int(self.spx_size*(x2-x1)/(y2-y1)), 2))

        self._XL = self._X
        self._YL = self._Y
        self._AL = np.zeros(self._XL.shape)

        # Arrays for retrace line
        self._return_XL = np.linspace(self._XL[-1], self._XL[0], self.return_slowness)
        self._return_AL = np.zeros(self._return_XL.shape)

        self._image_horz_axis = self._X
        self._image_vert_axis = self._Y
        # creats an image where each pixel will be [x,y,z,counts]

        self.current_pointer = self.generate_pointer()
        #print('Generating pointer as {0}'.format(self.current_pointer))

        self.xy_image[self.current_pointer] = OrderedDict()

        self.xy_image[self.current_pointer]['label'] = self.current_pointer

        self.xy_image[self.current_pointer]['image'] = np.zeros((
            self.spx_size,
            self.spx_size,
                3 + len(self.get_scanner_count_channels())
            ))

        motor_position = self.get_motor_position()
        self.xy_image[self.current_pointer]['x'] = motor_position[0]
        self.xy_image[self.current_pointer]['y'] = motor_position[1]
        self.xy_image[self.current_pointer]['z'] = motor_position[2]

        angle =self.get_angle()
        self.xy_image[self.current_pointer]['pol'] = angle

        self.xy_image[self.current_pointer]['res'] = self.spx_size
        self.xy_image[self.current_pointer]['fl'] =   np.zeros((
            self.spx_size,
            self.spx_size,
            640
        ))

        self.xy_image[self.current_pointer]['image'][:, :, 0] = np.full(
            (len(self._image_vert_axis), len(self._X)), self._XL)

        y_value_matrix = np.full((len(self._X), len(self._image_vert_axis)), self._Y)
        self.xy_image[self.current_pointer]['image'][:, :, 1] = y_value_matrix.transpose()

        self.xy_image[self.current_pointer]['image'][:, :, 2] = self._current_z * np.ones(
            (len(self._image_vert_axis), len(self._X)))

        self.sigImageXYInitialized.emit()
        return 0


    def generate_pointer(self, label=None):

        d = datetime.now()

        if label is not None:
            name = d.strftime("%H%M%S_%d%m%y") + "_" + str(label)
        else:
            name = d.strftime("%H%M%S_%d%m%y")

        # generates a unique string
        return name



    def start_scanner(self):
        """Setting up the scanner device and starts the scanning procedure

        @return int: error code (0:OK, -1:error)
        """

        self.stopRequested = False

        clock_status = self._scanning_device.set_up_scanner_clock(
            clock_frequency=self._clock_frequency)

        if clock_status < 0:
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_position('scanner')
            return -1


        if self.singlemode is True:

            #Get current position, bound around that

            current_pos = self.get_piezo_position()

            xmin = current_pos[0] + self.scan_x_range[0]
            xmax = current_pos[0] + self.scan_x_range[1]
            ymin = current_pos[1] + self.scan_y_range[0]
            ymax = current_pos[1] + self.scan_y_range[1]

            xmin = np.clip(xmin, *self.spx_x_range)
            xmax = np.clip(xmax, *self.spx_x_range)
            ymin = np.clip(ymin, *self.spx_y_range)
            ymax = np.clip(ymax, *self.spx_y_range)



            scanner_status = self._scanning_device.set_up_scanner(res = self.spx_size, xrange = [xmin, xmax], yrange = [ymin, ymax])

        else:
            scanner_status = self._scanning_device.set_up_scanner(res = self.spx_size, xrange = self.spx_x_range, yrange = self.spx_y_range)

        print('test test')

        self.initialize_image()

        if scanner_status < 0:
            self._scanning_device.close_scanner_clock()
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_position('scanner')
            return -1

        self.signal_scan_lines_next.emit()
        return 0

    def continue_scanner(self):
        """Continue the scanning procedure

        @return int: error code (0:OK, -1:error)
        """

        self.log.error('Do not use continue scan yet')
        self.module_state.lock()
        self._scanning_device.module_state.lock()

        clock_status = self._scanning_device.set_up_scanner_clock(
            clock_frequency=self._clock_frequency)

        if clock_status < 0:
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_position('scanner')
            return -1

        scanner_status = self._scanning_device.set_up_scanner()

        if scanner_status < 0:
            self._scanning_device.close_scanner_clock()
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_position('scanner')
            return -1

        self.signal_scan_lines_next.emit()
        return 0

    def kill_scanner(self):
        """Closing the scanner device.

        @return int: error code (0:OK, -1:error)

        """


        try:
            print('Killing the scanner')
            self._scanning_device.close_scanner()
        except Exception as e:
            self.log.exception('Could not close the scanner.')
        try:
            self._scanning_device.close_scanner_clock()
        except Exception as e:
            self.log.exception('Could not close the scanner clock.')
        try:
            self._scanning_device.module_state.unlock()
            self._scanning_device.scanner_lock = False

        except Exception as e:
            self.log.exception('Could not unlock scanning device.')

        return 0

    def set_position(self, tag, x=None, y=None, z=None, a=None):
        """Forwarding the desired new position from the GUI to the scanning device.

        @param string tag: TODO

        @param float x: if defined, changes to postion in x-direction (microns)
        @param float y: if defined, changes to postion in y-direction (microns)
        @param float z: if defined, changes to postion in z-direction (microns)
        @param float a: if defined, changes to postion in a-direction (microns)

        @return int: error code (0:OK, -1:error)
        """
        # Changes the respective value

        if x is not None:
            self._current_xp = x
        if y is not None:
            self._current_yp = y
        if z is not None:
            self._current_zp = z
        if a is not None:
            self._current_a = a

        # Checks if the scanner is still running
        if self.module_state() == 'locked' or self._scanning_device.module_state() == 'locked':
            return -1
        else:
            self._change_position(tag)
            self.signal_change_position.emit(tag)
            return 0

    def set_zposition(self, tag, x=None, y=None, z=None, a=None):
        """Forwarding the desired new position from the GUI to the scanning device.

        @param string tag: TODO

        @param float x: if defined, changes to postion in x-direction (microns)
        @param float y: if defined, changes to postion in y-direction (microns)
        @param float z: if defined, changes to postion in z-direction (microns)
        @param float a: if defined, changes to postion in a-direction (microns)

        @return int: error code (0:OK, -1:error)
        """
        # Changes the respective value


        if z is not None:
            self._current_z = z

        # Checks if the scanner is still running
        if self.module_state() == 'locked' or self._scanning_device.module_state() == 'locked':
            return -1
        else:
            pos_dict = {}
            # print('changing z pos {0}'.format( self._current_z))
            pos_dict['z'] = self._current_z

            # Move piezo to correct position

            self._scanning_device.scanner_set_position(**pos_dict)
            self.signal_change_position.emit(tag)
            return 0

    def zero_stepper(self):
        self._motor.calibrate()

    def change_motor_position(self,x,y,z):
        self._motor.move_abs({'x': x, 'y': y, 'z': z})


        # motor will not move until it is at this position

        self._current_x = x
        self._current_y = y
        self._current_z = z

        self.superdict['x'] = x
        self.superdict['y'] = y
        self.superdict['z'] = z

    def change_angle(self, angle):
        self._rot.move_abs({'x': angle})

    def get_angle(self):
        return self._rot.get_pos()['x']


    def _change_position(self, tag):
        """ Threaded method to change the hardware position.

        @return int: error code (0:OK, -1:error)
        """

        #Move motor to correct position when in cmd + m motor move mode (slow)
        if tag is 'motor':
            print('Deprecated way to change motor')
        else:
            pos_dict = {}
            pos_dict['z'] = self._current_zp
            pos_dict['x'] = self._current_xp
            pos_dict['y'] = self._current_yp
            self._scanning_device.scanner_set_position(**pos_dict)

        return 0

    def jog(self, x=0,y=0,z=0):
        """ Threaded method to change the hardware position.

        @return int: error code (0:OK, -1:error)
        """

        #Move motor to correct position
        if x is -1:
            self._motor.jog(param_list ={'x'},positive=False)
        if y is -1:
            self._motor.jog(param_list ={'y'}, positive=False)
        if z is -1:
            self._motor.jog(param_list ={'z'}, positive=False)
        if x is 1:
            self._motor.jog(param_list ={'x'}, positive=True)
        if y is 1:
            self._motor.jog(param_list ={'y'}, positive=True)
        if z is 1:
            self._motor.jog(param_list ={'z'}, positive=True)


        return 0

    def change_jog_step(self, param_list =None):
        try:
            self._motor.jog_step(param_list=param_list)
        except AttributeError:
            pass


    def get_piezo_position(self):
        piezodict = self._scanning_device.get_scanner_position()
        self._current_xp = piezodict[0]
        self._current_yp = piezodict[1]
        self._current_zp = piezodict[2]

        return self._scanning_device.get_scanner_position()

    def get_motor_position(self):
        superdict = self._motor.get_pos()
        self.superdict = superdict

        self._current_x = superdict['x']
        self._current_y = superdict['y']
        self._current_z = superdict['z']
        return [float(superdict['x']),float(superdict['y']),float(superdict['z'])]

    def get_position(self):
        """ Get position from scanning device and motor to make global position.

        @return list: with three entries x, y and z denoting the current
                      position in meters
        """

        superdict = self._motor.get_pos()
        piezodict= self._scanning_device.get_scanner_position()
        if piezodict[0] is None:
            piezodict[0] = 0
        if piezodict[1] is None:
            piezodict[1] = 0
        return [float(superdict['x'])+ float(piezodict[0]), float(superdict['y'])+float(piezodict[1]), float(superdict['z'])+float(piezodict[2])]

    def get_scanner_axes(self):
        """ Get axes from scanning device.
          @return list(str): names of scanner axes
        """
        return self._scanning_device.get_scanner_axes()

    def get_scanner_count_channels(self):
        """ Get lis of counting channels from scanning device.
          @return list(str): names of counter channels
        """
        return self._scanning_device.get_scanner_count_channels()

    def set_super_pixel(self, size = None, overlap = None):

        if size is not None:
            self.spx_size = size

        if overlap is not None:
            self.spx_overlap = overlap

    def start_super_scan(self, tag=None):


        if self.module_state.current is not 'locked':
            self.module_state.lock()
        else:
            self.log.error('Module is locked')

        print('Start scanner')
        if self._scanning_device.module_state.current is not 'locked':
            self._scanning_device.module_state.lock()
        else:
            self.log.error('Module is locked')

        spxx = self.scan_x_range[1]-self.scan_x_range[0]
        spxy = self.scan_y_range[1]-self.scan_y_range[0]

        #grid of motor points to drive to

        incx = spxx * (1-self.spx_overlap)
        incy = spxy * (1-self.spx_overlap)

        #print('scan increment {0}, {1} with range {2}'.format(incx, incy, self.stitch_range))

        if tag is 'single':
            self.singlemode = True
            xvals = [self.superdict['x']]
            yvals = [self.superdict['y']]
        else:
            self.singlemode = False
            # the way it works is the grid of values are calculate around the current position reported by the motor
            xvals = np.arange(self.superdict['x'], self.superdict['x']+self.stitch_range-incx, incx)
            yvals = np.arange(self.superdict['y'], self.superdict['y']+self.stitch_range-incy, incy)

            # What happens if only a single scan is wanted?
            if self.stitch_range < 20e-6:
                xvals = [self.superdict['x']]
                yvals = [self.superdict['y']]


        #print('xvals to scan {0}'.format(xvals))
        #print('yvals to scan {0}'.format(yvals))

        self.spx_grid = []

        even = 0

        #Reversing coordinate list of every row means motor traverses optimally in zig zag
        for x in xvals:
            if even is 0:
                for y in yvals:
                    #print('( {0}, {1} )'.format(x,y))
                    self.spx_grid.append((x, y))
            else:
                for y in reversed(yvals):
                    #print('( {0}, {1} )'.format(x,y))
                    self.spx_grid.append((x, y))
            even = (even +1) %2

        print('Locations to go to {0}'.format(self.spx_grid))

        # Restart counter
        self._spx_counter = 0

        self.stopsuperRequested = False
        self.stopRequested = False


        if self.initialize_image() < 0:
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            return -1


        # Emit start scan
        self.signal_spx_next.emit()



    def _scan_super_px(self):


        if self.singlemode is False:

            if self.dynamic_z is True:
                self.dynamic_updating_status = 1
                if self.update_z_dynamic(tag='dynamic') > -1:
                    self.dynamic_updating_status = 0
                    self.log.info('Piezo updated to {0}'.format(self._current_zp))

            self._current_x = self.spx_grid[self._spx_counter][0]
            self._current_y = self.spx_grid[self._spx_counter][1]

            #print('updated current location to {0}, {1}'.format(self._current_x, self._current_y,self._current_z))


            #self.log.info('Scanning super px and moving motor to {0:.4f}, {1:.4f} mm'.format(1e3*self.spx_grid[self._spx_counter][0],1e3*self.spx_grid[self._spx_counter][1]))

            #self._current_z = self._calc_dz(self._current_x, self._current_y)  +  self._current_z

           #self.log.info('IF tilt was correct, would move z by {0:.4f} mm'.format(self._calc_dz(self._current_x, self._current_y)))
            # Function will not return until at this position

            self._motor.move_abs({'x': self.spx_grid[self._spx_counter][0],'y': self.spx_grid[self._spx_counter][1], 'z':self._current_z})



        self.signal_start_scanning.emit('scanner')


    def after_piezo_scan(self):

        self._scan_counter = 0


        # find NVs
        # get mapped data as an input


        # update POIs loc

        # stop scanning when last line scan was performed and makes scan not continuable
        if self._spx_counter >= len(self.spx_grid) : # changed from len - 1 JAS 10/09/18
            print('ending scan')

            if not self.permanent_scan:
                self.stopsuperRequested = True
                if self._zscan:
                    self._zscan_continuable = False
                else:
                    self._xyscan_continuable = False
            self._spx_counter = 0

            self._scanning_device.module_state.unlock()
            self._scanning_device.scanner_lock = False
            self.module_state.unlock()

        else:

            print('next')
            #update motor position


            #self.signal_spx_next.connect(self._scan_super_px, QtCore.Qt.QueuedConnection)
            self.signal_spx_next.emit()


    def stop_scanning_spx(self):

        # putting this line back in attempt to fix bug of restarting scan mid line - JS 07/06

        self._scan_counter = 0
        self.stopsuperRequested = True


        #self.signal_spx_next.emit()

    def stop_scanning_3d(self):
        self.stop3Drequested = True


    def _scan_line(self):
        """scanning an image in either depth or xy

        """
        # stops scanning
        if self.stopRequested:
            with self.threadlock:
                #self.signal_xy_image_updated.emit()
                #self.module_state.unlock()
                #print('stopping piezo scan')
                self._spx_counter += 1
                self.signal_stop_scanning.emit()
                #self._scanning_device.module_state.unlock()
                return

        if self.stopsuperRequested:
            with self.threadlock:
                #print('Stopping completely')
                self.stopsuperRequested = False

                self.module_state.unlock()

                # kill scanner locks so do not need to unlock
                #self._scanning_device.module_state.unlock()

                self.kill_scanner()

                #self.signal_xy_image_updated.emit() # 29/ 08, minimise refresh image calls
                self.signal_depth_image_updated.emit()
                if self._zscan:
                    self._depth_line_pos = self._scan_counter
                else:
                    self._xy_line_pos = self._scan_counter
                # add new history entry
                new_history = ConfocalHistoryEntry(self)
                new_history.snapshot(self)
                self.history.append(new_history)
                if len(self.history) > self.max_history_length:
                    self.history.pop(0)
                self.history_index = len(self.history) - 1
                return

        s_ch = len(self.get_scanner_count_channels())


        try:

            # scan the line in the scan

            new_line = self._scanning_device.scan_line()

            if self._scanning_device.fl_mode is True:
                self.xy_image[self.current_pointer]['fl'][-(self._scan_counter + 1), :, :] = self._scanning_device.fl_hist
            #     try:
            #         self.xy_image_fl[-(self._scan_counter + 1), :, :] = self._scanning_device.fl_hist
            #     except AttributeError:
            #         pass

            if np.any(new_line == -1):
                self.stopsuperRequested = True
                self.signal_scan_lines_next.emit()
                return

            # update image with counts from the line we just scanned

            if self.stopsuperRequested is False: # Change 20/01/2019 to stop first line being changed
                self.xy_image[self.current_pointer]['image'][-(self._scan_counter+1), :, 3:3 + s_ch] = new_line



            self.signal_xy_image_updated.emit()

            # next line in scan
            self._scan_counter += 1

            # stop scanning when last line scan was performed and makes scan not continuable
            if self._scan_counter >= np.size(self._image_vert_axis):
                self._scan_counter = 0
                if not self.permanent_scan:
                    self.stopRequested = True
                    if self._zscan:
                        self._zscan_continuable = False
                    else:
                        self._xyscan_continuable = False

            self.signal_scan_lines_next.emit()
        except:
            self.log.exception('The scan went wrong, killing the scanner.')
            self.stop_scanning()
            self.signal_scan_lines_next.emit()



    def start_zscan(self, zrange = [-2e-5, 0],res=130,inttime=8):

        self._scanning_device.set_up_1Dscan(range=zrange, res=res,inttime=inttime,channel=2)

    def start_xscan(self, zrange=[-2e-5, 0], res=130,inttime=8):

        self._scanning_device.set_up_1Dscan(range=zrange, res=res,inttime=inttime,channel=0)

    def start_yscan(self, zrange=[-2e-5, 0], res=130,inttime=8):

        self._scanning_device.set_up_1Dscan(range=zrange, res=res,inttime=inttime,channel=1)

    def z_line(self):

        vals = self._scanning_device.scan_line_z()

        if vals[0] is -1:
            self._scanning_device.close_scanner()

        return vals

    def kill_z(self):
        self._scanning_device.close_scanner()


    def scan_3d_start(self):

        #set up scanner
        vals = self._scanning_device.set_up_3dscan(delta_res = self.psf_deltares, xydelta=self.psf_deltaxy,
                                                   zdelta=self.psf_deltaz, xyres = self.psf_xyres, zres = self.psf_zres)

        if vals is -1 :
            return -1

        self.yline_3d = 0
        self.zline_3d = 0

        self.stop3Drequested = False

        self.psf_data = np.ones((self.psf_xyres, self.psf_xyres, self.psf_zres))



        if self._scanning_device.module_state.current is not 'locked':
            self._scanning_device.module_state.lock()
        else:
            self.log.error('Module is locked')

        # lock module
        self.module_state.lock()

        #call next line
        self.signal_3d_next.emit()



    def next_3d_line(self):

        # if 3D scan quit command, send to arduino, finish up
        if self.stop3Drequested:
            with self.threadlock:
                print('Stopping completely')
                self.stop3Drequested = False

                self.module_state.unlock()

                #self._scanning_device

                self.kill_scanner()
                return

        #else get line
        vals = self._scanning_device.scan_line()

        self.psf_data[:,self.yline_3d,self.zline_3d] = vals.flatten()

        #increment y line count, or z line if end of y line
        self.yline_3d = self.yline_3d + 1

        if self.yline_3d > self.psf_xyres-1:
            self.yline_3d = 0
            self.zline_3d = self.zline_3d + 1

        #if finished, request stop
        if self.zline_3d > self.psf_zres-1:
            self.stop3Drequested = True

        #signal next line
        self.signal_3d_next.emit()

        #print('finder psf')
        #print(self.yline_3d,self.zline_3d)
        #print(self.psf_data)

    #def update_psf_xy_res

    #def update_psf_z_res

    def update_z_dynamic(self,tag = None):
        # min_val = -10e-6
        # max_val = 10e-6
        #
        # key = next(reversed(self.xy_image))
        # record = self.xy_image[key]
        # i_max = np.argmax(record['image'][:, :, 3])
        # if i_max == 0:
        #     self._current_xp = 0
        #     self._current_yp = 0
        # else:
        #     x = record['image'][:, :, 0].flatten()[i_max]
        #     y = record['image'][:, :, 1].flatten()[i_max]
        #
        #     # print(x,y)
        #     xmot = self.superdict['x']
        #     ymot = self.superdict['y']
        #     self._current_xp = np.clip(x - xmot, min_val, max_val)
        #     self._current_yp = np.clip(y - ymot, min_val, max_val)
        #
        # self._change_position(tag='update_xy')
        #
        # # Maybe have this on certain tags JS 040820
        # # self.signal_change_position.emit('update_xy')
        #
        # # print(self._current_xp, self._current_yp)
        #
        # self.start_zscan(zrange=[min_val, max_val], res=self.res)
        #
        # data = (np.asarray(self.z_line()).T)[0, 0:]
        #
        # x_axis = np.linspace(min_val, max_val, num=self.res)
        #
        # try:
        #     result = self._fit_logic.make_gaussianlinearoffset_fit(x_axis=x_axis, data=data,
        #                                                            estimator=self._fit_logic.estimate_gaussianlinearoffset_peak)
        #     peak = result.best_values['center']
        #
        # except UnboundLocalError:
        #     self.log.error('Is Waterloo box on?')
        #
        # self._current_zp = peak
        #
        # self._change_position(tag='update_z')

        #self.update_z()
        #
        # pos_dict = {}
        # pos_dict['x'] = self._current_xp
        # pos_dict['y'] = self._current_yp
        # pos_dict['z'] = self._current_zp
        # self._scanning_device.scanner_set_position(**pos_dict)
        #self._change_position(tag='update_xy')
        return 0



    def update_z(self, tag=None):

        min_val = -10e-6
        max_val = 10e-6

        key = next(reversed(self.xy_image))
        record = self.xy_image[key]
        i_max = np.argmax(record['image'][:, :, 3])
        if i_max ==0:
            self._current_xp = 0
            self._current_yp = 0
        else:
            x = record['image'][:, :, 0].flatten()[i_max]
            y = record['image'][:, :, 1].flatten()[i_max]


            #print(x,y)
            xmot = self.superdict['x']
            ymot = self.superdict['y']
            self._current_xp = np.clip(x - xmot, min_val, max_val)
            self._current_yp = np.clip(y - ymot, min_val, max_val)

        self._change_position(tag = 'update_xy')


        #Maybe have this on certain tags JS 040820
        #self.signal_change_position.emit('update_xy')

        #print(self._current_xp, self._current_yp)

        self.start_zscan(zrange=[min_val, max_val], res=self.res)

        data = (np.asarray(self.z_line()).T)[0, 0:]

        x_axis = np.linspace(min_val, max_val, num=self.res)

        try:
            result = self._fit_logic.make_gaussianlinearoffset_fit(x_axis=x_axis, data=data,
                                                                   estimator=self._fit_logic.estimate_gaussianlinearoffset_peak)
            peak = result.best_values['center']

        except UnboundLocalError:
            self.log.error('Is Waterloo box on?')


        self._current_zp = peak

        self._change_position(tag = 'update_z')

        self.signal_change_position.emit('update_z')



    def add_poi(self,x,y, label = None):

        #Add poi from location to dictionary, POI name will be in format DDMMYY_HHMMSS


        d = datetime.now()

        dict = {}

        if label is not None:
            dict['label'] = d.strftime("%H%M%S_%d%m%y") +  "_" + str(label)
        else:
            dict['label'] = d.strftime("%H%M%S_%d%m%y")

        #Initial population: polarisation, g(2), FL
        dict['τ_F'] = 0.0
        dict['g(2)'] = '1'
        dict['(ϕ,θ)'] = '(0,0)'
        dict['(x,y)'] = (x,y)
        dict['rating' ] = 0
        dict['Marker'] = 'AA'

        self.pois[dict['label']] = dict

        self.save_POIFile()


    def del_poi(self):

        print('deleting')
        #self.current_poi['name']


        self.save_POIFile()


    def _loadPOIFile(self):
        """

          @return sting: path to poi file
        """
        path = self.getMainDir()
        # we first look for config/load.cfg which can point to another
        # config file using the "configfile" key
        loadConfigFile = os.path.join(path, 'POI', 'load.cfg')
        if os.path.isfile(loadConfigFile):
            self.log.info('load.cfg config file found at {0}'.format(
                loadConfigFile))
            try:
                confDict = config.load(loadConfigFile)
                if ('configfile' in confDict
                    and isinstance(confDict['configfile'], str)):
                    # check if this config file is existing
                    # try relative filenames
                    configFile = os.path.join(path, 'config',
                                              confDict['configfile'])
                    if os.path.isfile(configFile):
                        return configFile
                    # try absolute filename or relative to pwd
                    if os.path.isfile(confDict['configfile']):
                        return confDict['configfile']
                    else:
                        self.log.error('Couldn\'t find POI file '
                                        ': {0}'.format(
                            confDict['configfile']))
            except Exception:
                self.log.exception('Error while handling POI.')

        raise Exception('Could not find POI file.')

    def save_POIFile(self):

        """Write a file into the currently used config directory.

          @param dict data: dictionary to write into file
          @param string fileName: path for filr to be written
        """

        fileName = self.pois[list(self.pois)[0]]['label']

        with self.lock:
            # dirName = os.path.dirname(fileName)
            # if not os.path.exists(dirName):
            #     os.makedirs(dirName)
            config.save('C:/Data/POIs/'+fileName, self.pois)


    def save_xy_data(self, colorscale_range=None, percentile_range=None):
        """ Save the current confocal xy data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.

        A figure is also saved.

        @param: list colorscale_range (optional) The range [min, max] of the display colour scale (for the figure)

        @param: list percentile_range (optional) The percentile range [min, max] of the color scale
        """
        filepath = self._save_logic.get_path_for_module('Confocal')
        timestamp = datetime.now()
        # Prepare the metadata parameters (common to both saved files):
        parameters = OrderedDict()

        # parameters['X image min (m)'] = self.image_x_range[0]
        # parameters['X image max (m)'] = self.image_x_range[1]
        # parameters['X image range (m)'] = self.image_x_range[1] - self.image_x_range[0]
        #
        # parameters['Y image min'] = self.image_y_range[0]
        # parameters['Y image max'] = self.image_y_range[1]
        # parameters['Y image range'] = self.image_y_range[1] - self.image_y_range[0]
        #
        # parameters['XY resolution (samples per range)'] = self.xy_resolution
        # parameters['XY Image at z position (m)'] = self._current_z
        #
        parameters['Clock frequency of scanner (Hz)'] = self._clock_frequency
        parameters['Return Slowness (Steps during retrace line)'] = self.return_slowness

        # if self.singlemode is False:
        #     print(self.xy_image)
        #     print()


        # Update... break into smaller chunks? JAS 04/08/2020


        data = OrderedDict()

        for key, record in self.xy_image.items():

            if key is 'History':
                continue

            try:
                # Can't do nested dicts JS 22/02/20
                #print(key)
                # Save x, y z, res
                parameters['x'] = record['x']
                parameters['y'] = record['y']
                parameters['z'] = record['z']

                parameters['resolution'] = record['res']

                # Save polarisation
                parameters['polarisation'] = record['pol']

                # Save image and fl
                data['img'] = record['image'].flatten()

                if self._scanning_device.fl_mode is True:
                    data['fl'] = record['fl'].flatten()

                data['x-piezo'] = record['image'][:, :, 0].flatten()
                data['y-piezo'] = record['image'][:, :, 1].flatten()
                data['z-piezo'] = record['image'][:, :, 2].flatten()

                # Save the raw data to file
                filelabel = 'confocal_xy_data_{0}'.format(key)

                dir_path = os.path.join(filepath, timestamp.strftime('%H%M-%S'))

                if not os.path.exists(dir_path):
                    os.makedirs(dir_path)

                self._save_logic.save_data(data,
                                           filepath=dir_path,
                                           timestamp=timestamp,
                                           parameters=parameters,
                                           filelabel=filelabel,
                                           fmt='%.6e',
                                           delimiter='\t')

            except KeyError as inst:
                self.log.error('{0} has no {1}'.format(key,inst))



        self.log.debug('Confocal Image saved.')
        self.signal_xy_data_saved.emit()
        return

    def save_last_xy_data(self, colorscale_range=None, percentile_range=None):
        """ Save the current confocal xy data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.

        A figure is also saved.

        @param: list colorscale_range (optional) The range [min, max] of the display colour scale (for the figure)

        @param: list percentile_range (optional) The percentile range [min, max] of the color scale
        """
        filepath = self._save_logic.get_path_for_module('Confocal')
        timestamp = datetime.now()
        # Prepare the metadata parameters (common to both saved files):
        parameters = OrderedDict()

        # parameters['X image min (m)'] = self.image_x_range[0]
        # parameters['X image max (m)'] = self.image_x_range[1]
        # parameters['X image range (m)'] = self.image_x_range[1] - self.image_x_range[0]
        #
        # parameters['Y image min'] = self.image_y_range[0]
        # parameters['Y image max'] = self.image_y_range[1]
        # parameters['Y image range'] = self.image_y_range[1] - self.image_y_range[0]
        #
        # parameters['XY resolution (samples per range)'] = self.xy_resolution
        # parameters['XY Image at z position (m)'] = self._current_z
        #
        parameters['Clock frequency of scanner (Hz)'] = self._clock_frequency
        parameters['Return Slowness (Steps during retrace line)'] = self.return_slowness

        data = OrderedDict()

        key = next(reversed(self.xy_image))

        record = self.xy_image[key]

        try:
            # Can't do nested dicts JS 22/02/20

            # Save x, y z, res
            parameters['{0}_x'.format(key)] = record['x']
            parameters['{0}_y'.format(key)] = record['y']
            parameters['{0}_z'.format(key)] = record['z']

            parameters['{0}_res'.format(key)] = record['res']

            # Save polarisation
            parameters['{0}_pol'.format(key)] = record['pol']

            # Save image and fl
            data['{0}_img'.format(key)] = record['image'].flatten()
            if self._scanning_device.fl_mode is True:
                data['{0}_fl'.format(key)] = record['fl'].flatten()

            data['{0}_xpiezo'.format(key)] = record['image'][:, :, 0].flatten()
            data['{0}_ypiezo'.format(key)] = record['image'][:, :, 1].flatten()
            data['{0}_zpiezo'.format(key)] = record['image'][:, :, 2].flatten()

        except KeyError as inst:
            self.log.error('{0} has no {1}'.format(key, inst))

        # Save the raw data to file
        filelabel = 'last_confocal_xy_data' + key

        dir_path = os.path.join(filepath,timestamp)

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        self._save_logic.save_data(data,
                                   filepath=dir_path,
                                   timestamp=timestamp,
                                   parameters=parameters,
                                   filelabel=filelabel,
                                   fmt='%.6e',
                                   delimiter='\t')

        self.log.debug('Confocal Image saved.')
        self.signal_xy_data_saved.emit()
        return

    def save_depth_data(self, colorscale_range=None, percentile_range=None):
        """ Save the current confocal depth data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.
        """
        filepath = self._save_logic.get_path_for_module('Confocal')
        timestamp = datetime.datetime.now()
        # Prepare the metadata parameters (common to both saved files):
        parameters = OrderedDict()

        # TODO: This needs to check whether the scan was XZ or YZ direction
        parameters['X image min (m)'] = self.image_x_range[0]
        parameters['X image max (m)'] = self.image_x_range[1]
        parameters['X image range (m)'] = self.image_x_range[1] - self.image_x_range[0]

        parameters['Z image min'] = self.image_z_range[0]
        parameters['Z image max'] = self.image_z_range[1]
        parameters['Z image range'] = self.image_z_range[1] - self.image_z_range[0]

        parameters['XY resolution (samples per range)'] = self.xy_resolution
        parameters['Z resolution (samples per range)'] = self.z_resolution
        parameters['Depth Image at y position (m)'] = self._current_y

        parameters['Clock frequency of scanner (Hz)'] = self._clock_frequency
        parameters['Return Slowness (Steps during retrace line)'] = self.return_slowness

        if self.depth_img_is_xz:
            horizontal_range = [self.image_x_range[0], self.image_x_range[1]]
            axes = ['X', 'Z']
            crosshair_pos = [self.get_position()[0], self.get_position()[2]]
        else:
            horizontal_range = [self.image_y_range[0], self.image_y_range[1]]
            axes = ['Y', 'Z']
            crosshair_pos = [self.get_position()[1], self.get_position()[2]]

        image_extent = [horizontal_range[0],
                        horizontal_range[1],
                        self.image_z_range[0],
                        self.image_z_range[1]]

        figs = {ch: self.draw_figure(data=self.depth_image[:, :, 3 + n],
                                     image_extent=image_extent,
                                     scan_axis=axes,
                                     cbar_range=colorscale_range,
                                     percentile_range=percentile_range,
                                     crosshair_pos=crosshair_pos)
                for n, ch in enumerate(self.get_scanner_count_channels())}

        # Save the image data and figure
        for n, ch in enumerate(self.get_scanner_count_channels()):
            # data for the text-array "image":
            image_data = OrderedDict()
            image_data['Confocal pure depth scan image data without axis.\n'
                'The upper left entry represents the signal at the upper left pixel position.\n'
                'A pixel-line in the image corresponds to a row in '
                'of entries where the Signal is in counts/s:'] = self.depth_image[:, :, 3 + n]

            filelabel = 'confocal_depth_image_{0}'.format(ch.replace('/', ''))
            self._save_logic.save_data(image_data,
                                       filepath=filepath,
                                       timestamp=timestamp,
                                       parameters=parameters,
                                       filelabel=filelabel,
                                       fmt='%.6e',
                                       delimiter='\t',
                                       plotfig=figs[ch])

        # prepare the full raw data in an OrderedDict:
        data = OrderedDict()
        data['x position (m)'] = self.depth_image[:, :, 0].flatten()
        data['y position (m)'] = self.depth_image[:, :, 1].flatten()
        data['z position (m)'] = self.depth_image[:, :, 2].flatten()

        for n, ch in enumerate(self.get_scanner_count_channels()):
            data['count rate {0} (Hz)'.format(ch)] = self.depth_image[:, :, 3 + n].flatten()

        # Save the raw data to file
        filelabel = 'confocal_depth_data'
        self._save_logic.save_data(data,
                                   filepath=filepath,
                                   timestamp=timestamp,
                                   parameters=parameters,
                                   filelabel=filelabel,
                                   fmt='%.6e',
                                   delimiter='\t')

        self.log.debug('Confocal Image saved.')
        self.signal_depth_data_saved.emit()
        return

    def draw_figure(self, data, image_extent, scan_axis=None, cbar_range=None, percentile_range=None,  crosshair_pos=None):
        """ Create a 2-D color map figure of the scan image.

        @param: array data: The NxM array of count values from a scan with NxM pixels.

        @param: list image_extent: The scan range in the form [hor_min, hor_max, ver_min, ver_max]

        @param: list axes: Names of the horizontal and vertical axes in the image

        @param: list cbar_range: (optional) [color_scale_min, color_scale_max].  If not supplied then a default of
                                 data_min to data_max will be used.

        @param: list percentile_range: (optional) Percentile range of the chosen cbar_range.

        @param: list crosshair_pos: (optional) crosshair position as [hor, vert] in the chosen image axes.

        @return: fig fig: a matplotlib figure object to be saved to file.
        """
        if scan_axis is None:
            scan_axis = ['X', 'Y']

        # If no colorbar range was given, take full range of data
        if cbar_range is None:
            cbar_range = [np.min(data), np.max(data)]

        # Scale color values using SI prefix
        prefix = ['', 'k', 'M', 'G']
        prefix_count = 0
        image_data = data
        print('printing data to save')
        print(data)

        draw_cb_range = np.array(cbar_range)
        image_dimension = image_extent.copy()

        while draw_cb_range[1] > 1000:
            image_data = image_data/1000
            draw_cb_range = draw_cb_range/1000
            prefix_count = prefix_count + 1

        c_prefix = prefix[prefix_count]


        # Scale axes values using SI prefix
        axes_prefix = ['', 'm', r'$\mathrm{\mu}$', 'n']
        x_prefix_count = 0
        y_prefix_count = 0

        while np.abs(image_dimension[1]-image_dimension[0]) < 1:
            image_dimension[0] = image_dimension[0] * 1000.
            image_dimension[1] = image_dimension[1] * 1000.
            x_prefix_count = x_prefix_count + 1

        while np.abs(image_dimension[3] - image_dimension[2]) < 1:
            image_dimension[2] = image_dimension[2] * 1000.
            image_dimension[3] = image_dimension[3] * 1000.
            y_prefix_count = y_prefix_count + 1

        x_prefix = axes_prefix[x_prefix_count]
        y_prefix = axes_prefix[y_prefix_count]

        # Use qudi style
        plt.style.use(self._save_logic.mpl_qd_style)

        # Create figure
        fig, ax = plt.subplots()

        # Create image plot
        cfimage = ax.imshow(image_data,
                            cmap=plt.get_cmap('inferno'), # reference the right place in qd
                            origin="lower",
                            vmin=draw_cb_range[0],
                            vmax=draw_cb_range[1],
                            interpolation='none',
                            extent=image_dimension
                            )

        ax.set_aspect(1)
        ax.set_xlabel(scan_axis[0] + ' position (' + x_prefix + 'm)')
        ax.set_ylabel(scan_axis[1] + ' position (' + y_prefix + 'm)')
        ax.spines['bottom'].set_position(('outward', 10))
        ax.spines['left'].set_position(('outward', 10))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

        # draw the crosshair position if defined
        if crosshair_pos is not None:
            trans_xmark = mpl.transforms.blended_transform_factory(
                ax.transData,
                ax.transAxes)

            trans_ymark = mpl.transforms.blended_transform_factory(
                ax.transAxes,
                ax.transData)

            ax.annotate('', xy=(crosshair_pos[0]*np.power(1000,x_prefix_count), 0),
                        xytext=(crosshair_pos[0]*np.power(1000,x_prefix_count), -0.01), xycoords=trans_xmark,
                        arrowprops=dict(facecolor='#17becf', shrink=0.05),
                        )

            ax.annotate('', xy=(0, crosshair_pos[1]*np.power(1000,y_prefix_count)),
                        xytext=(-0.01, crosshair_pos[1]*np.power(1000,y_prefix_count)), xycoords=trans_ymark,
                        arrowprops=dict(facecolor='#17becf', shrink=0.05),
                        )

        # Draw the colorbar
        cbar = plt.colorbar(cfimage, shrink=0.8)#, fraction=0.046, pad=0.08, shrink=0.75)
        cbar.set_label('Fluorescence (' + c_prefix + 'c/s)')

        # remove ticks from colorbar for cleaner image
        cbar.ax.tick_params(which=u'both', length=0)

        # If we have percentile information, draw that to the figure
        if percentile_range is not None:
            cbar.ax.annotate(str(percentile_range[0]),
                             xy=(-0.3, 0.0),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
            cbar.ax.annotate(str(percentile_range[1]),
                             xy=(-0.3, 1.0),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
            cbar.ax.annotate('(percentile)',
                             xy=(-0.3, 0.5),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
        self.signal_draw_figure_completed.emit()
        return fig

    ##################################### Tilt correction ########################################

    @QtCore.Slot()
    def set_tilt_point1(self):
        """ Gets the first reference point for tilt correction."""
        self.point1 = np.array(self._scanning_device.get_scanner_position()[:3]) + np.array([self._current_x,self._current_y, self._current_z])
        self.signal_tilt_correction_update.emit()

    @QtCore.Slot()
    def set_tilt_point2(self):
        """ Gets the second reference point for tilt correction."""
        self.point2 = np.array(self._scanning_device.get_scanner_position()[:3]) + np.array([self._current_x,self._current_y, self._current_z])
        self.signal_tilt_correction_update.emit()

    @QtCore.Slot()
    def set_tilt_point3(self):
        """Gets the third reference point for tilt correction."""
        self.point3 = np.array(self._scanning_device.get_scanner_position()[:3]) + np.array([self._current_x,self._current_y, self._current_z])
        self.signal_tilt_correction_update.emit()

    @QtCore.Slot()
    def calc_tilt_correction(self):
        """ Calculates the values for the tilt correction. """
        a = self.point2 - self.point1
        b = self.point3 - self.point1
        n = np.cross(a, b)
        self.tilt_variable_ax = n[0] / n[2]
        self.tilt_variable_ay = n[1] / n[2]

    @QtCore.Slot(bool)
    def set_tilt_correction(self, enabled):
        """ Set tilt correction in tilt interfuse.

            @param bool enabled: whether we want to use tilt correction
        """
        self.tilt_reference_x = self._scanning_device.get_scanner_position()[0]
        self.tilt_reference_y = self._scanning_device.get_scanner_position()[1]
        self.signal_tilt_correction_active.emit(enabled)


    def _calc_dz(self, x, y):
        """Calculates the change in z for given tilt correction."""

        dz = -(
            (x - self.tilt_reference_x) * self.tilt_variable_ax
            + (y - self.tilt_reference_y) * self.tilt_variable_ay
        )

        #print('Tilt correction: ',dz)
        return dz

    def history_forward(self):
        """ Move forward in confocal image history.
        """
        if self.history_index < len(self.history) - 1:
            self.history_index += 1
            self.history[self.history_index].restore(self)
            self.signal_xy_image_updated.emit()
            self.signal_depth_image_updated.emit()
            self.signal_tilt_correction_update.emit()
            self.signal_tilt_correction_active.emit(self._scanning_device.tiltcorrection)
            self._change_position('history')
            self.signal_change_position.emit('history')
            self.signal_history_event.emit()

    def history_back(self):
        """ Move backwards in confocal image history.
        """
        if self.history_index > 0:
            self.history_index -= 1
            self.history[self.history_index].restore(self)
            self.signal_xy_image_updated.emit()
            self.signal_depth_image_updated.emit()
            self.signal_tilt_correction_update.emit()
            self.signal_tilt_correction_active.emit(self._scanning_device.tiltcorrection)
            self._change_position('history')
            self.signal_change_position.emit('history')
            self.signal_history_event.emit()
