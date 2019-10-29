



# On initialisation


# Read x pos, y pos, zpos ( raw and mm )

# Velocity

# Jog step

# Hardware limits

from qtpy import QtCore


from collections import OrderedDict

from core import config

import os


from logic.generic_logic import GenericLogic
from core.util.mutex import Mutex
from core.module import Connector, ConfigOption, StatusVar


class StepperLogic(GenericLogic):
    """
    This is the Logic class for confocal scanning.
    """
    _modclass = 'confocallogic'
    _modtype = 'logic'

    # declare connectors
    savelogic = Connector(interface='SaveLogic')
    motor = Connector(interface = 'MotorInterface')

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

        self._save_logic = self.get_connector('savelogic')
        self._motor = self.get_connector('motor')

        self.lock = Mutex(recursive=True)

        # Reads in the maximal scanning range. The unit of that scan range is micrometer!
        # Reads in the maximal scanning range. The unit of that scan range is meters!
        self.x_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[0]
        self.y_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[1]
        self.z_range = [0e-6, 6e-3]  # confocal._motor.get_position_range()[2]

        self.scan_x_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[0]
        self.scan_y_range = [-10e-6, 10e-6]  # confocal._motor.get_position_range()[1]



        self.stitch_range = 160e-6

        self.stopsuperRequested = False


        self.motor_range = [0e-3, 6e-3]  # confocal._motor.get_position_range()[1]

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

        self.get_motor_position()


        self.pois = {}



    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """

        if self.module_state.current is 'locked':
            self.module_state.unlock()


        return 0

    def zero_stepper(self):
        self._motor.calibrate()

    def change_jog_step(self, param_list =None):
        try:
            self._motor.jog_step(param_list=param_list)
        except AttributeError:
            pass

    def get_motor_position(self):
        superdict = self._motor.get_pos()
        self.superdict = superdict
        self._current_x = superdict['x']
        self._current_y = superdict['y']
        self._current_z = superdict['z']
        return [float(superdict['x']),float(superdict['y']),float(superdict['z'])]

    def get_motor_position_raw(self):
        pos = self._motor.get_pos_raw()
        return [int(pos['x']),int(pos['y']),int(pos['z'])]

    def get_motor_velocity(self):
        pos = self._motor.get_velocity()
        return [float(pos['x']),float(pos['y']),float(pos['z'])]

    def get_motor_params(self):
        pos = self._motor.get_params()
        return [pos['x'], pos['y'], pos['z']]


    def change_motor_position(self,x,y,z):
        self._motor.move_abs({'x': x, 'y': y, 'z': z})


        # motor will not move until it is at this position

        self._current_x = x
        self._current_y = y
        self._current_z = z

    def home_x(self):
        self._motor.home({'x'})

    def home_y(self):
        self._motor.home({'y'})

    def home_z(self):
        self._motor.home({'z'})


    def reset_x(self):
        self._motor.reset({'x'})

    def reset_y(self):
        self._motor.reset({'y'})

    def reset_z(self):
        self._motor.reset({'z'})


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

    def switch_hardware(self, to_on=False):
        """ Switches the Hardware off or on.

        @param to_on: True switches on, False switched off

        @return int: error code (0:OK, -1:error)
        """
        if to_on:
            return 0 #self._scanning_device.activation()
        else:
            return 0 #self._scanning_device.reset_hardware()