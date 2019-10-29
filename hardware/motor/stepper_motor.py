import math
import numpy as np
from datetime import datetime as dt
from time import time, sleep
from random import random
from scipy.optimize import minimize

from collections import OrderedDict
import serial

from numpy import arange

import re

import visa
import signal

from core.module import Base, ConfigOption
from interface.motor_interface import MotorInterface


class StepperStage(Base, MotorInterface):
    """unstable: Christoph Müller, Simon Schmitt
    This is the Interface class to define the controls for the Arduino stepper motor stage
    """
    # _modclass = 'MotorStageNanomax'
    # _modtype = 'hardware'

    _com_port_nano_xyz = ConfigOption('com_port_nano_xyz', 'COM6')
    _nano_xyz_baud_rate = ConfigOption('nano_xyz_baud_rate', 57600)
    #_nano_xyz_timeout = ConfigOption('nano_xyz_timeout', 1000)
    _nano_xyz_term_char = ConfigOption('nano_xyz_term_char', '\r\n')
    _first_axis_label = ConfigOption('nano_first_axis_label', 'x')
    _second_axis_label = ConfigOption('nano_second_axis_label', 'y')
    _first_axis_ID = ConfigOption('nano_first_axis_ID', '0')
    _second_axis_ID = ConfigOption('nano_second_axis_ID', '1')

    constraints = {}
    axis0 = {}
    axis1 = {}
    #axis2 = {}

    _min_first = ConfigOption('nano_first_min', -1e-3)  # Values in m
    _max_first = ConfigOption('nano_first_max', 1e-3)
    _min_second = ConfigOption('nano_second_min', -1e-3)
    _max_second = ConfigOption('nano_second_max', 1e-3)
   # _min_third = ConfigOption('nano_third_min', -10e-6)
    #_max_third = ConfigOption('nano_third_max', 10e-6)

    step_first_axis = ConfigOption('nano_first_axis_step', 1e-7)
    step_second_axis = ConfigOption('nano_second_axis_step', 1e-7)
    #step_third_axis = ConfigOption('nano_third_axis_step', 1e-7)

    _vel_min_first = ConfigOption('vel_first_min', 1e-5)
    _vel_max_first = ConfigOption('vel_first_max', 5e-2)
    _vel_min_second = ConfigOption('vel_second_min', 1e-5)
    _vel_max_second = ConfigOption('vel_second_max', 5e-2)
   # _vel_min_third = ConfigOption('vel_third_min', 1e-5)
    #_vel_max_third = ConfigOption('vel_third_max', 5e-2)

    _vel_step_first = ConfigOption('vel_first_axis_step', 1e-5)
    _vel_step_second = ConfigOption('vel_second_axis_step', 1e-5)
    #_vel_step_third = ConfigOption('vel_third_axis_step', 1e-5)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        @return: error code
        """

        self.stepper = StepperController(port = self._com_port_nano_xyz)

        self.vel = {}
        self.vel['x-axis'] = 1e-5
        self.vel['y-axis'] = 1e-5
       # self.vel['z-axis'] = 1e-5
        return 0

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        @return: error code
        """

        self.stepper.close_connection()

        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the sequence generation and GUI

        Provides all the constraints for the xyz stage  and rot stage (like total
        movement, velocity, ...)
        Each constraint is a tuple of the form
            (min_value, max_value, stepsize)
        """
        constraints = OrderedDict()

        axis0 = {}
        axis0['label'] = self._first_axis_label
        axis0['ID'] = 'x'
        axis0['unit'] = 'm'  # the SI units
        # axis0['ramp'] = None # a possible list of ramps
        axis0['pos_min'] = self._min_first
        axis0['pos_max'] = self._max_first
        axis0['scan_min'] = -10e-6
        axis0['scan_max'] = 10e-6
        # axis0['pos_step'] = self.step_first_axis
        # axis0['vel_min'] = self._vel_min_first
        # axis0['vel_max'] = self._vel_max_first
        # axis0['vel_step'] = self._vel_step_first
        # axis0['acc_min'] = None
        # axis0['acc_max'] = None
        # axis0['acc_step'] = None
        #
        axis1 = {}
        axis1['label'] = self._second_axis_label
        axis1['ID'] = self._second_axis_ID
        # axis1['unit'] = 'm'        # the SI units
        # axis1['ramp'] = None # a possible list of ramps
        axis1['pos_min'] = self._min_second
        axis1['pos_max'] = self._max_second
        axis1['scan_min'] = -10e-6
        axis1['scan_max'] = 10e-6
        # axis1['pos_step'] = self.step_second_axis
        # axis1['vel_min'] = self._vel_min_second
        # axis1['vel_max'] = self._vel_max_second
        # axis1['vel_step'] = self._vel_step_second
        # axis1['acc_min'] = None
        # axis1['acc_max'] = None
        # axis1['acc_step'] = None
        #
        # axis2 = {}
        # axis2['label'] = self._third_axis_label
        # axis2['ID'] = self._third_axis_ID
        # # axis2['unit'] = 'm'        # the SI units
        # # axis2['ramp'] = None # a possible list of ramps
        # axis2['pos_min'] = self._min_third
        # axis2['pos_max'] = self._max_third
        # axis2['scan_min'] = -10e-6
        # axis2['scan_max'] = 10e-6
        # axis2['pos_step'] = self.step_third_axis
        # axis2['vel_min'] = self._vel_min_third
        # axis2['vel_max'] = self._vel_max_third
        # axis2['vel_step'] = self._vel_step_third
        # axis2['acc_min'] = None
        # axis2['acc_max'] = None
        # axis2['acc_step'] = None
        #
        #
        # # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
       # constraints[axis2['label']] = axis2

        return constraints

    def move_rel(self, param_dict):
        """Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.


        @return dict pos: dictionary with the current magnet position
        """

        for axis_label in param_dict:
            # self.log.info(axis_label)
            step = param_dict[axis_label]
            self._do_move_rel(axis_label, step)

        return param_dict

    def move_abs(self, param_dict):
        """Moves stage to absolute position

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
                                The values for the axes are in millimeter,
                                the value for the rotation is in degrees.

        @return dict pos: dictionary with the current axis position
        """
            #     for axis_label in param_dict:
            # move = param_dict[axis_label]
            # self._do_move_abs(axis_label, move)

        self._do_move_abs(param_dict)

        return param_dict

    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.stepper.close_connection()
        return 0

    def get_pos(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.        """

        constraints = self.get_constraints()
        param_dict = {}

        values = self.stepper.get_pos()

        if param_list is not None:
            for axis_label in param_list:
                if axis_label is 'x':
                    param_dict['x'] = values[0]
                if axis_label is 'y':
                    param_dict['y'] = values[1]

        else:
            param_dict['x'] = values[0]
            param_dict['y'] = values[1]

        return param_dict


    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
         """
        constraints = self.get_constraints()
        return 0

    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        After calibration the stage moves to home position which will be the
        zero point for the passed axis.

        @return dict pos: dictionary with the current position of the ac#xis
        """

        self.stepper.set_zero()

        return 0

    def get_velocity(self, param_list=None):
        """ Gets the current velocity for all connected axes in m/s.

        @param list param_list: optional, if a specific velocity of an axis
                                    is desired, then the labels of the needed
                                    axis should be passed as the param_list.
                                    If nothing is passed, then from each axis the
                                    velocity is asked.

        @return dict : with the axis label as key and the velocity as item.
            """
        constraints = self.get_constraints()
        param_dict = {}
        try:
            if param_list is not None:
                for axis_label in param_list:
                    param_dict[axis_label] = self.vel[axis_label]
            else:
                for axis_label in constraints:
                    param_dict[axis_label] = self.vel[axis_label]
            return param_dict
        except:
            self.log.error('Could not find current axis velocity')
            return -1

    def set_velocity(self, param_dict):
        """ Write new value for velocity in m/s.

        @param dict param_dict: dictionary, which passes all the relevant
                                    parameters, which should be changed. Usage:
                                     {'axis_label': <the-velocity-value>}.
                                     'axis_label' must correspond to a label given
                                     to one of the axis.

        @return dict param_dict2: dictionary with the updated axis velocity
        """
        # constraints = self.get_constraints()
        try:
            for axis_label in param_dict:
                self.vel[axis_label] = int(param_dict[axis_label])

            return param_dict

        except:
            self.log.error('Could not set axis velocity')
            return -1



            ########################## internal methods ##################################


    def _do_get_pos(self, axis):
        constraints = self.get_constraints()
        voltage = self.stepper.get_voltage(axis)
        #print('voltage ')
        #print(voltage)
        pos = self._volt2dist(float(voltage))
        #print(pos)
        return pos

    def _do_move_rel(self, axis, step):
        """internal method for the relative move

        @param axis string: name of the axis that should be moved

        @param float step: step in millimeter

        @return str axis: axis which is moved
                move float: absolute position to move to
        """
        constraints = self.get_constraints()
        if not (abs(constraints[axis]['pos_step']) < abs(step)):
            self.log.warning('Cannot make the movement of the axis "{0}"'
                             'since the step is too small! Ignore command!')
        else:
            current_pos = self.get_pos(axis)[axis]
            move = current_pos + step
            # self.log.info('Move is {0} '.format(move))
            self._do_move_abs(axis, move)
        return axis, move


    def _volt2dist(self, voltage):

        self.dist = -1*(20 / 8) * 1e-6 * voltage

        return self.dist

    def _dist2volt(self, dist):

        #Move by 16 to go 20 um
        #Set range for now at 4mm by 4mm with 0 at

        self.volt = -1*float((dist ) / ((20 / 8) * 1e-6))

        return self.volt

    def _do_move_abs(self, param_dict):
        """internal method for the absolute move in meter

        @param axis string: name of the axis that should be moved

        @param float move: desired position in millimeter

        @return str axis: axis which is moved
                move float: absolute position to move to
        """

        constraints = self.get_constraints()

        if not (constraints['x']['pos_min'] <= param_dict['x'] <= constraints['x']['pos_max'])  :
            self.log.warning('Cannot make the movement of the axis "{0}" to {1} since the border [{2},{3}] would be crossed! Ignore command!'.format('x', param_dict['x'], constraints['x']['pos_min'], constraints['x']['pos_max']))
        if not (constraints['y']['pos_min'] <= param_dict['y'] <= constraints['y']['pos_max']) :
            self.log.warning('Cannot make the movement of the axis "{0}" to {1}'
                             'since the border [{2},{3}] would be crossed! Ignore command!'
                             ''.format('y', param_dict['y'], constraints['y']['pos_min'], constraints['y']['pos_max']))
        else:
            self.stepper.set_voltage(param_dict)
            self.log.info('Moving stepper ({0:.2f}, {1:.2f}) um'.format(1e6*param_dict['x'],1e6*param_dict['y']))# 1e7 to convert meter to SI units


    def _in_movement_xyz(self):
        '''this method checks if the magnet is still moving and returns
        a dictionary which of the axis are moving.

        @return: dict param_dict: Dictionary displaying if axis are moving:
        0 for immobile and 1 for moving
        '''
        constraints = self.get_constraints()
        param_dict = {}
        for axis_label in constraints:
            tmp0 = 0  # dunno if moving
            param_dict[axis_label] = tmp0 % 2

        return param_dict

    def _motor_stopped(self):
        '''this method checks if the magnet is still moving and returns
            False if it is moving and True of it is immobile

            @return: bool stopped: False for immobile and True for moving
                '''
        param_dict = self._in_movement_xyz()
        stopped = True
        for axis_label in param_dict:
            if param_dict[axis_label] != 0:
                self.log.info('Dunno if the stage is stopped')
                stopped = False
        return stopped


import serial
import logging
import time


class StepperController(serial.Serial):
    '''
    Python class for controlling 2-axis stepper motor for broad stepper scans
    '''

    def __init__(self, MAX_VOLTAGE=800.0, port='COM6', baudrate=57600):

        self.MAX_VOLTAGE = MAX_VOLTAGE

        # Initialise the class using super class of (py)serial
        serial.Serial.__init__(self, port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                               stopbits=serial.STOPBITS_ONE, timeout=0.3)

        # Close any open connections and open a serial connection
        # self.close()
        self.xvalue = 0.0
        self.yvalue = 0.0

        #logging.info('Checking voltage')
        #check = self.get_voltage_from_hw()

        self.histo_file = open('stepper.txt', 'a+')

        logging.info('(X,Y) stepper connected')


    def get_voltage(self):

        return(self.xvalue, self.yvalue)

    def get_pos(self):
        return(self.vol2dist(self.xvalue),self.vol2dist(self.yvalue))



    def get_voltage_from_hw(self):
        '''
        get the voltage for the x,y,z axes
        --------
        axis - (str) x y or z axis to set the voltage
        '''

        self.__sendData(2)
        value = self.__getData()
        while 'w' in value or value is '':
            value = self.__getData()

        split = value.split(',')
        x = -1*int(split[0])
        y = int(split[1])
        return (x,y)


    def set_zero(self):
        self.__sendData(4)
        return self.__getData()

    def set_voltage(self, param_dict, step=2.5):

        #convert value to voltage
        #update local voltage
        #send to motor driver

        for axis_label in param_dict:
            move = param_dict[axis_label]
            if axis_label is 'x':
                self.xvalue = -1*int(self.dist2volt(move))
            if axis_label is 'y':
                self.yvalue = int(self.dist2volt(move))

        self.histo_file.write('{0}, {1}'.format(self.xvalue, self.yvalue) + '\n')


        self.__sendData(1)
        self.__sendData(self.xvalue)
        self.__sendData(self.yvalue)

        return self.__getData()


    def __sendData(self, serial_data):
        while (("w" not in self.__getData()) ):
            pass
        serial_data = str(int(serial_data)).encode()
        self.write(serial_data)

    def __getData(self):
        input_string = self.readline()
        input_string = input_string.decode('utf-8')
        return input_string.rstrip('\r\n')


    def zero_all_axes(self):
        '''
        Set all the axis to zero
        #############################################################
        WARNING DO NOT EXECUTE THIS COMMAND WHEN VGA IS NEAR THE CHIP
        #############################################################
        '''
        self.set_voltage("x", 0.0)
        self.set_voltage("y", 0.0)
        self.set_voltage("z", 0.0)

    def close_connection(self):
        self.close()
        #time.sleep(0.2)

    def vol2dist(self, voltage):

        self.dist = 1*(20 / 8) * 1e-6 * voltage

        return self.dist

    def dist2volt(self, dist):

        #Move by 16 to go 20 um
        #Set range for now at 4mm by 4mm with 0 at

        volt = float((dist ) / ((20 / 8) * 1e-6))

        if not -2500 <= volt <= 2500:
            logging.error('The requested {0} step is out of range'.format(volt))
            return 0

        return volt

    def __del__(self):

        self.close()
        self.histo_file.write('--' + '\n')
        self.histo_file.close()



