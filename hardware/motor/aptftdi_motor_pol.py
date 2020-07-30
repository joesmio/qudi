# -*- coding: utf-8 -*-

"""
This file contains the dummy for a motorized stage interface.

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


from collections import OrderedDict
import time

from core.module import Base
from interface.motor_interface import MotorInterface

from hardware.bpc.apt import AptMotor


class MotorAxisDummy:
    """ Generic dummy motor representing one axis. """
    def __init__(self, label):
        self.label = label


class MotorApt(Base, MotorInterface):
    """ This is the dummy class to simulate a motorized stage. """

    _modclass = 'MotorApt'
    _modtype = 'hardware'

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        self.log.debug('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.info('{0}: {1}'.format(key,config[key]))

        # these label should be actually set by the config.
        self._x_axis = MotorAxisDummy('x')

        self._wait_after_movement = 1 #in seconds

    #TODO: Checks if configuration is set and is reasonable

    def on_activate(self):

        # PLEASE REMEMBER: DO NOT CALL THE POSITION SIMPLY self.x SINCE IT IS
        # EXTREMLY DIFFICULT TO SEARCH FOR x GLOBALLY IN A FILE!
        # Same applies to all other axis. I.e. choose more descriptive names.

        self._x_axis.pos = 0.0


        self._x_axis.vel = 1.0


        self._x_axis.status = 0


        # scale factor in mm is steps per revolution * micro step resolution * lead screw pitch

        self.kcube_x = AptMotor(hwser=27503007)

        # controller TDC001

        # stage PRM1-Z8

        '''
        hw_type: 'TDC001'
        serial_num: 27503007
        pitch: 17.87
        unit: 'degree'
        constraints:
        pos_min: -360
        pos_max: 720
        vel_min: 1.0
        vel_max: 10.0
        acc_min: 4.0
        acc_max: 10.0
        '''


    def on_deactivate(self):
        pass


    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the magnet hardware. These
                      constraints will be passed via the logic to the GUI so
                      that proper display elements with boundary conditions
                      could be made.

        Provides all the constraints for each axis of a motorized stage
        (like total travel distance, velocity, ...)
        Each axis has its own dictionary, where the label is used as the
        identifier throughout the whole module. The dictionaries for each axis
        are again grouped together in a constraints dictionary in the form

            {'<label_axis0>': axis0 }

        where axis0 is again a dict with the possible values defined below. The
        possible keys in the constraint are defined here in the interface file.
        If the hardware does not support the values for the constraints, then
        insert just None. If you are not sure about the meaning, look in other
        hardware files to get an impression.
        """
        constraints = {}

        config = self.getConfiguration()

        for axis_label in config['axis_labels']:
            # create a dictionary for the constraints of this axis
            this_axis = {}

            axisconfig = config[axis_label]

            # Get the constraints from the config file if they have been specified.
            if 'constraints' in axisconfig:
                constraintsconfig = axisconfig['constraints']
            else:
                constraintsconfig = OrderedDict()

            # Now we can read through these axisconstraints

            # Position minimum (units)
            if 'pos_min' in constraintsconfig.keys():
                this_axis['pos_min'] = constraintsconfig['pos_min']
            else:
                self.log.warning('aptmotor has no pos_min specified in config file,'
                                 'using default value of 0.'
                                 )
                this_axis['pos_min'] = 0

            # Position maximum (units)
            if 'pos_max' in constraintsconfig.keys():
                this_axis['pos_max'] = constraintsconfig['pos_max']
            else:
                self.log.warning('aptmotor has no pos_max specified in config file,'
                                 'using default value of 360.'
                                 )
                this_axis['pos_max'] = 360

            # Velocity minimum (units/s)
            if 'vel_min' in constraintsconfig.keys():
                this_axis['vel_min'] = constraintsconfig['vel_min']
            else:
                self.log.warning('aptmotor has no vel_min specified in config file,'
                                 'using default value of 0.1.'
                                 )
                this_axis['vel_min'] = 0.1

            # Velocity maximum (units/s)
            if 'vel_max' in constraintsconfig.keys():
                this_axis['vel_max'] = constraintsconfig['vel_max']
            else:
                self.log.warning('aptmotor has no vel_max specified in config file,'
                                 'using default value of 5.0.'
                                 )
                this_axis['vel_max'] = 5.0

            # Acceleration minimum (units/s^2)
            if 'acc_min' in constraintsconfig.keys():
                this_axis['acc_min'] = constraintsconfig['acc_min']
            else:
                self.log.warning('aptmotor has no acc_min specified in config file,'
                                 'using default value of 4.0.'
                                 )
                this_axis['acc_min'] = 4.0

            # Acceleration maximum (units/s^2)
            if 'acc_max' in constraintsconfig.keys():
                this_axis['acc_max'] = constraintsconfig['acc_max']
            else:
                self.log.warning('aptmotor has no acc_max specified in config file,'
                                 'using default value of 5.0.'
                                 )
                this_axis['acc_max'] = 5.0

            # What are these ones used for?
            this_axis['ramp'] = ['Trapez']  # a possible list of ramps

            this_axis['pos_step'] = 0.01  # in °
            this_axis['vel_step'] = 0.1  # in °/s (a rather arbitrary number)
            this_axis['acc_step'] = 0.01  # in °/s^2 (a rather arbitrary number)

            constraints[axis_label] = this_axis

        return constraints

    def move_rel(self,  param_dict):
        """ Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed.
                                With get_constraints() you can obtain all
                                possible parameters of that stage. According to
                                this parameter set you have to pass a dictionary
                                with keys that are called like the parameters
                                from get_constraints() and assign a SI value to
                                that. For a movement in x the dict should e.g.
                                have the form:
                                    dict = { 'x' : 23 }
                                where the label 'x' corresponds to the chosen
                                axis label.

        A smart idea would be to ask the position after the movement.
        """
        curr_pos_dict = self.get_pos()
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            move_x = param_dict[self._x_axis.label]
            curr_pos_x = curr_pos_dict[self._x_axis.label]

            if  (curr_pos_x + move_x > constraints[self._x_axis.label]['pos_max'] ) or\
                (curr_pos_x + move_x < constraints[self._x_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis '
                        '"{0}" with the step {1}, since the border [{2},{3}] '
                        'was reached! Ignore command!'.format(
                            self._x_axis.label, move_x,
                            constraints[self._x_axis.label]['pos_min'],
                            constraints[self._x_axis.label]['pos_max']))
            else:
                self._make_wait_after_movement()
                self._x_axis.pos = self._x_axis.pos + move_x

        if param_dict.get(self._y_axis.label) is not None:
            move_y = param_dict[self._y_axis.label]
            curr_pos_y = curr_pos_dict[self._y_axis.label]

            if  (curr_pos_y + move_y > constraints[self._y_axis.label]['pos_max'] ) or\
                (curr_pos_y + move_y < constraints[self._y_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis '
                        '"{0}" with the step {1}, since the border [{2},{3}] '
                        'was reached! Ignore command!'.format(
                            self._y_axis.label, move_y,
                            constraints[self._y_axis.label]['pos_min'],
                            constraints[self._y_axis.label]['pos_max']))
            else:
                self._make_wait_after_movement()
                self._y_axis.pos = self._y_axis.pos + move_y

        if param_dict.get(self._z_axis.label) is not None:
            move_z = param_dict[self._z_axis.label]
            curr_pos_z = curr_pos_dict[self._z_axis.label]

            if  (curr_pos_z + move_z > constraints[self._z_axis.label]['pos_max'] ) or\
                (curr_pos_z + move_z < constraints[self._z_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis '
                        '"{0}" with the step {1}, since the border [{2},{3}] '
                        'was reached! Ignore command!'.format(
                            self._z_axis.label, move_z,
                            constraints[self._z_axis.label]['pos_min'],
                            constraints[self._z_axis.label]['pos_max']))
            else:
                self._make_wait_after_movement()
                self._z_axis.pos = self._z_axis.pos + move_z





    def move_abs(self, param_dict):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <a-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
        A smart idea would be to ask the position after the movement.
        """
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            desired_pos = param_dict[self._x_axis.label]
            constr = constraints[self._x_axis.label]

            if not(constr['pos_min'] <= desired_pos <= constr['pos_max']):
                self.log.warning('Cannot make absolute movement of the axis '
                        '"{0}" to position {1}, since it exceeds the limits '
                        '[{2},{3}] ! Command is ignored!'.format(
                            self._x_axis.label, desired_pos,
                            constr['pos_min'],
                            constr['pos_max']))
            else:
                if abs(desired_pos - self._x_axis.pos) > 1e-7: # put a 100 nm tolerance on the stepper motor
                    self.kcube_x.setPosition(position = float(desired_pos))
                    self._make_wait_after_movement()
                    self._x_axis.pos = desired_pos






    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.log.info('MotorDummy: Movement stopped!')
        return 0

    def get_pos(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pos = {}

        self._x_axis.pos = (self.kcube_x.getPosition())


        pos[self._x_axis.label] = self._x_axis.pos


        return pos

    def get_pos_raw(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pos = {}

        pos[self._x_axis.label] = self.kcube_x.getPositionRaw()

        return pos

    def get_velocity(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pos = {}

        pos[self._x_axis.label] = self.kcube_x.GetVelocity()*1e-3


        return pos

    def get_params(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pos = {}

        pos[self._x_axis.label] = self.kcube_x.getLimitParams()
        pos[self._x_axis.label].update(self.kcube_x.getHomeParams())


        try:
            pos[self._x_axis.label]['CW software limit'] = str((float(pos[self._x_axis.label]['CW software limit'])-3) * 1e-3)
            pos[self._x_axis.label]['CCW software limit'] = str((float(pos[self._x_axis.label]['CCW software limit'])-3) * 1e-3)
        except KeyError:
            pass


        return pos

    def update_encoder(self, param_list = None):

        if param_list is not None:
            if self._x_axis.label in param_list:
                desired_pos = self.kcube_x.UpdateEncoder(position =(param_list[self._x_axis.label]))
                #print(desired_pos)


    def reset_limit_switches(self):
        self.kcube_x.setHomeParams(0)
        self.kcube_x.SetLimitSwitchParams(0)


    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """

        status = {}

        status[self._x_axis.label] = self._x_axis.status


        return status


    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        @return int: error code (0:OK, -1:error)

        After calibration the stage moves to home position which will be the
        zero point for the passed axis. The calibration procedure will be
        different for each stage.
        """

        self._x_axis.pos = 0.0

        return 0


    def home(self,param_list = None):
        if param_list is not None:
            if self._x_axis.label in param_list:
                self.kcube_x.MoveHome(channel=0)


    def reset(self,param_list = None):
        if param_list is not None:
            if self._x_axis.label in param_list:
                self.kcube_x.setHomeParams(0)
                self.kcube_x.SetLimitSwitchParams(0)




    def jog_step(self, param_list = None):

        if param_list is not None:
            if self._x_axis.label in param_list:
                self.kcube_x.jog_step(param_list[self._x_axis.label])




    def jog(self, param_list = None, positive=True):
        self.log.info('Jog not implemented for rotation motor yet')



    def jog_actual(self, param_list = None, positive =True):

        self.log.info('Jog not implemented for rotation motor yet')


    def set_velocity(self, param_dict=None):
        """ Write new value for velocity.

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
        """
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            desired_vel = param_dict[self._x_axis.label]
            constr = constraints[self._x_axis.label]

            if not(constr['vel_min'] <= desired_pos <= constr['vel_max']):
                self.log.warning('Cannot make absolute movement of the axis '
                        '"{0}" to possition {1}, since it exceeds the limits '
                        '[{2},{3}] ! Command is ignored!'.format(
                            self._x_axis.label, desired_vel,
                            constr['vel_min'],
                            constr['vel_max']))
            else:
                self._x_axis.vel = desired_vel



    def _make_wait_after_movement(self):
        """ Define a time which the dummy should wait after each movement. """
        time.sleep(self._wait_after_movement)

