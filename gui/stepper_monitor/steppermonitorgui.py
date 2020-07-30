# -*- coding: utf-8 -*-

"""
This file contains the Qudi GUI for general Confocal control.

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
import os
import pyqtgraph as pg

import pyqtgraph.opengl as gl



import time

from collections import OrderedDict

#import phidl.geometry as ph

from astropy.stats import sigma_clipped_stats
#from photutils import DAOStarFinder
from qimage2ndarray import rgb_view


from qtpy.QtCore import QPointF

from qtpy.QtGui import QColor, QPolygonF, QPen, QGraphicsPolygonItem

from core.util.modules import get_main_dir


from core.module import Connector, ConfigOption, StatusVar
from gui.guibase import GUIBase
from gui.guiutils import ColorBar
from gui.colordefs import ColorScaleInferno
from gui.colordefs import ColorScaleInfernoAlphaGL
from gui.colordefs import ColorScaleInfernoAlpha

from gui.colordefs import QudiPalettePale as palette
from gui.fitsettings import FitParametersWidget
from qtpy import QtCore
from qtpy import QtGui
from qtpy import QtWidgets
from qtpy import uic


from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem


from OpenGL.GL import glVertex3f, glEnd, glColor4f, glBegin, GL_LINES


class StepperMainWindow(QtWidgets.QMainWindow):

    """ Create the Mainwindow based on the corresponding *.ui file. """

    sigPressKeyBoard = QtCore.Signal(QtCore.QEvent)
    sigDoubleClick = QtCore.Signal()

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_steppermongui.ui')
        self._doubleclicked = False

        # Load it
        super(StepperMainWindow, self).__init__()
        uic.loadUi(ui_file, self)
        self.show()

    def keyPressEvent(self, event):
        """Pass the keyboard press event from the main window further. """
        self.sigPressKeyBoard.emit(event)

    def mouseDoubleClickEvent(self, event):
        self._doubleclicked = True
        self.sigDoubleClick.emit()


class ConfocalSettingDialog(QtWidgets.QDialog):

    """ Create the SettingsDialog window, based on the corresponding *.ui file."""

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_cf_settings.ui')

        # Load it
        super(ConfocalSettingDialog, self).__init__()
        uic.loadUi(ui_file, self)


class StepperGui(GUIBase):

    """ Main Confocal Class for xy and depth scans.
    """
    _modclass = 'ConfocalGui'
    _modtype = 'gui'

    # declare connectors
    stepperlogic = Connector(interface='StepperLogic')
    savelogic = Connector(interface='SaveLogic')


    # config options for gui
    fixed_aspect_ratio_xy = ConfigOption('fixed_aspect_ratio_xy', True)
    fixed_aspect_ratio_depth = ConfigOption('fixed_aspect_ratio_depth', True)
    image_x_padding = ConfigOption('image_x_padding', 0.02)
    image_y_padding = ConfigOption('image_y_padding', 0.02)
    image_z_padding = ConfigOption('image_z_padding', 0.02)

    # status var
    adjust_cursor_roi = StatusVar(default=True)
    slider_small_step = StatusVar(default=10e-9)    # initial value in meter
    slider_big_step = StatusVar(default=100e-9)     # initial value in meter


    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initializes all needed UI files and establishes the connectors.

        This method executes the all the inits for the differnt GUIs and passes
        the event argument from fysom to the methods.
        """

        # Getting an access to all connectors:
        self._stepper_logic = self.get_connector('stepperlogic')


        self._hardware_state = True

        self.single_clicked = False

        self.flagset = 0

        self.initMainUI()      # initialize the main GUI
        self.initSettingsUI()  # initialize the settings GUI
        #self.initOptimizerSettingsUI()  # initialize the optimizer settings GUI



    def initMainUI(self):
        """ Definition, configuration and initialisation of the confocal GUI.

        This init connects all the graphic modules, which were created in the
        *.ui file and configures the event handling between the modules.
        Moreover it sets default values.
        """
        self._mw = StepperMainWindow()

        ###################################################################
        #               Configuring the dock widgets                      #
        ###################################################################
        # All our gui elements are dockable, and so there should be no "central" widget.
        #self._mw.centralwidget.hide()
        self._mw.setDockNestingEnabled(True)


        # Move cursor in piezo or stepper range
        self.move_stepper = False

        # Set initial position for the crosshair, default is the middle of the
        # screen:

        ini_pos_z_crosshair = 0


        self._mw.xjogstep.setValue(self._stepper_logic.initial_jog_x_step)
        self._mw.yjogstep.setValue(self._stepper_logic.initial_jog_y_step)
        self._mw.zjogstep.setValue(self._stepper_logic.initial_jog_z_step)

        self._mw.jogfor_x.clicked.connect(self.jog_x_up_pressed)
        self._mw.jogback_x.clicked.connect(self.jog_x_down_pressed)

        self._mw.jogfor_y.clicked.connect(self.jog_y_up_pressed)
        self._mw.jogback_y.clicked.connect(self.jog_y_down_pressed)

        self._mw.jogfor_z.clicked.connect(self.jog_z_up_pressed)
        self._mw.jogback_z.clicked.connect(self.jog_z_down_pressed)


        # Setup the Sliders:
        # Calculate the needed Range for the sliders. The image ranges comming
        # from the Logic module must be in meters.
        # 1 nanometer resolution per one change, units are meters
        self.slider_res = 20e-6

        # How many points are needed for that kind of resolution:
        num_of_points_x = (self._stepper_logic.x_range[1] - self._stepper_logic.x_range[0]) / self.slider_res
        num_of_points_y = (self._stepper_logic.y_range[1] - self._stepper_logic.y_range[0]) / self.slider_res
        num_of_points_z = (self._stepper_logic.z_range[1] - self._stepper_logic.z_range[0]) / self.slider_res

        # Set a Range for the sliders:
        self._mw.x_SliderWidget.setRange(0, num_of_points_x)
        self._mw.y_SliderWidget.setRange(0, num_of_points_y)

        self._mw.z_SliderWidget.setRange(0, num_of_points_y)

        # Just to be sure, set also the possible maximal values for the spin
        # boxes of the current values:
        self._mw.x_current_InputWidget.setRange(self._stepper_logic.x_range[0], self._stepper_logic.x_range[1])
        self._mw.y_current_InputWidget.setRange(self._stepper_logic.y_range[0], self._stepper_logic.y_range[1])
        self._mw.z_current_InputWidget.setRange(self._stepper_logic.y_range[0], self._stepper_logic.y_range[1])
        #print('spx range')
        #print(self._stepper_logic.spx_x_range[0], self._stepper_logic.spx_x_range[1])

        # set minimal steps for the current value
        self._mw.x_current_InputWidget.setMinimalStep(10e-6)
        self._mw.y_current_InputWidget.setMinimalStep(10e-6)
        self._mw.z_current_InputWidget.setMinimalStep(10e-6)

        # Predefine the maximal and minimal image range as the default values
        # for the display of the range:
        #self._mw.x_min_InputWidget.setValue(self._stepper_logic.scan_x_range[0])
        #self._mw.x_max_InputWidget.setValue(self._stepper_logic.scan_x_range[1])
        #self._mw.y_min_InputWidget.setValue(self._stepper_logic.scan_y_range[0])
        #self._mw.y_max_InputWidget.setValue(self._stepper_logic.scan_y_range[1])

        # set the minimal step size
        #self._mw.x_min_InputWidget.setOpts(minStep=1e-9)
        #self._mw.x_max_InputWidget.setMinimalStep(1e-9)
        #self._mw.y_min_InputWidget.setOpts(minStep=1e-9)
        #self._mw.y_max_InputWidget.setOpts(minStep=1e-9)

        # Handle slider movements by user:
        self._mw.x_SliderWidget.sliderMoved.connect(self.update_from_slider_x)
        self._mw.y_SliderWidget.sliderMoved.connect(self.update_from_slider_y)
        self._mw.z_SliderWidget.sliderMoved.connect(self.update_from_slider_z)


        # Update the inputed/displayed numbers if the cursor has left the field:
        self._mw.x_current_InputWidget.editingFinished.connect(self.update_from_input_x)
        self._mw.y_current_InputWidget.editingFinished.connect(self.update_from_input_y)
        self._mw.z_current_InputWidget.editingFinished.connect(self.update_from_input_z)


        #self._mw.x_min_InputWidget.editingFinished.connect(self.change_x_range)
        #self._mw.x_max_InputWidget.editingFinished.connect(self.change_x_range)
        #self._mw.y_min_InputWidget.editingFinished.connect(self.change_y_range)
        #self._mw.y_max_InputWidget.editingFinished.connect(self.change_y_range)


        #self._mw.xstepperadjust.editingFinished.connect(self.update_from_x_stepper_adjust)
        #self._mw.ystepperadjust.editingFinished.connect(self.update_from_y_stepper_adjust)
        #self._mw.zstepperadjust.editingFinished.connect(self.update_from_z_stepper_adjust)


        # Connect the change of the viewed area to an adjustment of the ROI:
        #self.adjust_cursor_roi = True
        #self.qpoly.shape().sigRangeChanged.connect(self.update_roi_xy_size)


        #################################################################
        #                           Actions                             #
        #################################################################
        # Connect the scan actions to the events if they are clicked. Connect
        # also the adjustment of the displayed windows.
        self._mw.action_stop_scanning.triggered.connect(self.ready_clicked)
        self._mw.actionZero_Stepper.triggered.connect(self.zero_stepper)
        self._mw.moveMotor.clicked.connect(self.move_motor_button_press)

        self._mw.sigPressKeyBoard.connect(self.keyPressEvent)

        self._mw.reset_x.clicked.connect(self.reset_x_button_press)
        self._mw.reset_y.clicked.connect(self.reset_y_button_press)
        self._mw.reset_z.clicked.connect(self.reset_z_button_press)
        self._mw.home_x.clicked.connect(self.home_x_button_press)
        self._mw.home_y.clicked.connect(self.home_y_button_press)
        self._mw.home_z.clicked.connect(self.home_z_button_press)

        pos = self._stepper_logic.get_motor_position()

        # update x slider and input widget
        self.update_slider_x(pos[0])
        self.update_input_x(pos[0])
        self.update_slider_y(pos[1])
        self.update_input_y(pos[1])
        self.update_slider_z(pos[2])
        self.update_input_z(pos[2])

        raw = self._stepper_logic.get_motor_position_raw()

        self._mw.xraw.setText('{0}'.format(raw[0]))
        self._mw.yraw.setText('{0}'.format(raw[1]))
        self._mw.zraw.setText('{0}'.format(raw[2]))

        vel = self._stepper_logic.get_motor_velocity()
        self.update_vel(vel)


        raw = self._stepper_logic.get_motor_params()


        self.fillTreeItem(self._mw.xlimits, raw[0])
        self.fillTreeItem(self._mw.ylimits, raw[1])
        self.fillTreeItem(self._mw.zlimits, raw[2])

        #self._mw.ylimit.setText('{0}'.format(raw[1]))
        #self._mw.zlimit.setText('{0}'.format(raw[2]))

        self.show()

    def initSettingsUI(self):
        """ Definition, configuration and initialisation of the settings GUI.

        This init connects all the graphic modules, which were created in the
        *.ui file and configures the event handling between the modules.
        Moreover it sets default values if not existed in the logic modules.
        """
        # Create the Settings window
        self._sd = ConfocalSettingDialog()
        # Connect the action of the settings window with the code:
        self._sd.accepted.connect(self.update_settings)
        self._sd.rejected.connect(self.keep_former_settings)
        self._sd.buttonBox.button(QtWidgets.QDialogButtonBox.Apply).clicked.connect(self.update_settings)
        self._sd.hardware_switch.clicked.connect(self.switch_hardware)

        # write the configuration to the settings window of the GUI.
        self.keep_former_settings()


    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        self._mw.close()
        return 0

    def show(self):
        """Make main window visible and put it above all other windows. """
        # Show the Main Confocal GUI:
        self._mw.show()
        self._mw.activateWindow()
        self._mw.raise_()




    def fillTreeItem(self, widget, valuedict):
        """ Recursively fill a QTreeWidgetItem with the contents from a
            dictionary.
          @param QTreeWidgetItem item: the widget item to fill
          @param (dict, list, etc) value: value to fill in
        """

        for key,value in valuedict.items():

            if '(raw)' in str(key):
                #child = widget.[]#QtWidgets.QTreeWidgetItem(widget, [key.replace(' (raw)','')])
                child.setText(2, value)
            else:
                child = QtWidgets.QTreeWidgetItem(widget, [key])
                child.setText(1, value)


    def keyPressEvent(self, event):
        """ Handles the passed keyboard events from the main window.

        @param object event: qtpy.QtCore.QEvent object.
        """
        modifiers = QtWidgets.QApplication.keyboardModifiers()

        # position = self._stepper_logic.get_motor_position()   # in meters
        # x_pos = position[0]
        # y_pos = position[1]
        # z_pos = position[2]

        # if modifiers == QtCore.Qt.ControlModifier:
        #     if event.key() == QtCore.Qt.Key_M:
        #         self.hline_xy.stepper = not self.hline_xy.stepper
        #         self.vline_xy.stepper = not self.vline_xy.stepper
        #     if event.key() == QtCore.Qt.Key_Right:
        #         self.update_from_key(x=float(round(x_pos + self.slider_big_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Left:
        #         self.update_from_key(x=float(round(x_pos - self.slider_big_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Up:
        #         self.update_from_key(y=float(round(y_pos + self.slider_big_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Down:
        #         self.update_from_key(y=float(round(y_pos - self.slider_big_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_PageUp:
        #         self.update_from_key(z=float(round(z_pos + self.slider_big_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_PageDown:
        #         self.update_from_key(z=float(round(z_pos - self.slider_big_step, 10)))
        #     else:
        #         event.ignore()
        # else:
        #     if event.key() == QtCore.Qt.Key_Right:
        #         self.update_from_key(x=float(round(x_pos + self.slider_small_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Left:
        #         self.update_from_key(x=float(round(x_pos - self.slider_small_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Up:
        #         self.update_from_key(y=float(round(y_pos + self.slider_small_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_Down:
        #         self.update_from_key(y=float(round(y_pos - self.slider_small_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_PageUp:
        #         self.update_from_key(z=float(round(z_pos + self.slider_small_step, 10)))
        #     elif event.key() == QtCore.Qt.Key_PageDown:
        #         self.update_from_key(z=float(round(z_pos - self.slider_small_step, 10)))
        #     else:
        #         event.ignore()


    def menu_settings(self):
        """ This method opens the settings menu. """
        self._sd.exec_()

    def update_settings(self):
        """ Write new settings from the gui to the file. """
        self._stepper_logic.set_clock_frequency(self._sd.clock_frequency_InputWidget.value())
        self._stepper_logic.return_slowness = self._sd.return_slowness_InputWidget.value()
        self._stepper_logic.permanent_scan = self._sd.loop_scan_CheckBox.isChecked()
        self.fixed_aspect_ratio_xy = self._sd.fixed_aspect_xy_checkBox.isChecked()
        self.slider_small_step = self._sd.slider_small_step_DoubleSpinBox.value()
        self.slider_big_step = self._sd.slider_big_step_DoubleSpinBox.value()
        #self.adjust_cursor_roi = self._sd.adjust_cursor_to_optimizer_checkBox.isChecked()

        # Update GUI icons to new loop-scan state
        self._set_scan_icons()
        # update cursor


    def keep_former_settings(self):
        """ Keep the old settings and restores them in the gui. """
        self._sd.clock_frequency_InputWidget.setValue(int(self._stepper_logic._clock_frequency))
        self._sd.return_slowness_InputWidget.setValue(int(self._stepper_logic.return_slowness))
        self._sd.loop_scan_CheckBox.setChecked(self._stepper_logic.permanent_scan)


        #self._sd.adjust_cursor_to_optimizer_checkBox.setChecked(self.adjust_cursor_roi)
        self._sd.fixed_aspect_xy_checkBox.setChecked(self.fixed_aspect_ratio_xy)

        self._sd.slider_small_step_DoubleSpinBox.setValue(float(self.slider_small_step))
        self._sd.slider_big_step_DoubleSpinBox.setValue(float(self.slider_big_step))

    def menu_optimizer_settings(self):
        """ This method opens the settings menu. """
        #self.keep_former_optimizer_settings()
        self._osd.exec_()




    def zero_stepper(self):
        """ Zeros stepper. """

        self.disable_scan_actions()
        self._stepper_logic.zero_stepper()
        self.enable_scan_actions()




    def ready_clicked(self):
        """ Stop the scan if the state has switched to ready. """



        if self._stepper_logic.module_state() == 'locked':
            self._stepper_logic.permanent_scan = False
            self._stepper_logic.stop_scanning_spx()
        # if self._optimizer_logic.module_state() == 'locked':
        #     self._optimizer_logic.stop_refocus()
        if self._track_logic.module_state() == 'locked':
            self._track_logic.stop_refocus()
        self.enable_scan_actions()

    def xy_scan_clicked(self):
        """ Manages what happens if the xy scan is started. """
        self.disable_scan_actions()
        if self.single_clicked is True:
            self._stepper_logic.start_super_scan(tag = 'single')
        else:
            self._stepper_logic.start_super_scan()

    def continue_xy_scan_clicked(self):
        """ Continue xy scan. """
        self.disable_scan_actions()
        self._stepper_logic.continue_scanning(zscan=False,tag='gui')



    def refocus_clicked(self):
        """ Start optimize position. """
        self.disable_scan_actions()
        # Get the current crosshair position to send to optimizer
        crosshair_pos = self._stepper_logic.get_position()
        #self.sigStartOptimizer.emit(crosshair_pos, 'confocalgui')



    def move_motor_button_press(self):

        #propagate values to motor

        x_pos = self._mw.x_current_InputWidget.value()
        y_pos = self._mw.y_current_InputWidget.value()
        z_pos = self._mw.z_current_InputWidget.value()

        print('Moving motor to ', x_pos, y_pos, z_pos)

        self._stepper_logic.change_motor_position(x=x_pos, y=y_pos, z = z_pos)
        self.update_raw()

    def reset_x_button_press(self):
        self._stepper_logic.reset_x()
        self.update_params()

    def reset_y_button_press(self):
        self._stepper_logic.reset_y()
        self.update_params()

    def reset_z_button_press(self):
        self._stepper_logic.reset_z()
        self.update_params()

    def home_x_button_press(self):
        self._stepper_logic.home_x()
        x_motor = self._stepper_logic.get_motor_position()[0]

        self.update_slider_x(x_motor)
        self.update_input_x(x_motor)

        self.update_raw()

    def home_y_button_press(self):
        self._stepper_logic.home_y()
        y_motor = self._stepper_logic.get_motor_position()[1]

        self.update_slider_y(y_motor)
        self.update_input_y(y_motor)

        self.update_raw()

    def home_z_button_press(self):
        self._stepper_logic.home_z()
        z_motor = self._stepper_logic.get_motor_position()[2]

        self.update_slider_z(z_motor)
        self.update_input_z(z_motor)

        self.update_raw()
        #self.update_params()


    def update_params(self):
        raw = self._stepper_logic.get_motor_params()
        self.fillTreeItem(self._mw.xlimits, raw[0])
        self.fillTreeItem(self._mw.ylimits, raw[1])
        self.fillTreeItem(self._mw.zlimits, raw[2])


    def update_from_key(self, x=None, y=None, z=None):
        """The user pressed a key to move the crosshair, adjust all GUI elements.

        @param float x: new x position in m
        @param float y: new y position in m
        @param float z: new z position in m
        """
        if x is not None:




            self._stepper_logic.set_position('xinput', x=x)

        if y is not None:



            self._stepper_logic.set_position('yinput', y=y)


        if z is not None:


            self._stepper_logic.set_position('zinput', z=z)


    def update_raw(self):
        raw = self._stepper_logic.get_motor_position_raw()
        self._mw.xraw.setText('{0}'.format(raw[0]))
        self._mw.yraw.setText('{0}'.format(raw[1]))
        self._mw.zraw.setText('{0}'.format(raw[2]))

    def update_from_input_x(self):
        """ The user changed the number in the x position spin box, adjust all
            other GUI elements."""
        x_pos = self._mw.x_current_InputWidget.value()

        self.update_slider_x(x_pos)

        self.x_pos = x_pos



    def update_from_input_y(self):
        """ The user changed the number in the y position spin box, adjust all
            other GUI elements."""
        y_pos = self._mw.y_current_InputWidget.value()

        self.update_slider_y(y_pos)

        self.y_pos = y_pos

    def update_from_input_z(self):
        """ The user changed the number in the y position spin box, adjust all
            other GUI elements."""
        z_pos = self._mw.z_current_InputWidget.value()

        self.update_slider_z(z_pos)

        self.z_pos = z_pos



    def update_from_x_stepper_adjust(self):

        jog_step = self._mw.xjogstep.value()

        # change jog step
        self._stepper_logic.change_jog_step({'x' :jog_step})

    def update_from_y_stepper_adjust(self):

        jog_step = self._mw.yjogstep.value()

        # change jog step
        self._stepper_logic.change_jog_step({'y' :jog_step})

    def update_from_z_stepper_adjust(self):

        jog_step = self._mw.zjogstep.value()

        # change jog step
        self._stepper_logic.change_jog_step({'z' :jog_step})


    def jog_x_up_pressed(self):

        #send jog
        self._stepper_logic.jog(x=1)

        x_motor = self._stepper_logic.get_motor_position()[0]

        #update x slider and input widget
        self.update_slider_x(x_motor)
        self.update_input_x(x_motor)
        self.update_raw()


    def jog_x_down_pressed(self):

        #send jog
        self._stepper_logic.jog(x=-1)

        x_motor = self._stepper_logic.get_motor_position()[0]

        #update x slider and input widget
        self.update_slider_x(x_motor)
        self.update_input_x(x_motor)
        self.update_raw()

    def jog_y_up_pressed(self):

        #send jog
        self._stepper_logic.jog(y=1)

        y_motor = self._stepper_logic.get_motor_position()[1]

        #update y slider and input widget
        self.update_slider_y(y_motor)
        self.update_input_y(y_motor)
        self.update_raw()


    def jog_y_down_pressed(self):

        #send jog
        self._stepper_logic.jog(y=-1)

        y_motor = self._stepper_logic.get_motor_position()[1]

        # update y slider and input widget
        self.update_slider_y(y_motor)
        self.update_input_y(y_motor)
        self.update_raw()

    def jog_z_up_pressed(self):

        #send jog
        self._stepper_logic.jog(z=1)

        z_motor = self._stepper_logic.get_motor_position()[2]

        #update zslider and input widget
        self.update_slider_z(z_motor)
        self.update_input_z(z_motor)
        self.update_raw()


    def jog_z_down_pressed(self):

        #send jog
        self._stepper_logic.jog(z=-1)

        z_motor = self._stepper_logic.get_motor_position()[2]

        # update z slider and input widget
        self.update_slider_z(z_motor)
        self.update_raw()


    def update_vel(self, vel):
        """ Update the displayed x-value.

        @param float x_pos: the current value of the x position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.xvel.setValue(vel[0])
        self._mw.yvel.setValue(vel[1])
        self._mw.zvel.setValue(vel[2])


    def update_input_x(self, x_pos):
        """ Update the displayed x-value.

        @param float x_pos: the current value of the x position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.x_current_InputWidget.setValue(x_pos)


    def update_input_y(self, y_pos):
        """ Update the displayed y-value.

        @param float y_pos: the current value of the y position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.y_current_InputWidget.setValue(y_pos)


    def update_input_z(self, z_pos):
        """ Update the displayed z-value.

        @param float z_pos: the current value of the z position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.z_current_InputWidget.setValue(z_pos)



    def update_from_slider_x(self, sliderValue):
        """The user moved the x position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        x_pos = self._stepper_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_input_x(x_pos)
        self.x_pos = x_pos


    def update_from_slider_y(self, sliderValue):
        """The user moved the y position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        y_pos = self._stepper_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_input_y(y_pos)
        self.y_pos = y_pos



    def update_from_slider_z(self, sliderValue):
        """The user moved the z position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        z_pos = self._stepper_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_input_z(z_pos)
        self.z_pos = z_pos



    def update_slider_x(self, x_pos):
        """ Update the x slider when a change happens.

        @param float x_pos: x position in m
        """
        self._mw.x_SliderWidget.setValue((x_pos - self._stepper_logic.motor_range[0]) / self.slider_res)

    def update_slider_y(self, y_pos):
        """ Update the y slider when a change happens.

        @param float y_pos: x yosition in m
        """
        self._mw.y_SliderWidget.setValue((y_pos - self._stepper_logic.motor_range[0]) / self.slider_res)


    def update_slider_z(self, z_pos):
        """ Update the z slider when a change happens.

        @param float z_pos: z position in m
        """
        self._mw.z_SliderWidget.setValue((z_pos - self._stepper_logic.motor_range[0]) / self.slider_res)



    def change_xy_resolution(self):
        """ Update the xy resolution in the logic according to the GUI.
        """
        #self._stepper_logic.xy_resolution = self._mw.xy_res_InputWidget.value()

    def change_z_resolution(self):
        """ Update the z resolution in the logic according to the GUI.
        """
        self._stepper_logic.z_resolution = self._mw.z_res_InputWidget.value()

    def change_x_range(self):
        """ Adjust the image range for x in the logic. """
        self._stepper_logic.scan_x_range = [
            -1*self._mw.x_max_InputWidget.value(),
            self._mw.x_max_InputWidget.value()]

        self._stepper_logic.scan_y_range = [
            -1 * self._mw.x_max_InputWidget.value(),
            self._mw.x_max_InputWidget.value()]



    def switch_hardware(self):
        """ Switches the hardware state. """
        self._stepper_logic.switch_hardware(to_on=False)

    def restore_default_view(self):
        """ Restore the arrangement of DockWidgets to the default
        """
        # Show any hidden dock widgets
        self._mw.xy_scan_dockWidget.show()
        self._mw.scan_control_dockWidget.show()
        self._mw.depth_scan_dockWidget.show()
        self._mw.optimizer_dockWidget.show()
        self._mw.tilt_correction_dockWidget.hide()
        self._mw.scanLineDockWidget.hide()

        # re-dock any floating dock widgets
        self._mw.xy_scan_dockWidget.setFloating(False)
        self._mw.scan_control_dockWidget.setFloating(False)
        self._mw.depth_scan_dockWidget.setFloating(False)
        self._mw.optimizer_dockWidget.setFloating(False)
        self._mw.tilt_correction_dockWidget.setFloating(False)
        self._mw.scanLineDockWidget.setFloating(False)

        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(1), self._mw.xy_scan_dockWidget)
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(8), self._mw.scan_control_dockWidget)
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(2), self._mw.depth_scan_dockWidget)
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(2), self._mw.optimizer_dockWidget)
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(8), self._mw.tilt_correction_dockWidget)
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(2), self._mw.scanLineDockWidget)

        # Resize window to default size
        self._mw.resize(1255, 939)



    def set_full_scan_range_xy(self):
        #xMin = self._stepper_logic.scan_x_range[0]
        xMax = self._stepper_logic.scan_x_range[1]
        #self._mw.x_min_InputWidget.setValue(xMin)
        self._mw.x_max_InputWidget.setValue(xMax)
        #self.change_x_image_range()

        #yMin = self._stepper_logic.scan_y_range[0]
        #yMax = self._stepper_logic.scan_y_range[1]
        #self._mw.y_min_InputWidget.setValue(yMin)
        #self._mw.y_max_InputWidget.setValue(yMax)
        #self.change_y_image_range()

        # for i in range(2):
        #     self.xy_image.getViewBox().setRange(xRange=(xMin, xMax), yRange=(yMin, yMax),
        #         update=True)

    def activate_zoom_double_click(self):
        if self._mw.action_zoom.isChecked():
            self._mw.action_zoom.setChecked(False)
        else:
            self._mw.action_zoom.setChecked(True)

    def depth_scan_start_zoom_point(self, event):
        """ Get the mouse coordinates if the mouse button was pressed.

        @param QMouseEvent event: Mouse Event object which contains all the
                                  information at the time the event was emitted
        """
        if self._mw._doubleclicked:
            event.ignore()
            return
        # catch the event if the zoom mode is activated and if the event is
        # coming from a left mouse button.
        if not (self._mw.action_zoom.isChecked() and (event.button() == QtCore.Qt.LeftButton)):
            event.ignore()
            return

        pos = self.depth_image.getViewBox().mapSceneToView(event.localPos())
        self._current_depth_zoom_start = [pos.x(), pos.y()]

        # store the initial mouse position in a class variable
        event.accept()

    def depth_scan_end_zoom_point(self, event):
        """ Get the mouse coordinates if the mouse button was released.

        @param QEvent event:
        """
        if self._mw._doubleclicked:
            self._mw._doubleclicked = False
            event.ignore()
            return

        # catch the event if the zoom mode is activated and if the event is
        # coming from a left mouse button.
        if not (self._mw.action_zoom.isChecked() and (event.button() == QtCore.Qt.LeftButton)):
            event.ignore()
            return

        # get the ViewBox which is also responsible for the depth_image
        viewbox = self.depth_image.getViewBox()

        # Map the mouse position in the whole ViewWidget to the coordinate
        # system of the ViewBox, which also includes the 2D graph:
        pos = viewbox.mapSceneToView(event.localPos())
        endpos = [pos.x(), pos.y()]
        initpos = self._current_depth_zoom_start

        # get the right corners from the zoom window:
        if initpos[0] > endpos[0]:
            xMin = endpos[0]
            xMax = initpos[0]
        else:
            xMin = initpos[0]
            xMax = endpos[0]

        if initpos[1] > endpos[1]:
            zMin = endpos[1]
            zMax = initpos[1]
        else:
            zMin = initpos[1]
            zMax = endpos[1]

        self._mw.z_min_InputWidget.setValue(zMin)
        self._mw.z_max_InputWidget.setValue(zMax)
        self.change_z_image_range()

        event.accept()
        # Finally change the visible area of the ViewBox:
        viewbox.setRange(xRange=(xMin, xMax), yRange=(zMin, zMax))
        # second time is really needed, otherwisa zooming will not work for the first time
        viewbox.setRange(xRange=(xMin, xMax), yRange=(zMin, zMax))

        self._mw.action_zoom.setChecked(False)


    def reset_depth_imagerange(self):
        """ Reset the imagerange if autorange was pressed.

        Take the image range values directly from the scanned image and set
        them as the current image ranges.
        """
        # extract the range directly from the image:
        xMin = self._stepper_logic.depth_image[0, 0, 0]
        zMin = self._stepper_logic.depth_image[0, 0, 2]
        xMax = self._stepper_logic.depth_image[-1, -1, 0]
        zMax = self._stepper_logic.depth_image[-1, -1, 2]


    def set_full_scan_range_z(self):

        if self._stepper_logic.depth_img_is_xz:
            hMin = self._stepper_logic.x_range[0]
            hMax = self._stepper_logic.x_range[1]
            # self._mw.x_min_InputWidget.setValue(hMin)
            # self._mw.x_max_InputWidget.setValue(hMax)
            # self.change_x_image_range()
        else:
            hMin = self._stepper_logic.y_range[0]
            hMax = self._stepper_logic.y_range[1]
            # self._mw.y_min_InputWidget.setValue(hMin)
            # self._mw.y_max_InputWidget.setValue(hMax)
            # self.change_y_image_range()

        vMin = self._stepper_logic.z_range[0]
        vMax = self._stepper_logic.z_range[1]
        self._mw.z_min_InputWidget.setValue(vMin)
        self._mw.z_max_InputWidget.setValue(vMax)
        self.change_z_image_range()

        for i in range(2):
            self.depth_image.getViewBox().setRange(xRange=(hMin, hMax), yRange=(vMin, vMax), update=True)

