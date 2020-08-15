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

#import phidl.geometry as ph ## Need to rebuild this package JS 2/2020

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


class GLTextItem(GLGraphicsItem):
    def __init__(self, X=None, Y=None, Z=None, text=None):
        GLGraphicsItem.__init__(self)

        self.text = text
        self.X = X
        self.Y = Y
        self.Z = Z

    def setGLViewWidget(self, GLViewWidget):
        self.GLViewWidget = GLViewWidget

    def setText(self, text):
        self.text = text
        self.update()

    def setX(self, X):
        self.X = X
        self.update()

    def setY(self, Y):
        self.Y = Y
        self.update()

    def setZ(self, Z):
        self.Z = Z
        self.update()

    def paint(self):
        self.GLViewWidget.qglColor(QtCore.Qt.white)
        self.GLViewWidget.renderText(self.X, self.Y, self.Z, self.text)


class CrossRoi3D(GLGraphicsItem):
    """
    **Bases:** :class:`GLGraphicsItem <pyqtgraph.opengl.GLGraphicsItem>`

    Displays three lines indicating origin and orientation of local coordinate system.

    """

    def __init__(self, size=None, antialias=True, glOptions='translucent'):
        GLGraphicsItem.__init__(self)
        if size is None:
            size = QtGui.QVector3D(1, 1, 1)
        self.antialias = antialias
        self.setSize(size=size)
        self.setGLOptions(glOptions)

        self.x = 0
        self.y = 0
        self.z = 0

    def setGLViewWidget(self, GLViewWidget):
        self.GLViewWidget = GLViewWidget

    def setSize(self, x=None, y=None, z=None, size=None):
        """
        Set the size of the axes (in its local coordinate system; this does not affect the transform)
        Arguments can be x,y,z or size=QVector3D().
        """
        if size is not None:
            x = size.x()
            y = size.y()
            z = size.z()
        self.__size = [x, y, z]
        self.update()

    def size(self):
        return self.__size[:]

    def set_pos(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.update()

    def paint(self):

        #glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        #glEnable( GL_BLEND )
        #glEnable( GL_ALPHA_TEST )
        self.setupGLState()

        # if self.antialias:
        #     gl.glEnable(gl.GL_LINE_SMOOTH)
        #     gl.glHint(gl.GL_LINE_SMOOTH_HINT, gl.GL_NICEST)
        #
        glBegin(GL_LINES)

        #self.GLViewWidget.qglColor(QtCore.Qt.green)

        x, y, z = [20,20, 20]#self.size()

        rx, ry, rz = self.pos()

        glColor4f(0, 1, 0, .6)  # z is green
        glVertex3f(rx-x, ry, rz)
        glVertex3f(rx+x, ry, rz)

        glColor4f(0, 1, 0, .6)  # y is green
        glVertex3f(rx, ry-y, rz)
        glVertex3f(rx, ry+y, rz)

        glColor4f(0, 1, 0, .6)  # x is green
        glVertex3f(rx, ry, rz-z)
        glVertex3f(rx, ry, rz+z)
        glEnd()

    def pos(self):

        return self.x, self.y, self.z



import pyqtgraph.exporters


class CrossROI(pg.ROI):

    """ Create a Region of interest, which is a zoomable rectangular.

    @param float pos: optional parameter to set the position
    @param float size: optional parameter to set the size of the roi

    Have a look at:
    http://www.pyqtgraph.org/documentation/graphicsItems/roi.html
    """
    sigUserRegionUpdate = QtCore.Signal(object)
    sigMachineRegionUpdate = QtCore.Signal(object)

    def __init__(self, pos, size, **args):
        """Create a ROI with a central handle."""
        self.userDrag = False
        pg.ROI.__init__(self, pos, size, **args)
        # That is a relative position of the small box inside the region of
        # interest, where 0 is the lowest value and 1 is the higherst:
        center = [0.5, 0.5]
        # Translate the center to the intersection point of the crosshair.
        self.addTranslateHandle(center)

        self.sigRegionChangeStarted.connect(self.startUserDrag)
        self.sigRegionChangeFinished.connect(self.stopUserDrag)
        self.sigRegionChanged.connect(self.regionUpdateInfo)


    # def contextMenuEvent(self, event):
    #
    #     newmenu = QtGui.QMenu()
    #
    #     menu = self.scene().addParentContextMenus(self,newmenu, event)
    #
    #     newpoi = menu.addAction("New POI")
    #     action = menu.exec_(event.screenPos())
    #     if action == newpoi:
    #         self.printxy()


        ## Make sure it is still ok to remove this handle
        #removeAllowed = all([r.checkRemoveHandle(self) for r in self.rois])
        #self.removeAction.setEnabled(removeAllowed)
        #
        #menu.popup(QtCore.QPoint(pos.x(), pos.y()))

    #
    #
    # def raiseContextMenu(self, event):
    #
    #     newmenu = QtGui.QMenu()
    #
    #     #newmenu.addAction("New POI", self.printxy)
    #
    #     newmenu.addAction("New POI") #, self.printxy)
    #
    #     menu = self.scene().addParentContextMenus(self,newmenu, event)
    #
    #     pos = event.screenPos()
    #
    #     #menu.popup(QtCore.QPoint(pos.x(), pos.y()))
    #
    #     menu.popup(QtCore.QPoint(0, 0))
    #
    #
    #
    #
    # def mouseClickEvent(self, ev):
    #     ## right-click cancels drag
    #     if ev.button() == QtCore.Qt.RightButton:
    #         self.raiseContextMenu(ev)
    #         self.sigClicked.emit(self, ev)
    #     else:
    #         ev.ignore()
    #
    # def printxy(self):
    #     print(self.pos())

    def setPos(self, pos, update=True, finish=False):
        """Sets the position of the ROI.

        @param bool update: whether to update the display for this call of setPos
        @param bool finish: whether to emit sigRegionChangeFinished

        Changed finish from parent class implementation to not disrupt user dragging detection.
        """
        super().setPos(pos, update=update, finish=finish)

    def setSize(self,size, update=True,finish=True):
        """
        Sets the size of the ROI
        @param bool update: whether to update the display for this call of setPos
        @param bool finish: whether to emit sigRegionChangeFinished
        """
        super().setSize(size,update=update,finish=finish)

    def handleMoveStarted(self):
        """ Handles should always be moved by user."""
        super().handleMoveStarted()
        self.userDrag = True

    def startUserDrag(self, roi):
        """ROI has started being dragged by user."""
        self.userDrag = True

    def stopUserDrag(self, roi):
        """ROI has stopped being dragged by user"""
        self.userDrag = False

    def regionUpdateInfo(self, roi):
        """When the region is being dragged by the user, emit the corresponding signal."""
        if self.userDrag:
            self.sigUserRegionUpdate.emit(roi)
        else:
            self.sigMachineRegionUpdate.emit(roi)


class CrossLine(pg.InfiniteLine):

    """ Construct one line for the Crosshair in the plot.

    @param float pos: optional parameter to set the position
    @param float angle: optional parameter to set the angle of the line
    @param dict pen: Configure the pen.

    For additional options consider the documentation of pyqtgraph.InfiniteLine
    """

    def __init__(self, **args):
        pg.InfiniteLine.__init__(self, **args)
#        self.setPen(QtGui.QPen(QtGui.QColor(255, 0, 255),0.5))

        self.stepper = False

    def adjust(self, extroi):
        """
        Run this function to adjust the position of the Crosshair-Line

        @param object extroi: external roi object from pyqtgraph
        """

        if self.stepper is False:
            self.setPen(palette.green)

        if self.stepper is True:
            self.setPen(palette.blue)

        if self.angle == 0:
            self.setValue(extroi.pos()[1] + extroi.size()[1] * 0.5)
        if self.angle == 90:
            self.setValue(extroi.pos()[0] + extroi.size()[0] * 0.5)


class ConfocalMainWindow(QtWidgets.QMainWindow):

    """ Create the Mainwindow based on the corresponding *.ui file. """

    sigPressKeyBoard = QtCore.Signal(QtCore.QEvent)
    sigDoubleClick = QtCore.Signal()

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_confocalgui.ui')
        self._doubleclicked = False

        # Load it
        super(ConfocalMainWindow, self).__init__()
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


class OptimizerSettingDialog(QtWidgets.QDialog):
    """ User configurable settings for the optimizer embedded in cofocal gui"""

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_optim_settings.ui')

        # Load it
        super(OptimizerSettingDialog, self).__init__()
        uic.loadUi(ui_file, self)


class ConfocalGui(GUIBase):

    """ Main Confocal Class for xy and depth scans.
    """
    _modclass = 'ConfocalGui'
    _modtype = 'gui'

    # declare connectors
    confocallogic1 = Connector(interface='ConfocalLogic')
    savelogic = Connector(interface='SaveLogic')

    g2logic = Connector(interface='g2Logic')
    #optimizerlogic1 = Connector(interface='OptimizerLogic')
    saturationlogic =Connector(interface='saturationLogic')

    tracklogic = Connector(interface='TrackLogic')

    fllogic = Connector(interface='flLogic')


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

    # signals
    sigStartOptimizer = QtCore.Signal(list, str)

    sigStartTracker = QtCore.Signal(list, str)

    sigStartg2 = QtCore.Signal(list, str)


    sigStart3dscan = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initializes all needed UI files and establishes the connectors.

        This method executes the all the inits for the differnt GUIs and passes
        the event argument from fysom to the methods.
        """

        # Getting an access to all connectors:
        self._scanning_logic = self.get_connector('confocallogic1')
        self._save_logic = self.get_connector('savelogic')
        #self._optimizer_logic = self.get_connector('optimizerlogic1')
        self._track_logic = self.get_connector('tracklogic')
        self._g2_logic = self.get_connector('g2logic')
        self._fl_logic = self.get_connector('fllogic')
        self._sat_logic = self.get_connector('saturationlogic')


        self._hardware_state = True

        self.single_clicked = True

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
        self._mw = ConfocalMainWindow()

        ###################################################################
        #               Configuring the dock widgets                      #
        ###################################################################
        # All our gui elements are dockable, and so there should be no "central" widget.
        self._mw.centralwidget.hide()
        self._mw.setDockNestingEnabled(True)

        # always use first channel on startup, can be changed afterwards
        self.xy_channel = 0
        self.depth_channel = 0
        self.opt_channel = 0

        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0


        self._mw.stitch_InputWidget.setValue(self._scanning_logic.stitch_range)

        self._mw.track_res.setValue(self._track_logic.points)

        time_text = '{0:.1f} s'.format(self._track_logic.wait_time)
        self._mw.track_wait.setValue(self._track_logic.wait_time)
        self._mw.track_time.setText(time_text)
        self._mw.track_delta.setValue(self._track_logic.res)
        self._mw.track_Delta.setValue(self._track_logic.res*self._track_logic.points)

        # Get the image data (not position) for the display from the logic (last off list)
        location = self._scanning_logic.xy_image[next(reversed(self._scanning_logic.xy_image))]

        image = location['image']

        raw_data_xy = image[:, :, 3 + self.xy_channel]


        # Move cursor in piezo or stepper range
        self.move_stepper = False

        # Set initial position for the crosshair, default is the middle of the
        # screen:

        ini_pos_z_crosshair = 0

        self.curri = 0


        raw_data_xy = np.zeros(raw_data_xy.shape)

        xy_image = pg.ImageItem(image=np.array(raw_data_xy, copy=True), axisOrder='row-major', opacity=1, )

        xy_image.setPxMode(0)

        xy_image.setRect(
            QtCore.QRectF(
                0 - 2e-5,
                0 - 2e-5,
                2e-5,
                2e-5
            )
        )

        self._mw.xy_ViewWidget.addItem(xy_image, ignoreBounds=True)

        assert xy_image.getViewWidget() is self._mw.xy_ViewWidget

        self.xy_image = [xy_image]

        # Load the images for xy and depth in the display:
        #self.xy_image.append(pg.ImageItem(image=raw_data_xy, axisOrder='row-major'))



        # self.xy_image[self.curri].setRect(
        #     QtCore.QRectF(
        #         self._scanning_logic.image_x_range[0],
        #         self._scanning_logic.image_y_range[0],
        #         2*self._scanning_logic.image_x_range[1],
        #         2*self._scanning_logic.image_y_range[1]
        #     )
        # )

        # Hide tilt correction window
        self._mw.tilt_correction_dockWidget.hide()

        # Add the display item to the xy and depth ViewWidget, which was defined
        # in the UI file:
        #self._mw.xy_ViewWidget.addItem(self.xy_image[self.curri])

        pathgds = os.path.join(get_main_dir(), 'largedm_4corners.gds')

        self._mw.xy_ViewWidget.setAspectLocked(lock=True, ratio=1)

        '''
        # Disabling GDS 6/06/18
     
        crosses = ph.import_gds(filename=pathgds, flatten=True, unit=1)

        polygons_spec = crosses.get_polygons(by_spec=True, depth=None)
        qcolor = QColor()
        qcolor.setNamedColor('#FF9456')
        qcolor.setAlphaF(1)

        qpen = QPen()
        qpen.setWidthF(1e-7)
        qpen.setColor(QColor(255, 255, 255))
        for key in sorted(polygons_spec):
            polygons = polygons_spec[key]
            # layerprop = _get_layerprop(layer = key[0], datatype = key[1])

            for points in polygons:
                qpoly = QGraphicsPolygonItem(QPolygonF([QPointF(p[0], p[1]) for p in points]))
                qpoly.setBrush(qcolor)
                qpoly.setPen(qpen)
                self._mw.xy_ViewWidget.addItem(qpoly, ignoreBounds=True)
            points = [(-2.25e-3, -2.1e-3), (-2.25e-3, -1.5e-3), (-1.5e-3, -1.5e-3), (-1.5e-3, -2.1e-3)]

            self.qpoly = QGraphicsPolygonItem(QPolygonF([QPointF(p[0], p[1]) for p in points]))
            self._mw.xy_ViewWidget.addItem(self.qpoly)
            qcolor.setAlphaF(0)
            self.qpoly.setBrush(qcolor)

            self.qpoly.setPen(qpen)
            
        '''

        # set up scan line plot
        sc = self._scanning_logic._scan_counter
        sc = sc - 1 if sc >= 1 else sc

        # here is some coordinate data
        #data = self._scanning_logic.xy_image[sc, :, 0:4:3]

        #print(data)

        # ###################################################################
        # #               Configuration of the optimizer tab                #
        # ###################################################################
        # # Load the image for the optimizer tab
        # self.xy_refocus_image = pg.ImageItem(
        #     image=self._optimizer_logic.xy_refocus_image[:, :, 3 + self.opt_channel],
        #     axisOrder='row-major')
        # self.xy_refocus_image.setRect(
        #     QtCore.QRectF(
        #         self._optimizer_logic._initial_pos_x - 0.5 * self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic._initial_pos_y - 0.5 * self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic.refocus_XY_size
        #     )
        # )
        # self.depth_refocus_image = pg.PlotDataItem(
        #     x=self._optimizer_logic._zimage_Z_values,
        #     y=self._optimizer_logic.z_refocus_line[:, self._optimizer_logic.opt_channel],
        #     pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
        #     symbol='o',
        #     symbolPen=palette.c1,
        #     symbolBrush=palette.c1,
        #     symbolSize=7
        # )
        # self.depth_refocus_fit_image = pg.PlotDataItem(
        #     x=self._optimizer_logic._fit_zimage_Z_values,
        #     y=self._optimizer_logic.z_fit_data,
        #     pen=pg.mkPen(palette.c2)
        # )

        self.x_track = pg.PlotDataItem(
            x=self._track_logic._x_values,
            y=self._track_logic.x_track_line,
            pen=None,
            stepMode=True, fillLevel=0, brush=(188, 0, 188, 255)
        )

        self.y_track = pg.PlotDataItem(
            x=self._track_logic._y_values,
            y=self._track_logic.y_track_line,
            pen=None,
            stepMode=True, fillLevel=0, brush=(188, 0, 188, 255)
        )

        #self.y_track.setLabel('bottom', 'X position', units='m')

        self.z_track = pg.PlotDataItem(
            x=self._track_logic._z_values,
            y=self._track_logic.z_track_line,
            pen = None,
            stepMode=True, fillLevel=0, brush=(188, 0, 188, 255)
        )



        self.fl_track_c1 = pg.PlotDataItem(
            x=self._fl_logic._x_values,
            y=self._fl_logic.x_track_line_c1,
            pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
            symbol='o',
            symbolPen=palette.c1,
            symbolBrush=palette.c1,
            symbolSize=7
        )
        self.fl_track_c2 = pg.PlotDataItem(
            x=self._fl_logic._x_values,
            y=self._fl_logic.x_track_line_c2,
            pen=pg.mkPen(palette.c2, style=QtCore.Qt.DotLine),
            symbol='o',
            symbolPen=palette.c2,
            symbolBrush=palette.c2,
            symbolSize=7
        )
        self.fl_track_total = pg.PlotDataItem(
            x=self._fl_logic._x_values,
            y=self._fl_logic.x_track_line_total,
            pen=pg.mkPen(palette.c3, style=QtCore.Qt.DotLine),
            symbol='o',
            symbolPen=palette.c3,
            symbolBrush=palette.c3,
            symbolSize=7
        )

        self.g2_track = pg.PlotDataItem(
            x=self._g2_logic._x_values,
            y=self._g2_logic.x_track_line,
            pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
            symbol='o',
            symbolPen=palette.c1,
            symbolBrush=palette.c1,
            symbolSize=7
        )

        self.sat_track = pg.PlotDataItem(
            x=self._sat_logic.power_vector,
            y=self._sat_logic.count_vector,
            pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
            symbol='o',
            symbolPen=palette.c1,
            symbolBrush=palette.c1,
            symbolSize=7
        )

        # self.smoothxarr= pg.PlotDataItem(
        #     x=self._track_logic._x_values,
        #     y=self._track_logic.smooth_x,
        #     pen=pg.mkPen(palette.c2, width = 2))

        self._mw.xtrack_ViewWidget.addItem(self.x_track)

        self._mw.xtrack_ViewWidget.setLabel('bottom', 'x-pos', units='m')
        self._mw.ytrack_ViewWidget.addItem(self.y_track)
        self._mw.ztrack_ViewWidget.addItem(self.z_track)

        self._mw.sat_PlotWidget.addItem(self.sat_track)

        self._mw.FL_plotwidget.addItem(self.fl_track_c1)
        self._mw.FL_plotwidget.addItem(self.fl_track_c2)
        self._mw.FL_plotwidget.addItem(self.fl_track_total)

        self._mw.ytrack_ViewWidget.setLabel('bottom', 'y-pos', units='m')
        self._mw.ztrack_ViewWidget.setLabel('bottom', 'z-pos', units='m')

        self._mw.xstepperadjust.setValue(self._scanning_logic.initial_jog_x_step)
        self._mw.ystepperadjust.setValue(self._scanning_logic.initial_jog_y_step)
        self._mw.zstepperadjust.setValue(self._scanning_logic.initial_jog_z_step)

        self._mw.jogfor_x.clicked.connect(self.jog_x_up_pressed)
        self._mw.jogback_x.clicked.connect(self.jog_x_down_pressed)

        self._mw.jogfor_y.clicked.connect(self.jog_y_up_pressed)
        self._mw.jogback_y.clicked.connect(self.jog_y_down_pressed)

        self._mw.jogfor_z.clicked.connect(self.jog_z_up_pressed)
        self._mw.jogback_z.clicked.connect(self.jog_z_down_pressed)

        self._mw.poi_add_PushButton.clicked.connect(self.poi_add_clicked)
        #self._mw.poi_del_PushButton.clicked.connect(self.poi_del_clicked)


        #self._mw.xtrack_ViewWidget.addItem(self.smoothxarr)

        self._mw.startpowerBox.setValue(self._sat_logic.power_min)
        self._mw.endpowerBox.setValue(self._sat_logic.power_max)
        self._mw.satpointsBox.setValue(self._sat_logic.points)
        self._mw.sattimeBox.setValue(self._sat_logic.int_time)

        self._mw.g2_PlotWidget.addItem(self.g2_track)


        self._mw.psf_xy.setValue(self._scanning_logic.psf_deltaxy)
        self._mw.psf_z.setValue(self._scanning_logic.psf_deltaz)
        self._mw.psf_res.setValue(self._scanning_logic.psf_deltares)

        #return


        self._mw.PSF_volume.setBackgroundColor(48,47,47)


        #g = gl.GLGridItem()
        #g.scale(10, 10, 1)
        #self._mw.PSF_volume.addItem(g)

        #ax = gl.GLAxisItem()

        #g = gl.GLGridItem()
        #g.scale(2, 2, 1)

        ## create three grids, add each to the view
        #xgrid = gl.GLGridItem()
        #ygrid = gl.GLGridItem()
        #zgrid = gl.GLGridItem()



        #self._mw.PSF_volume.addItem(xgrid)
        #self._mw.PSF_volume.addItem(ygrid)
        #self._mw.PSF_volume.addItem(zgrid)

        #xgrid.scale(16, 16, 16)
        #ygrid.scale(100, 100, 100)
        #xgrid.scale(100, 100, 100)

        ## rotate x and y grids to face the correct direction
        #xgrid.rotate(90, 0, 1, 0)
        #ygrid.rotate(90, 1, 0, 0)

        #self._mw.PSF_volume.pan(100, 100, 150)


        self._mw.psf_start.clicked.connect(self.psf_start_clicked)

        self._mw.psf_start.setCheckable(True)
        self._mw.psf_start.setChecked(False)

        self._mw.PSF_volume.orbit(-45,15)

        self._mw.PSF_volume.orbit(-90, 45)

        xres = int(self._scanning_logic.psf_deltaxy / self._scanning_logic.psf_deltares)
        zres = int(self._scanning_logic.psf_deltaz / self._scanning_logic.psf_deltares)

        #self._mw.PSF_volume.pan(xres/2, xres*1.5, xres*1.5)

        #self._mw.PSF_volume.

        #self.psfdata = gl.GLVolumeItem(self._scanning_logic.psf_data, smooth = True, glOptions='translucent')

        #self._mw.PSF_volume.addItem(self.psfdata)

        #ax = gl.GLAxisItem()



        self.psf_data = np.ones((self._scanning_logic.psf_xyres , self._scanning_logic.psf_xyres,
                        self._scanning_logic.psf_zres))

        self.my_colors_a = ColorScaleInfernoAlpha()

        self.my_colors_agl = ColorScaleInfernoAlphaGL()

        # Convert this data into the colourmap

        # Get cb range
        [cb_min, cb_max] = [0, 20] #self.get_3d_cb_range()

        scale = np.linspace(cb_min, cb_max, 2000)

        idxrange = np.arange(0, 2000,1)

        cube = self._scanning_logic.psf_xyres*np.ones((self._scanning_logic.psf_xyres * 4, self._scanning_logic.psf_xyres * 4,
                        self._scanning_logic.psf_zres * 4, 4), dtype=np.ubyte)

        for (i, j, k), element in np.ndenumerate(self.psf_data):
            idx = int(np.interp(element, idxrange, scale))
            #print('element',element)
            #print('range', range)
            #print('scale', scale)
            #print('idx',idx)
            [r,g,b,a] = self.my_colors_a.lut[idx] #/ 256
            #print('RGBA',RGBA)
            for x in range(0, 4):
                for y in range(0, 4):
                    for z in range(0, 4):
                        cube[4 * i + x, 4 * j + y, 4 * k + z, :] =[r, g , b, a/16] #[256*r,256*g,256*b,a/16]

        self.psf_render = gl.GLVolumeItem(cube,smooth = True, glOptions='translucent')

        self.psf_text_xmin =  GLTextItem(X=0, Y=-3, Z=-1, text="1")
        self.psf_text_xmax = GLTextItem(X=xres, Y=-3, Z=-1, text="0")

        self.psf_text_ymin = GLTextItem(X=-3, Y=0, Z=-1, text="0")
        self.psf_text_ymax = GLTextItem(X=-3, Y=xres, Z=-1, text="1")

        self.psf_text_xmin.setGLViewWidget(self._mw.PSF_volume)
        self.psf_text_xmax.setGLViewWidget(self._mw.PSF_volume)
        self.psf_text_ymin.setGLViewWidget(self._mw.PSF_volume)
        self.psf_text_ymax.setGLViewWidget(self._mw.PSF_volume)


        self.crosshair3D = CrossRoi3D()
        self.crosshair3D.setGLViewWidget(self._mw.PSF_volume)



        self.psf_xygrid = gl.GLGridItem()


        self._mw.PSF_volume.addItem(self.psf_render)

        self._mw.PSF_volume.addItem(self.crosshair3D)

        self._mw.PSF_volume.addItem(self.psf_text_xmin)
        self._mw.PSF_volume.addItem(self.psf_text_ymin)
        self._mw.PSF_volume.addItem(self.psf_text_xmax)
        self._mw.PSF_volume.addItem(self.psf_text_ymax)

        #self._mw.PSF_volume.addItem(self.psf_xygrid) # change this to be two axes


        #
        #
        # def psi(i, j, k, offset=(50, 50, 100)):
        #     x = i - offset[0]
        #     y = j - offset[1]
        #     z = k - offset[2]
        #     th = np.arctan2(z, (x ** 2 + y ** 2) ** 0.5)
        #     phi = np.arctan2(y, x)
        #     r = (x ** 2 + y ** 2 + (z/2) ** 2) ** 0.5
        #     a0 = 1000
        #     # ps = (1./81.) * (2./np.pi)**0.5 * (1./a0)**(3/2) * (6 - r/a0) * (r/a0) * np.exp(-r/(3*a0)) * np.cos(th)
        #     ps =  (r / a0) ** 2 * np.exp(
        #         -r / (3 * a0))
        #
        #     return ps
        #
        #     # return ((1./81.) * (1./np.pi)**0.5 * (1./a0)**(3/2) * (r/a0)**2 * (r/a0) * np.exp(-r/(3*a0)) * np.sin(th) * np.cos(th) * np.exp(2 * 1j * phi))**2
        #
        # data = np.fromfunction(psi, (100, 100, 200))
        # #print(np.clip(data, 1e-6, data.max()) ** 2)
        # positive = np.log(np.clip(data, 1e-6, data.max()) ** 2)
        # #negative = np.log(np.clip(-data, 0, -data.min()) ** 2)
        #
        # d2 = np.empty(data.shape + (4,), dtype=np.ubyte)
        # d2[..., 0] = positive * (255. / positive.max())
        # #d2[..., 1] = negative * (255. / negative.max())
        # d2[..., 2] = d2[..., 1]
        # d2[..., 3] = d2[..., 0] * 0.3 + d2[..., 1] * 0.3
        # d2[..., 3] = (d2[..., 3].astype(float) / 255.) ** 2 * 255
        #
        # d2[:, 0, 0] = [255, 0, 0, 100]
        # d2[0, :, 0] = [0, 255, 0, 100]
        # d2[0, 0, :] = [0, 0, 255, 100]





        #self.psf_cb = ColorBar(self.my_colors.cmap_normed, width=100, cb_min=0, cb_max=100)
        # self.depth_cb = ColorBar(self.my_colors.cmap_normed, width=100, cb_min=0, cb_max=100)
        #self._mw.PSF_volume.addItem(self.psf_cb)
        #self._mw.xy_cb_ViewWidget.hideAxis('bottom')
        #self._mw.xy_cb_ViewWidget.setLabel('left', 'Fluorescence', units='c/s')
        #self._mw.xy_cb_ViewWidget.setMouseEnabled(x=False, y=False)

        # Add the display item to the xy and depth VieWidget, which was defined in
        # the UI file.
        # self._mw.xy_refocus_ViewWidget_2.addItem(self.xy_refocus_image)
        # self._mw.depth_refocus_ViewWidget_2.addItem(self.depth_refocus_image)
        #
        # # Labelling axes
        # self._mw.xy_refocus_ViewWidget_2.setLabel('bottom', 'X position', units='m')
        # self._mw.xy_refocus_ViewWidget_2.setLabel('left', 'Y position', units='m')
        #
        # self._mw.depth_refocus_ViewWidget_2.addItem(self.depth_refocus_fit_image)
        #
        # self._mw.depth_refocus_ViewWidget_2.setLabel('bottom', 'Z position', units='m')
        # self._mw.depth_refocus_ViewWidget_2.setLabel('left', 'Fluorescence', units='c/s')
        #
        # # Add crosshair to the xy refocus scan
        # self.vLine = pg.InfiniteLine(
        #     pen=QtGui.QPen(
        #         palette.green,
        #         self._optimizer_logic.refocus_XY_size / 50),
        #     pos=50,
        #     angle=90,
        #     movable=False)
        # self.hLine = pg.InfiniteLine(
        #     pen=QtGui.QPen(
        #         palette.green,
        #         self._optimizer_logic.refocus_XY_size / 50),
        #     pos=50,
        #     angle=0,
        #     movable=False)
        # self._mw.xy_refocus_ViewWidget_2.addItem(self.vLine, ignoreBounds=True)
        # self._mw.xy_refocus_ViewWidget_2.addItem(self.hLine, ignoreBounds=True)

        # Set the state button as ready button as default setting.
        self._mw.action_stop_scanning.setEnabled(False)
        self._mw.action_scan_xy_resume.setEnabled(False)
        self._mw.action_scan_depth_resume.setEnabled(False)





        # Label the axes:
        self._mw.xy_ViewWidget.setLabel('bottom', 'X position', units='m')
        self._mw.xy_ViewWidget.setLabel('left', 'Y position', units='m')

        ini_pos_x_crosshair = -20e-6 # 0-2.0455  # e-3 #len(raw_data_xy) / 2
        ini_pos_y_crosshair = -20e-6 #-2.1750  # e-3 #len(raw_data_xy) / 2

        # Create Region of Interest for xy image and add to xy Image Widget:
        self.roi_xy = CrossROI(
            [
                ini_pos_x_crosshair,
                ini_pos_y_crosshair
            ],
            [20e-5, 20e-5],
            pen={'color': "F0F", 'width': 1},
            removable=False
        )

        self._mw.xy_ViewWidget.addItem(self.roi_xy)

        # create horizontal and vertical line as a crosshair in xy image:
        self.hline_xy = CrossLine(pos=self.roi_xy.pos() + self.roi_xy.size() * 0.5,
                                  angle=0, pen={'color': palette.green, 'width': 1})
        self.vline_xy = CrossLine(pos=self.roi_xy.pos() + self.roi_xy.size() * 0.5,
                                  angle=90, pen={'color': palette.green, 'width': 1})

        # connect the change of a region with the adjustment of the crosshair:
        self.roi_xy.sigRegionChanged.connect(self.hline_xy.adjust)
        self.roi_xy.sigRegionChanged.connect(self.vline_xy.adjust)
        self.roi_xy.sigUserRegionUpdate.connect(self.update_from_roi_xy)
        self.roi_xy.sigRegionChangeFinished.connect(self.roi_xy_bounds_check)

        # add the configured crosshair to the xy Widget
        self._mw.xy_ViewWidget.addItem(self.hline_xy)
        self._mw.xy_ViewWidget.addItem(self.vline_xy)

        # Set up and connect xy channel combobox
        scan_channels = self._scanning_logic.get_scanner_count_channels()
        for n, ch in enumerate(scan_channels):
            self._mw.xy_channel_ComboBox.addItem(str(ch), n)

        self._mw.xy_channel_ComboBox.activated.connect(self.update_xy_channel)


        # Dict for the NV candidates
        self.roi_dict = []


        # Set up and connect depth channel combobox
        scan_channels = self._scanning_logic.get_scanner_count_channels()

        # Setup the Sliders:
        # Calculate the needed Range for the sliders. The image ranges comming
        # from the Logic module must be in meters.
        # 1 nanometer resolution per one change, units are meters
        self.slider_res = 20e-6
        self.sliderp_res = 1e-9

        # How many points are needed for that kind of resolution:
        num_of_points_x = (self._scanning_logic.x_range[1] - self._scanning_logic.x_range[0]) / self.slider_res
        num_of_points_y = (self._scanning_logic.y_range[1] - self._scanning_logic.y_range[0]) / self.slider_res

        num_of_points_xp = (self._scanning_logic.spx_x_range[1] - self._scanning_logic.spx_x_range[0]) / self.sliderp_res
        num_of_points_yp = (self._scanning_logic.spx_y_range[1] - self._scanning_logic.spx_y_range[0]) / self.sliderp_res


        num_of_points_z = (self._scanning_logic.z_range[1] - self._scanning_logic.z_range[0]) / self.sliderp_res

        # Set a Range for the sliders:
        self._mw.x_SliderWidget.setRange(0, num_of_points_x)
        self._mw.y_SliderWidget.setRange(0, num_of_points_y)

        self._mw.xp_SliderWidget.setRange(0, num_of_points_xp)
        self._mw.yp_SliderWidget.setRange(0, num_of_points_yp)

        self._mw.zp_SliderWidget.setRange(0, num_of_points_yp)

        self._mw.z_SliderWidget.setRange(0, num_of_points_y)

        # Just to be sure, set also the possible maximal values for the spin
        # boxes of the current values:
        self._mw.x_current_InputWidget.setRange(self._scanning_logic.x_range[0], self._scanning_logic.x_range[1])
        self._mw.y_current_InputWidget.setRange(self._scanning_logic.y_range[0], self._scanning_logic.y_range[1])
        self._mw.z_current_InputWidget.setRange(self._scanning_logic.y_range[0], self._scanning_logic.y_range[1])
        #print('spx range')
        #print(self._scanning_logic.spx_x_range[0], self._scanning_logic.spx_x_range[1])

        self._mw.xp_current_InputWidget.setRange(self._scanning_logic.piezo_range[0], self._scanning_logic.piezo_range[1])
        self._mw.yp_current_InputWidget.setRange(self._scanning_logic.piezo_range[0], self._scanning_logic.piezo_range[1])
        self._mw.zp_current_InputWidget.setRange(self._scanning_logic.piezo_range[0], self._scanning_logic.piezo_range[1])

        # Set range for resolution, hardcoded for now JS 29/04

        self._mw.spx_size_InputWidget.setRange(30,160)


        # set minimal steps for the current value
        self._mw.x_current_InputWidget.setMinimalStep(10e-6)
        self._mw.y_current_InputWidget.setMinimalStep(10e-6)
        self._mw.z_current_InputWidget.setMinimalStep(10e-6)

        # Predefine the maximal and minimal image range as the default values
        # for the display of the range:
        #self._mw.x_min_InputWidget.setValue(self._scanning_logic.scan_x_range[0])
        self._mw.x_max_InputWidget.setValue(self._scanning_logic.scan_x_range[1])
        #self._mw.y_min_InputWidget.setValue(self._scanning_logic.scan_y_range[0])
        #self._mw.y_max_InputWidget.setValue(self._scanning_logic.scan_y_range[1])

        self._mw.spx_size_InputWidget.setValue(self._scanning_logic.spx_size)
        self._mw.spx_overlap_InputWidget.setValue(self._scanning_logic.spx_overlap)

        # set the maximal ranges for the imagerange from the logic:
        #self._mw.x_min_InputWidget.setRange(self._scanning_logic.x_range[0], self._scanning_logic.x_range[1])
        #self._mw.x_max_InputWidget.setRange(self._scanning_logic.x_range[0], self._scanning_logic.x_range[1])
        #self._mw.y_min_InputWidget.setRange(self._scanning_logic.y_range[0], self._scanning_logic.y_range[1])
        #self._mw.y_max_InputWidget.setRange(self._scanning_logic.y_range[0], self._scanning_logic.y_range[1])


        self._mw.track_res.setRange(1, 300)

        # set the minimal step size
        #self._mw.x_min_InputWidget.setOpts(minStep=1e-9)
        self._mw.x_max_InputWidget.setMinimalStep(1e-9)
        #self._mw.y_min_InputWidget.setOpts(minStep=1e-9)
        #self._mw.y_max_InputWidget.setOpts(minStep=1e-9)


        # Handle slider movements by user:
        self._mw.x_SliderWidget.sliderMoved.connect(self.update_from_slider_x)
        self._mw.y_SliderWidget.sliderMoved.connect(self.update_from_slider_y)
        self._mw.z_SliderWidget.sliderMoved.connect(self.update_from_slider_z)
        self._mw.xp_SliderWidget.sliderMoved.connect(self.update_from_slider_xp)
        self._mw.yp_SliderWidget.sliderMoved.connect(self.update_from_slider_yp)
        self._mw.zp_SliderWidget.sliderMoved.connect(self.update_from_slider_zp)



        # Update the inputed/displayed numbers if the cursor has left the field:
        self._mw.x_current_InputWidget.editingFinished.connect(self.update_from_input_x)
        self._mw.y_current_InputWidget.editingFinished.connect(self.update_from_input_y)
        self._mw.z_current_InputWidget.editingFinished.connect(self.update_from_input_z)

        self._mw.xp_current_InputWidget.editingFinished.connect(self.update_from_input_xp)
        self._mw.yp_current_InputWidget.editingFinished.connect(self.update_from_input_yp)
        self._mw.zp_current_InputWidget.editingFinished.connect(self.update_from_input_zp)

        self._mw.spx_size_InputWidget.editingFinished.connect(self.update_from_input_spx_size)
        self._mw.spx_overlap_InputWidget.editingFinished.connect(self.update_from_input_spx_overlap)

        self._mw.stitch_InputWidget.editingFinished.connect(self.update_from_input_stitch_range)

        self._mw.track_res.editingFinished.connect(self.update_from_input_track_res)
        self._mw.track_delta.editingFinished.connect(self.update_from_input_track_delta)
        self._mw.track_Delta.editingFinished.connect(self.update_from_input_track_delta2)

        self._mw.track_av.editingFinished.connect(self.update_from_input_track_av)
        self._mw.track_wait.editingFinished.connect(self.update_from_input_track_wait)

        # PSF input widgets
        self._mw.psf_res.editingFinished.connect(self.update_from_input_psf)
        self._mw.psf_xy.editingFinished.connect(self.update_from_input_psf)
        self._mw.psf_z.editingFinished.connect(self.update_from_input_psf)


        #self._mw.x_min_InputWidget.editingFinished.connect(self.change_x_range)
        self._mw.x_max_InputWidget.editingFinished.connect(self.change_x_range)
        #self._mw.y_min_InputWidget.editingFinished.connect(self.change_y_range)
        #self._mw.y_max_InputWidget.editingFinished.connect(self.change_y_range)


        self._mw.xstepperadjust.editingFinished.connect(self.update_from_x_stepper_adjust)
        self._mw.ystepperadjust.editingFinished.connect(self.update_from_y_stepper_adjust)
        self._mw.zstepperadjust.editingFinished.connect(self.update_from_z_stepper_adjust)





        # Connect the change of the viewed area to an adjustment of the ROI:
        self.adjust_cursor_roi = True
        #self.qpoly.shape().sigRangeChanged.connect(self.update_roi_xy_size)



        #################################################################
        #                           Actions                             #
        #################################################################
        # Connect the scan actions to the events if they are clicked. Connect
        # also the adjustment of the displayed windows.
        self._mw.action_stop_scanning.triggered.connect(self.ready_clicked)
        self._mw.actionZero_Stepper.triggered.connect(self.zero_stepper)
        self._mw.moveMotor.clicked.connect(self.move_motor_button_press)

        self._mw.takeFLpx.clicked.connect(self.update_fl_from_cursor)


        self._scan_xy_start_proxy = pg.SignalProxy(
            self._mw.action_scan_xy_start.triggered,
            delay=0.1,
            slot=self.xy_scan_clicked
            )
        self._scan_xy_resume_proxy =  pg.SignalProxy(
            self._mw.action_scan_xy_resume.triggered,
            delay=0.1,
            slot=self.continue_xy_scan_clicked
            )


        # self._optimize_position_proxy = pg.SignalProxy(
        #     self._mw.action_optimize_position.triggered,
        #     delay=0.1,
        #     slot=self.refocus_clicked
        #     )

        self._track_position_proxy = pg.SignalProxy(
            self._mw.actionTracking.triggered,
            delay=0.1,
            slot=self.track_clicked
        )

        # history actions
        self._mw.actionForward.triggered.connect(self._scanning_logic.history_forward)
        self._mw.actionBack.triggered.connect(self._scanning_logic.history_back)
        self._scanning_logic.signal_history_event.connect(lambda: self.set_history_actions(True))
        self._scanning_logic.signal_history_event.connect(self.update_xy_cb_range)
        #self._scanning_logic.signal_history_event.connect(self.update_depth_cb_range)
        self._scanning_logic.signal_history_event.connect(self._mw.xy_ViewWidget.autoRange)
        self._scanning_logic.signal_history_event.connect(self.reset_xy_imagerange)


        # Get initial tilt correction values
        self._mw.action_TiltCorrection.setChecked(
            self._scanning_logic._scanning_device.tiltcorrection)

        self._mw.tilt_01_x_pos_doubleSpinBox.setValue(self._scanning_logic.point1[0])
        self._mw.tilt_01_y_pos_doubleSpinBox.setValue(self._scanning_logic.point1[1])
        self._mw.tilt_01_z_pos_doubleSpinBox.setValue(self._scanning_logic.point1[2])

        self._mw.tilt_02_x_pos_doubleSpinBox.setValue(self._scanning_logic.point2[0])
        self._mw.tilt_02_y_pos_doubleSpinBox.setValue(self._scanning_logic.point2[1])
        self._mw.tilt_02_z_pos_doubleSpinBox.setValue(self._scanning_logic.point2[2])

        self._mw.tilt_03_x_pos_doubleSpinBox.setValue(self._scanning_logic.point3[0])
        self._mw.tilt_03_y_pos_doubleSpinBox.setValue(self._scanning_logic.point3[1])
        self._mw.tilt_03_z_pos_doubleSpinBox.setValue(self._scanning_logic.point3[2])

        # Connect tilt correction buttons
        self._mw.action_TiltCorrection.triggered.connect(self._scanning_logic.set_tilt_correction)
        self._mw.tilt_set_01_pushButton.clicked.connect(self._scanning_logic.set_tilt_point1)
        self._mw.tilt_set_02_pushButton.clicked.connect(self._scanning_logic.set_tilt_point2)
        self._mw.tilt_set_03_pushButton.clicked.connect(self._scanning_logic.set_tilt_point3)
        self._mw.calc_tilt_pushButton.clicked.connect(self._scanning_logic.calc_tilt_correction)
        self._scanning_logic.signal_tilt_correction_update.connect(self.update_tilt_correction)
        self._scanning_logic.signal_tilt_correction_active.connect(
            self._mw.action_TiltCorrection.setChecked)

        self._mw.satButton.clicked.connect(self.update_sat)
        self._mw.satkillButton.clicked.connect(self.kill_sat)

        #refresh_z
        self._mw.dynamiczButton.clicked.connect(self.dynamic_z)
        self._mw.refreshzButton.clicked.connect(self.refresh_z)

        self._mw.startG2.clicked.connect(self._g2_logic.start_refocus)
        self._mw.stopG2.clicked.connect(self._g2_logic.stop_refocus)
        self._mw.resetG2.clicked.connect(self._g2_logic.clear_g2)

        self._mw.startFL.clicked.connect(self._fl_logic.start_refocus)
        self._mw.stopFL.clicked.connect(self._fl_logic.stop_refocus)
        self._mw.resetFL.clicked.connect(self._fl_logic.clear)

        self._mw.singleScan.clicked.connect(self.scan_single)

        self._mw.singleScan.setText('Stitching is off')


        # Connect the default view action
        self._mw.restore_default_view_Action.triggered.connect(self.restore_default_view)
        #self._mw.optimizer_only_view_Action.triggered.connect(self.small_optimizer_view)
        self._mw.actionAutoRange_xy.triggered.connect(self._mw.xy_ViewWidget.autoRange)

        # Connect the buttons and inputs for the xy colorbar
        self._mw.xy_cb_manual_RadioButton.clicked.connect(self.update_xy_cb_range)
        self._mw.xy_cb_centiles_RadioButton.clicked.connect(self.update_xy_cb_range)

        self._mw.xy_cb_min_DoubleSpinBox.valueChanged.connect(self.shortcut_to_xy_cb_manual)
        self._mw.xy_cb_max_DoubleSpinBox.valueChanged.connect(self.shortcut_to_xy_cb_manual)
        self._mw.xy_cb_low_percentile_DoubleSpinBox.valueChanged.connect(self.shortcut_to_xy_cb_centiles)
        self._mw.xy_cb_high_percentile_DoubleSpinBox.valueChanged.connect(self.shortcut_to_xy_cb_centiles)

        self._mw.xy_cb_manual_RadioButton_2.clicked.connect(self.update_3d_cb_range)
        self._mw.xy_cb_centiles_RadioButton_2.clicked.connect(self.update_3d_cb_range)

        self._mw.xy_cb_min_DoubleSpinBox_2.valueChanged.connect(self.shortcut_to_3d_cb_manual)
        self._mw.xy_cb_max_DoubleSpinBox_2.valueChanged.connect(self.shortcut_to_3d_cb_manual)
        self._mw.xy_cb_low_percentile_DoubleSpinBox_2.valueChanged.connect(self.shortcut_to_3d_cb_centiles)
        self._mw.xy_cb_high_percentile_DoubleSpinBox_2.valueChanged.connect(self.shortcut_to_3d_cb_centiles)

        # Connect the buttons and inputs for the depth colorbars
        # RadioButtons in Main tab
        #self._mw.depth_cb_manual_RadioButton.clicked.connect(self.update_depth_cb_range)
       # self._mw.depth_cb_centiles_RadioButton.clicked.connect(self.update_depth_cb_range)

        # input edits in Main tab
        #self._mw.depth_cb_min_DoubleSpinBox.valueChanged.connect(self.shortcut_to_depth_cb_manual)
        #self._mw.depth_cb_max_DoubleSpinBox.valueChanged.connect(self.shortcut_to_depth_cb_manual)
        #self._mw.depth_cb_low_percentile_DoubleSpinBox.valueChanged.connect(self.shortcut_to_depth_cb_centiles)
        #self._mw.depth_cb_high_percentile_DoubleSpinBox.valueChanged.connect(self.shortcut_to_depth_cb_centiles)

        # Connect the emitted signal of an image change from the logic with
        # a refresh of the GUI picture:
        self._scanning_logic.signal_xy_image_updated.connect(self.refresh_xy_image)
        self._scanning_logic.signal_xy_image_updated.connect(self.refresh_scan_line)

        self._scanning_logic.signal_spx_next.connect(self.nextscanMotor)

        self._scanning_logic.signal_3d_next.connect(self.drawPSF)

        #self._optimizer_logic.sigImageUpdated.connect(self.refresh_refocus_image)

        self._track_logic.sigImageUpdated.connect(self.refresh_tracking)

        # We call the below function in the refresh_tracking function instead
        #self._track_logic.sigImageUpdated.connect(self.update_crosshair_position_from_logic)


        self._track_logic.sigTimeUpdate.connect(self.update_track_timer)

        self._g2_logic.sigImageUpdated.connect(self.refresh_g2)
        self._fl_logic.sigImageUpdated.connect(self.refresh_fl)


        self._sat_logic.sigImageUpdated.connect(self.refresh_saturation)

        #self._scanning_logic.sigImageXYInitialized.connect(self.adjust_xy_window)

        # Connect the signal from the logic with an update of the cursor position
        self._scanning_logic.signal_change_position.connect(self.update_crosshair_position_from_logic)

        # Connect other signals from the logic with an update of the gui

        self._scanning_logic.signal_start_scanning.connect(self.logic_started_scanning)
        self._scanning_logic.signal_continue_scanning.connect(self.logic_continued_scanning)
        #self._optimizer_logic.sigRefocusStarted.connect(self.logic_started_refocus)
        #self._scanning_logic.signal_stop_scanning.connect()

        # Connect the tracker
        #self.sigStartOptimizer.connect(self._optimizer_logic.start_refocus)
        #self._optimizer_logic.sigRefocusFinished.connect(self._refocus_finished_wrapper)
        #self._optimizer_logic.sigRefocusXySizeChanged.connect(self.update_roi_xy_size)

        self.sigStartTracker.connect(self._track_logic.start_refocus)

        self.sigStartg2.connect(self._g2_logic.start_refocus)


        self.sigStart3dscan.connect(self._scanning_logic.scan_3d_start)

        self.n_checker = 0


        # Connect the 'File' Menu dialog and the Settings window in confocal
        # with the methods:
        self._mw.action_Settings.triggered.connect(self.menu_settings)
        #self._mw.action_optimizer_settings.triggered.connect(self.menu_optimizer_settings)
        self._mw.actionSave_XY_Scan.triggered.connect(self.save_xy_scan_data)


        # Configure and connect the zoom actions with the desired buttons and
        # functions if
        self._mw.action_full_range_xy.triggered.connect(self.set_full_scan_range_xy)
        self._mw.action_full_range_z.triggered.connect(self.set_full_scan_range_z)

        self._mw.action_zoom.toggled.connect(self.zoom_clicked)
        self._mw.sigDoubleClick.connect(self.activate_zoom_double_click)
        self._mw.xy_ViewWidget.sigMouseClick.connect(self.xy_scan_start_zoom_point)
        self._mw.xy_ViewWidget.sigMouseReleased.connect(self.xy_scan_end_zoom_point)




        ###################################################################
        #               Icons for the scan actions                        #
        ###################################################################

        self._scan_xy_single_icon = QtGui.QIcon()
        self._scan_xy_single_icon.addPixmap(
            QtGui.QPixmap("artwork/icons/qudiTheme/22x22/scan-xy-start.png"),
            QtGui.QIcon.Normal,
            QtGui.QIcon.Off)




        self._scan_xy_loop_icon = QtGui.QIcon()
        self._scan_xy_loop_icon.addPixmap(
            QtGui.QPixmap("artwork/icons/qudiTheme/22x22/scan-xy-loop.png"),
            QtGui.QIcon.Normal,
            QtGui.QIcon.Off)

        #################################################################
        #           Connect the colorbar and their actions              #
        #################################################################
        # Get the colorscale and set the LUTs
        self.my_colors = ColorScaleInferno()

        #self.xy_image[self.curri].setLookupTable(self.my_colors.lut)


        # Create colorbars and add them at the desired place in the GUI. Add
        # also units to the colorbar.

        self.xy_cb = ColorBar(self.my_colors.cmap_normed, width=100, cb_min=0, cb_max=100)
        #self.depth_cb = ColorBar(self.my_colors.cmap_normed, width=100, cb_min=0, cb_max=100)
        self._mw.xy_cb_ViewWidget.addItem(self.xy_cb)
        self._mw.xy_cb_ViewWidget.hideAxis('bottom')
        self._mw.xy_cb_ViewWidget.setLabel('left', 'Fluorescence', units='c/s')
        self._mw.xy_cb_ViewWidget.setMouseEnabled(x=False, y=False)


        #We will set a cut off of opacity at some value

        self.my_colors_a = ColorScaleInfernoAlpha()


        self.xy_cb_2 = ColorBar(self.my_colors_a.cmap_normed, width=100, cb_min=0, cb_max=100)
        # self.depth_cb = ColorBar(self.my_colors.cmap_normed, width=100, cb_min=0, cb_max=100)
        self._mw.xy_cb_ViewWidget_2.addItem(self.xy_cb_2)
        self._mw.xy_cb_ViewWidget_2.hideAxis('bottom')
        self._mw.xy_cb_ViewWidget_2.setLabel('left', 'Fluorescence', units='c/s')
        self._mw.xy_cb_ViewWidget_2.setMouseEnabled(x=False, y=False)






        # self._mw.depth_cb_ViewWidget.addItem(self.depth_cb)
        # self._mw.depth_cb_ViewWidget.hideAxis('bottom')
        # self._mw.depth_cb_ViewWidget.setLabel('left', 'Fluorescence', units='c/s')
        # self._mw.depth_cb_ViewWidget.setMouseEnabled(x=False, y=False)

        self._mw.sigPressKeyBoard.connect(self.keyPressEvent)

        # Now that the ROI for xy and depth is connected to events, update the
        # default position and initialize the position of the crosshair and
        # all other components:
        self.enable_scan_actions()
        self.update_crosshair_position_from_logic('init')
        self.adjust_xy_window()


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

    # def initOptimizerSettingsUI(self):
    #     """ Definition, configuration and initialisation of the optimizer settings GUI.
    #
    #     This init connects all the graphic modules, which were created in the
    #     *.ui file and configures the event handling between the modules.
    #     Moreover it sets default values if not existed in the logic modules.
    #     """
    #     # Create the Settings window
    #     self._osd = OptimizerSettingDialog()
    #     # Connect the action of the settings window with the code:
    #     self._osd.accepted.connect(self.update_optimizer_settings)
    #     self._osd.rejected.connect(self.keep_former_optimizer_settings)
    #     self._osd.buttonBox.button(QtWidgets.QDialogButtonBox.Apply).clicked.connect(self.update_optimizer_settings)
    #
    #     # Set up and connect xy channel combobox
    #     scan_channels = self._optimizer_logic.get_scanner_count_channels()
    #     for n, ch in enumerate(scan_channels):
    #         self._osd.opt_channel_ComboBox.addItem(str(ch), n)
    #
    #     # Generation of the fit params tab ##################
    #     self._osd.fit_tab = FitParametersWidget(self._optimizer_logic.z_params)
    #     self._osd.settings_tabWidget.addTab(self._osd.fit_tab, "Fit Params")
    #
    #     # write the configuration to the settings window of the GUI.
    #     self.keep_former_optimizer_settings()

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

    def keyPressEvent(self, event):
        """ Handles the passed keyboard events from the main window.

        @param object event: qtpy.QtCore.QEvent object.
        """
        modifiers = QtWidgets.QApplication.keyboardModifiers()

        position = self._scanning_logic.get_position()   # in meters
        x_pos = position[0]
        y_pos = position[1]
        z_pos = position[2]

        if modifiers == QtCore.Qt.ControlModifier:
            if event.key() == QtCore.Qt.Key_M:
                self.hline_xy.stepper = not self.hline_xy.stepper
                self.vline_xy.stepper = not self.vline_xy.stepper
            if event.key() == QtCore.Qt.Key_Right:
                self.update_from_key(x=float(round(x_pos + self.slider_big_step, 10)))
            elif event.key() == QtCore.Qt.Key_Left:
                self.update_from_key(x=float(round(x_pos - self.slider_big_step, 10)))
            elif event.key() == QtCore.Qt.Key_Up:
                self.update_from_key(y=float(round(y_pos + self.slider_big_step, 10)))
            elif event.key() == QtCore.Qt.Key_Down:
                self.update_from_key(y=float(round(y_pos - self.slider_big_step, 10)))
            elif event.key() == QtCore.Qt.Key_PageUp:
                self.update_from_key(z=float(round(z_pos + self.slider_big_step, 10)))
            elif event.key() == QtCore.Qt.Key_PageDown:
                self.update_from_key(z=float(round(z_pos - self.slider_big_step, 10)))
            else:
                event.ignore()
        else:
            if event.key() == QtCore.Qt.Key_Right:
                self.update_from_key(x=float(round(x_pos + self.slider_small_step, 10)))
            elif event.key() == QtCore.Qt.Key_Left:
                self.update_from_key(x=float(round(x_pos - self.slider_small_step, 10)))
            elif event.key() == QtCore.Qt.Key_Up:
                self.update_from_key(y=float(round(y_pos + self.slider_small_step, 10)))
            elif event.key() == QtCore.Qt.Key_Down:
                self.update_from_key(y=float(round(y_pos - self.slider_small_step, 10)))
            elif event.key() == QtCore.Qt.Key_PageUp:
                self.update_from_key(z=float(round(z_pos + self.slider_small_step, 10)))
            elif event.key() == QtCore.Qt.Key_PageDown:
                self.update_from_key(z=float(round(z_pos - self.slider_small_step, 10)))
            else:
                event.ignore()

    def scan_single(self):

        #self._scanning_logic.x_range = [0, 20e-6]
        #self._scanning_logic.y_range = [0, 20e-6]

        #self._mw.x_min_InputWidget.setValue(0)
        #self._mw.x_max_InputWidget.setValue(20e-6)
       #self._mw.y_min_InputWidget.setValue(0)
        #self._mw.y_max_InputWidget.setValue(20e-6)


        self.single_clicked = not self.single_clicked

        if self.single_clicked is True:
            self._mw.singleScan.setText('Stitching is off')
        else:
            self._mw.singleScan.setText('Stitching is on')


    def poi_add_clicked(self):
        print('Poi added')
        print(self.roi_xy.pos())
        print('{0},{1}'.format(self.roi_xy.pos()[0],self.roi_xy.pos()[1]))
        self._scanning_logic.add_poi(self.roi_xy.pos()[0],self.roi_xy.pos()[1])
        self.fillTreeItem(self._mw.poi_treeWidget,self._scanning_logic.pois)

    def fillTreeItem(self, widget, valuedict):
        """ Recursively fill a QTreeWidgetItem with the contents from a
            dictionary.
          @param QTreeWidgetItem item: the widget item to fill
          @param (dict, list, etc) value: value to fill in
        """

        for k, value in valuedict.items():
        #value = value[list(value.keys())[-1]]


            item = QtWidgets.QTreeWidgetItem(widget, [str(value['label'])])
            item.setExpanded(False)

            for key in value:
                child = QtWidgets.QTreeWidgetItem()
                child.setText(0, key)
                if '(x,y)' in key:
                    child.setText(1,'{:.6f} {:.6f} um'.format(*[i*1e6 for i in value[key]]))
                    item.addChild(child)
                elif 'τ_F' in key:
                    child.setText(1,'{:.3f} ns'.format(value[key]*1e9))
                    item.addChild(child)
                elif 'rating' in key:
                    item.setText(2,str(value[key]))
                elif 'label' in key:
                    pass
                else:
                    child.setText(1,value[key])
                    item.addChild(child)






    def nextscanMotor(self):

        # Write message to GUI that z is being updated...
        dynamicflag = 0

        while True:

            if dynamicflag is 0:

                x_pos = self._scanning_logic._current_x
                y_pos = self._scanning_logic._current_y

                hellow = pg.TextItem(text='Dynamic updating', anchor=(0, 0), fill=[0, 0, 0, 255])

                hellow.setPos(x_pos, y_pos)

                self._mw.xy_ViewWidget.addItem(hellow)
                dynamicflag = 1
            else:
                time.sleep(1)

            if self._scanning_logic.dynamic_updating_status == 0:
                self._mw.xy_ViewWidget.removeItem(hellow)
                position = self._scanning_logic.get_piezo_position()

                x_pos = position[0]
                y_pos = position[1]
                z_pos = position[2]

                self.update_slider_xp(x_pos)
                self.update_slider_yp(y_pos)
                self.update_slider_zp(z_pos)

                self.update_input_xp(x_pos)
                self.update_input_yp(y_pos)
                self.update_input_zp(z_pos)
                break


        self.drawPiezo()

        # if self.curri > 0:
        #
        #     self.drawNVs()


    #def drawNVs(self):


        if type((self.xy_image[self.curri]).qimage) is None:
            print('curri',self.curri)
            print(self.xy_image[self.curri])



        try:


            #print(type((self.xy_image[self.curri]).qimage))
            #print(self.xy_image[self.curri])
            #print(self.curri)

            if self.curri > 0:

                self.xy_image[self.curri-1].render()

                rgb = rgb_view((self.xy_image[self.curri-1]).qimage)

            else:
                self.xy_image[self.curri].render()

                rgb = rgb_view((self.xy_image[self.curri]).qimage)


            #print(rgb)

            data = 0.2989 * rgb[:, :, 0] + 0.5870 * rgb[:, :, 1] + 0.1140 * rgb[:, :, 2]

            #print(data)

            mean, median, std = sigma_clipped_stats(data, sigma=3.0, maxiters=5)

            '''
            daofind = DAOStarFinder(fwhm=2.0, threshold=4. * std, exclude_border=True) #, sharplo=0.4)

            #Be lenient on sharpness, roundness criteria then filter

            sources = daofind(data - median)

            # Now translate pixel positions to distance
            # Need (x, y) and resolution

            #print(sources)

            if len(sources) is not 0:


                if self.curri > 0:
                    position = self._scanning_logic.spx_grid[self.curri -1]
                    x_pos = position[1]
                    y_pos = position[0]
                else:
                    position = self._scanning_logic.get_motor_pos()
                    x_pos = position[0]
                    y_pos = position[1]

                res = self._scanning_logic.spx_size

                range = self._scanning_logic.spx_x_range[1]

                p_x = [-x_pos + range*s/res for s in sources['xcentroid']]

                p_y = [y_pos + range*s/res for s in sources['ycentroid']]



                # FIND POIS that overlap and mark as bad


                for i, p in enumerate(p_x):
                    #print(p_x[i], p_y[i])

                    #CircleROI origin defined as bottom left corner hence display by size

                    self.roi_dict.append(pg.CircleROI((p_x[i]-1e-6, p_y[i]-1e-6), size=2e-6))
                    label = str(self.curri) + "_" + str(i)
                    self._scanning_logic.add_poi(p_x[i], p_y[i], label = label)
                    self._mw.xy_ViewWidget.addItem(self.roi_dict[-1])

                self.fillTreeItem(self._mw.poi_treeWidget, self._scanning_logic.pois)
            '''

        except ValueError:
            #print('Error in type when converting')
            pass




    def drawPiezo(self):


        # First update box and slider for x, y

        #position = self._scanning_logic.get_motor_pos()
        # Should not need this hardware call as well, at this level it interferes with move motor calls

        x_pos = self._scanning_logic._current_x
        y_pos = self._scanning_logic._current_y

        print('getting current position now')

        self.update_input_x(x_pos)
        self.update_slider_x(x_pos)
        self.update_input_y(y_pos)
        self.update_slider_y(y_pos)

        raw_data_xy = np.zeros(self._scanning_logic.xy_image[self._scanning_logic.current_pointer]['image'][:, :, 3 + self.xy_channel].shape)


        if len(self.xy_image) is 1 and self.flagset is 0:

            self.xy_image[0].setLookupTable(self.my_colors.lut)
            print('As we drawPiezo, setting Rect centre to ',x_pos,y_pos)


            self.xy_image[0].setPxMode(0)

            self.xy_image[0].setRect(
                QtCore.QRectF(
                    x_pos - 1e-5,
                    y_pos - 1e-5,
                    2e-5,
                    2e-5
                )
            )

            self.flagset =1


        else:
            qpen = QPen()
            qpen.setWidthF(1e-7) #100 nm
            qpen.setColor(QColor(255, 255, 255))

            #compositionMode = QtGui.QPainter.CompositionMode_Multiply

            xy_image = pg.ImageItem(image=np.zeros(np.shape(raw_data_xy)), axisOrder='row-major', opacity=1, border=qpen)

            if self._scanning_logic.singlemode is True:
                piezo = self._scanning_logic.get_piezo_position()
                x_pos = x_pos + piezo[0]
                y_pos = y_pos + piezo[1]
                xrange = self._scanning_logic.scan_x_range[1] - self._scanning_logic.scan_x_range[0]
                yrange = self._scanning_logic.scan_y_range[1] - self._scanning_logic.scan_y_range[0]


                xy_image.setRect(
                    QtCore.QRectF(
                        x_pos+self._scanning_logic.scan_x_range[0],
                        y_pos+self._scanning_logic.scan_y_range[0],
                        xrange,
                        yrange
                    )
                )
            else:
                xy_image.setRect(
                    QtCore.QRectF(
                        x_pos - 1e-5,
                        y_pos - 1e-5,
                        2e-5,
                        2e-5
                    )
                )

            xy_image.setLookupTable(self.my_colors.lut)

            self._mw.xy_ViewWidget.addItem(xy_image, ignoreBounds=True)

            assert xy_image.getViewWidget() is self._mw.xy_ViewWidget

            self.xy_image.append(xy_image)


            self.curri += 1

        self.update_roi_xy(x_pos, y_pos)


    def drawPSF(self):

        #Updates PSF with 4D (x, y, z, opacity)
        self.psf_data = self._scanning_logic.psf_data

        #print('new psf_data at confocalgui')

        #Convert this data into the colourmap

        #Get cb range
        [cb_min, cb_max] = self.get_3d_cb_range()


        scale = np.linspace(cb_min, cb_max, 2000)

        #idxrange = np.arange(0,2000,1)

        #self.n_checker = self.n_checker + 1
        #print(self.n_checker)

        #cube = self.n_checker*np.ones((self._scanning_logic.psf_xyres * 4, self._scanning_logic.psf_xyres * 4, self._scanning_logic.psf_zres * 4, 4), dtype=np.ubyte)

        cube = np.empty((self._scanning_logic.psf_xyres, self._scanning_logic.psf_xyres, self._scanning_logic.psf_zres, 4), dtype=np.ubyte)

        for (i, j, k), element in np.ndenumerate(self.psf_data):
            idx = (np.abs(scale - element)).argmin()
            #if element > 1:
                #print('element {0}'.format(element))
            cube[i, j , k, :] = self.my_colors_a.lut[idx]  # [256*r,256*g,256*b,a/16]

        #
        # for (i, j, k), element in np.ndenumerate(self.psf_data):
        #     idx = int(np.interp(element, idxrange, scale))
        #     [r, g, b, a] = self.my_colors_a.lut[idx]  # / 256
        #     for x in range(0, 4):
        #         for y in range(0, 4):
        #             for z in range(0, 4):
        #                 cube[4 * i + x, 4 * j + y, 4 * k + z, :] = [r, g, b, a / 16]  # [256*r,256*g,256*b,a/16]

        self.psf_render.setData(cube) #= gl.GLVolumeItem(cube)
        #print(cube)


    def get_3d_cb_range(self):

        if self._mw.xy_cb_manual_RadioButton.isChecked():
            cb_min = self._mw.xy_cb_min_DoubleSpinBox_2.value()
            cb_max = self._mw.xy_cb_max_DoubleSpinBox_2.value()

        else:

            data = self.psf_data

            low_centile = self._mw.xy_cb_low_percentile_DoubleSpinBox_2.value()
            high_centile = self._mw.xy_cb_high_percentile_DoubleSpinBox_2.value()

            cb_min = np.percentile(self.psf_data, low_centile)
            cb_max = np.percentile(self.psf_data, high_centile)

        cb_range = [cb_min, cb_max]

        return cb_range


    def get_xy_cb_range(self):
        """ Determines the cb_min and cb_max values for the xy scan image
        """
        # If "Manual" is checked, or the image data is empty (all zeros), then take manual cb range.
        if self._mw.xy_cb_manual_RadioButton.isChecked()  or np.max(self.xy_image[0].image) == 0.0:
            cb_min = self._mw.xy_cb_min_DoubleSpinBox.value()
            cb_max = self._mw.xy_cb_max_DoubleSpinBox.value()

        # Otherwise, calculate cb range from percentiles.
        else:

            # Exclude any zeros (which are typically due to unfinished scan)
            # Exclude any values greater than 10 Mcs
            A = []

            # This can happen for instance, in the instance that the new scan has a different resolution

            for i in range(0, self.curri+1):

                A.append((self.xy_image[i].image).flatten())


            A = np.array(A)

            try:
                xy_image_nonzero = A[np.where(A > 0)]

                xy_image_y = xy_image_nonzero[np.where(xy_image_nonzero < 1e7)]

                # Read centile range
                low_centile = self._mw.xy_cb_low_percentile_DoubleSpinBox.value()
                high_centile = self._mw.xy_cb_high_percentile_DoubleSpinBox.value()

                cb_min = np.percentile(xy_image_y, low_centile)
                cb_max = np.percentile(xy_image_y, high_centile)

            except ValueError:
                #self.log.error('Error when trying to concatenate arrays for global maximum colourrange')
                cb_min = 0
                cb_max = 1e5
            except IndexError:
                #print('Value error', A)
                #self.log.error('Error with empty axis for global maximum colourrange')
                cb_min = 0
                cb_max = 1e5




        cb_range = [cb_min, cb_max]

        return cb_range


    def refresh_xy_colorbar(self):
        """ Adjust the xy colorbar.

        Calls the refresh method from colorbar, which takes either the lowest
        and highest value in the image or predefined ranges. Note that you can
        invert the colorbar if the lower border is bigger then the higher one.
        """
        cb_range = self.get_xy_cb_range()
        self.xy_cb.refresh_colorbar(cb_range[0], cb_range[1])

    def refresh_3d_colorbar(self):
        """ Adjust the xy colorbar.

        Calls the refresh method from colorbar, which takes either the lowest
        and highest value in the image or predefined ranges. Note that you can
        invert the colorbar if the lower border is bigger then the higher one.
        """
        cb_range = self.get_3d_cb_range()
        self.xy_cb_2.refresh_colorbar(cb_range[0], cb_range[1])

    def refresh_saturation(self):
        #print(self._sat_logic.power_vector)
        #print(self._sat_logic.count_vector)
        self.sat_track.setData(self._sat_logic.power_vector, self._sat_logic.count_vector)


    def kill_sat(self):
        self._sat_logic.stopRequested = True

    def refresh_z(self):
        self._scanning_logic.update_z()

    def dynamic_z(self):

        if self._scanning_logic.dynamic_z is False:
            self._scanning_logic.dynamic_z = True
            self._mw.dynamiczButton.setText('Dynamic z on')
        else:
            self._scanning_logic.dynamic_z = False
            self._mw.dynamiczButton.setText('Dynamic z off')

    def update_sat(self):
        #Push current values through to the saturation logic and start measurement
        time = self._mw.sattimeBox.value()
        points = self._mw.satpointsBox.value()
        start = self._mw.startpowerBox.value()
        end = self._mw.endpowerBox.value()

        self._sat_logic.update_param(start, end, points, time)
        self._sat_logic.on_measurement()


    def refresh_tracking(self, tag):
        #print('values are {0}'.format(self._track_logic._x_values))
        #print('line is {0}'.format(self._track_logic.x_track_line))
        self.x_track.setData(self._track_logic._x_values, self._track_logic.x_track_line)
        self.y_track.setData(self._track_logic._y_values, self._track_logic.y_track_line)
        self.z_track.setData(self._track_logic._z_values, self._track_logic.z_track_line)

        self.update_crosshair_position_from_logic('tracker')
        #self.smoothxarr.setData(self._track_logic._x_values, self._track_logic.smooth_x)

    def refresh_g2(self):
        # print('self x values')
        # print(self._g2_logic._x_values)
        # print('size')
        # print(len(self._g2_logic._x_values))
        # print('self x track line')
        # print(self._g2_logic.x_track_line)
        # print('size')
        # print(len(self._g2_logic.x_track_line))
        self.g2_track.setData(self._g2_logic._x_values, self._g2_logic.x_track_line)


    def refresh_fl(self):
        if self._mw.FL_chan1.isChecked() is True:
            self.fl_track_c1.setData(self._fl_logic._x_values, self._fl_logic.x_track_line_c1)
        else:
            self.fl_track_c1.clear()

        if self._mw.FL_chan2.isChecked() is True:
            self.fl_track_c2.setData(self._fl_logic._x_values, self._fl_logic.x_track_line_c2)
        else:
            self.fl_track_c2.clear()

        if self._mw.FL_total.isChecked() is True:
            self.fl_track_total.setData(self._fl_logic._x_values, self._fl_logic.x_track_line_total)
        else:
            self.fl_track_total.clear()

        if self._mw.FL_chan2.isChecked() is True:
            [t1, t2, error] = self._fl_logic.fit_data()
            fit_string = 'Fit result: ( {0:.2f},  {1:.2f}  ) ns with {2:.1f} error'.format(t1,t2,error)
            self._mw.FLfit_text.setText(fit_string)

    def just_piezo(self):

        # updates x and y piezo to 0 to 20 um
        print('just piezo')


    def disable_scan_actions(self):
        """ Disables the buttons for scanning.
        """
        # Ensable the stop scanning button
        self._mw.action_stop_scanning.setEnabled(True)

        # Disable the start scan buttons
        self._mw.action_scan_xy_start.setEnabled(False)

        self._mw.action_scan_xy_resume.setEnabled(False)

        self._mw.actionTracking.setEnabled(False)

        #self._mw.polarisation_scan.setEnabled(False)

        #self._mw.x_min_InputWidget.setEnabled(False)
        self._mw.x_max_InputWidget.setEnabled(False)
        #self._mw.y_min_InputWidget.setEnabled(False)
        #self._mw.y_max_InputWidget.setEnabled(False)

        # Set the zoom button if it was pressed to unpressed and disable it
        self._mw.action_zoom.setChecked(False)
        self._mw.action_zoom.setEnabled(False)

        self.set_history_actions(False)

    def enable_scan_actions(self):
        """ Reset the scan action buttons to the default active
        state when the system is idle.
        """
        # Disable the stop scanning button
        self._mw.action_stop_scanning.setEnabled(False)

        # Enable the scan buttons
        self._mw.action_scan_xy_start.setEnabled(True)


#        self._mw.actionRotated_depth_scan.setEnabled(True)

        self._mw.action_polarisation_scan.setEnabled(True)

        self._mw.actionTracking.setEnabled(True)

        #self._mw.x_min_InputWidget.setEnabled(True)
        self._mw.x_max_InputWidget.setEnabled(True)
        #self._mw.y_min_InputWidget.setEnabled(True)
        #self._mw.y_max_InputWidget.setEnabled(True)

        self._mw.action_zoom.setEnabled(True)

        self.set_history_actions(True)

        # Enable the resume scan buttons if scans were unfinished
        # TODO: this needs to be implemented properly.
        # For now they will just be enabled by default


        if self._scanning_logic._xyscan_continuable is True:
            self._mw.action_scan_xy_resume.setEnabled(True)
        else:
            self._mw.action_scan_xy_resume.setEnabled(False)

    def _refocus_finished_wrapper(self, caller_tag, optimal_pos):
        """ Re-enable the scan buttons in the GUI.
          @param str caller_tag: tag showing the origin of the action
          @param array optimal_pos: optimal focus position determined by optimizer

        Also, if the refocus was initiated here in confocalgui then we need to handle the
        "returned" optimal position.
        """
        # if caller_tag == 'confocalgui':
        #     self._scanning_logic.set_position(
        #         'optimizer',
        #         x=optimal_pos[0],
        #         y=optimal_pos[1],
        #         z=optimal_pos[2],
        #         a=0.0
        #     )
        self.enable_scan_actions()

    def set_history_actions(self, enable):
        """ Enable or disable history arrows taking history state into account. """
        if enable and self._scanning_logic.history_index < len(self._scanning_logic.history) - 1:
            self._mw.actionForward.setEnabled(True)
        else:
            self._mw.actionForward.setEnabled(False)
        if enable and self._scanning_logic.history_index > 0:
            self._mw.actionBack.setEnabled(True)
        else:
            self._mw.actionBack.setEnabled(False)

    def menu_settings(self):
        """ This method opens the settings menu. """
        self._sd.exec_()

    def update_settings(self):
        """ Write new settings from the gui to the file. """
        self._scanning_logic.set_clock_frequency(self._sd.clock_frequency_InputWidget.value())
        self._scanning_logic.return_slowness = self._sd.return_slowness_InputWidget.value()
        self._scanning_logic.permanent_scan = self._sd.loop_scan_CheckBox.isChecked()
        self.fixed_aspect_ratio_xy = self._sd.fixed_aspect_xy_checkBox.isChecked()
        self.slider_small_step = self._sd.slider_small_step_DoubleSpinBox.value()
        self.slider_big_step = self._sd.slider_big_step_DoubleSpinBox.value()
        #self.adjust_cursor_roi = self._sd.adjust_cursor_to_optimizer_checkBox.isChecked()

        # Update GUI icons to new loop-scan state
        self._set_scan_icons()
        # update cursor
        self.update_roi_xy_size()


    def keep_former_settings(self):
        """ Keep the old settings and restores them in the gui. """
        self._sd.clock_frequency_InputWidget.setValue(int(self._scanning_logic._clock_frequency))
        self._sd.return_slowness_InputWidget.setValue(int(self._scanning_logic.return_slowness))
        self._sd.loop_scan_CheckBox.setChecked(self._scanning_logic.permanent_scan)


        #self._sd.adjust_cursor_to_optimizer_checkBox.setChecked(self.adjust_cursor_roi)
        self._sd.fixed_aspect_xy_checkBox.setChecked(self.fixed_aspect_ratio_xy)

        self._sd.slider_small_step_DoubleSpinBox.setValue(float(self.slider_small_step))
        self._sd.slider_big_step_DoubleSpinBox.setValue(float(self.slider_big_step))

    def menu_optimizer_settings(self):
        """ This method opens the settings menu. """
        #self.keep_former_optimizer_settings()
        self._osd.exec_()

    def update_optimizer_settings(self):
        """ Write new settings from the gui to the file. """
        # self._optimizer_logic.refocus_XY_size = self._osd.xy_optimizer_range_DoubleSpinBox.value()
        # self._optimizer_logic.optimizer_XY_res = self._osd.xy_optimizer_resolution_SpinBox.value()
        # self._optimizer_logic.refocus_Z_size = self._osd.z_optimizer_range_DoubleSpinBox.value()
        # self._optimizer_logic.optimizer_Z_res = self._osd.z_optimizer_resolution_SpinBox.value()
        # self._optimizer_logic.set_clock_frequency(self._osd.count_freq_SpinBox.value())
        # self._optimizer_logic.return_slowness = self._osd.return_slow_SpinBox.value()
        # self._optimizer_logic.hw_settle_time = self._osd.hw_settle_time_SpinBox.value() / 1000
        # self._optimizer_logic.do_surface_subtraction = self._osd.do_surface_subtraction_CheckBox.isChecked()
        index = self._osd.opt_channel_ComboBox.currentIndex()
        # self._optimizer_logic.opt_channel = int(self._osd.opt_channel_ComboBox.itemData(index, QtCore.Qt.UserRole))
        #
        #
        # self._optimizer_logic.optimization_sequence = str(
        #     self._osd.optimization_sequence_lineEdit.text()
        #     ).upper().replace(" ", "").split(',')
        # self._optimizer_logic.check_optimization_sequence()
        # # z fit parameters
        # self._optimizer_logic.use_custom_params = self._osd.fit_tab.paramUseSettings
        # self.update_roi_xy_size()

    def keep_former_optimizer_settings(self):
        """ Keep the old settings and restores them in the gui. """
        # self._osd.xy_optimizer_range_DoubleSpinBox.setValue(self._optimizer_logic.refocus_XY_size)
        # self._osd.xy_optimizer_resolution_SpinBox.setValue(self._optimizer_logic.optimizer_XY_res)
        # self._osd.z_optimizer_range_DoubleSpinBox.setValue(self._optimizer_logic.refocus_Z_size)
        # self._osd.z_optimizer_resolution_SpinBox.setValue(self._optimizer_logic.optimizer_Z_res)
        # self._osd.count_freq_SpinBox.setValue(self._optimizer_logic._clock_frequency)
        # self._osd.return_slow_SpinBox.setValue(self._optimizer_logic.return_slowness)
        # self._osd.hw_settle_time_SpinBox.setValue(self._optimizer_logic.hw_settle_time * 1000)
        # self._osd.do_surface_subtraction_CheckBox.setChecked(self._optimizer_logic.do_surface_subtraction)
        #
        # old_ch = self._optimizer_logic.opt_channel
        # index = self._osd.opt_channel_ComboBox.findData(old_ch)
        # self._osd.opt_channel_ComboBox.setCurrentIndex(index)
        #
        # self._osd.optimization_sequence_lineEdit.setText(', '.join(self._optimizer_logic.optimization_sequence))
        #
        # # fit parameters
        # self._osd.fit_tab.resetFitParameters()
        self.update_roi_xy_size()



    def zero_stepper(self):
        """ Zeros stepper. """

        self.disable_scan_actions()
        self._scanning_logic.zero_stepper()
        self.enable_scan_actions()



    def ready_clicked(self):
        """ Stop the scan if the state has switched to ready. """



        if self._scanning_logic.module_state() == 'locked':
            self._scanning_logic.permanent_scan = False
            self._scanning_logic.stop_scanning_spx()
        # if self._optimizer_logic.module_state() == 'locked':
        #     self._optimizer_logic.stop_refocus()
        if self._track_logic.module_state() == 'locked':
            self._track_logic.stop_refocus()
        self.enable_scan_actions()

    def xy_scan_clicked(self):
        """ Manages what happens if the xy scan is started. """
        self.disable_scan_actions()
        if self.single_clicked is True:
            self._scanning_logic.start_super_scan(tag = 'single')
        else:
            self._scanning_logic.start_super_scan()

    def continue_xy_scan_clicked(self):
        """ Continue xy scan. """
        self.disable_scan_actions()
        self._scanning_logic.continue_scanning(zscan=False,tag='gui')



    def refocus_clicked(self):
        """ Start optimize position. """
        self.disable_scan_actions()
        # Get the current crosshair position to send to optimizer
        crosshair_pos = self._scanning_logic.get_position()
        #self.sigStartOptimizer.emit(crosshair_pos, 'confocalgui')

    def track_clicked(self):
        self.disable_scan_actions()
        crosshair_pos = self._scanning_logic.get_position()
        self.sigStartTracker.emit(crosshair_pos, 'confocalgui')



    def psf_start_clicked(self):

        print('is checked',self._mw.psf_start.isChecked())


        # Change labels and move crosshair

        position = self._scanning_logic.get_position()

        self.psf_position = position

        xrange = self._scanning_logic.psf_deltaxy
        #zrange = self._scanning_logic.psf_zres


        self.psf_text_xmin.setText("{0:.2f} μm".format((position[0] - xrange / 2)*1e6))
        self.psf_text_ymin.setText("{0:.2f} μm".format((position[1] - xrange / 2)*1e6))
        self.psf_text_xmax.setText("{0:.2f} μm".format((position[0] + xrange / 2)*1e6))
        self.psf_text_ymax.setText("{0:.2f} μm".format((position[1] + xrange / 2)*1e6))

        self.crosshair3D.set_pos(self._scanning_logic.psf_xyres/2, self._scanning_logic.psf_xyres/2, self._scanning_logic.psf_zres/2)





        if self._mw.psf_start.isChecked():
            #self.disable_scan_actions()
            self.sigStart3dscan.emit()
        else:
            if self._scanning_logic.module_state() == 'locked':
                self._scanning_logic.stop_scanning_3d()
            #self.enable_scan_actions()



    def update_crosshair_position_from_logic(self, tag):
        """ Update the GUI position of the crosshair from the logic.

        @param str tag: tag indicating the source of the update

        Ignore the update when it is tagged with one of the tags that the
        confocal gui emits, as the GUI elements were already adjusted.
        """
        if 'roi' not in tag and 'slider' not in tag and 'key' not in tag and 'input' not in tag and 'motor' not in tag:


            position = self._scanning_logic.get_motor_position()

            x_pos = position[0]
            y_pos = position[1]
            z_pos = position[2]
            #print('Total position',position)

            self.update_slider_x(x_pos)
            self.update_slider_y(y_pos)
            self.update_slider_z(z_pos)

            self.update_input_x(x_pos)
            self.update_input_y(y_pos)
            self.update_input_z(z_pos)


            position = self._scanning_logic.get_piezo_position()

            # XY image
            roi_h_view = x_pos - self.roi_xy.size()[0] * 0.5 + position[0]
            roi_v_view = y_pos - self.roi_xy.size()[1] * 0.5 + position[1]
            self.roi_xy.setPos([roi_h_view, roi_v_view])


            if 'tracker' in tag:
                self.roi_xy.setSize(2 * self._track_logic.res*self._track_logic.points, 2 * self._track_logic.res*self._track_logic.points)
            # else:
            #     #x_pos-
            #     self.roi_xy.setSize(2*self._scanning_logic.scan_x_range[1], 2*self._scanning_logic.scan_y_range[1])


            x_pos = position[0]
            y_pos = position[1]
            z_pos = position[2]


            #print('Piezo at',position)

            if self._mw.psf_start.isChecked():

                # range in x governed by self._scanning_logic.psf_xyres
                # total distance self._scanning_logic.psf_xydelta

                # edge is 0
                # centre is self._scanning_logic.psf_xyres / 2

                # current psf position
                #self.psf_position

                psf_x = (x_pos - self.psf_position[0]+self._scanning_logic.psf_deltaxy/2 )/self._scanning_logic.psf_deltares
                psf_y = (y_pos - self.psf_position[1]+self._scanning_logic.psf_deltaxy/2 ) /self._scanning_logic.psf_deltares
                psf_z = (z_pos - self.psf_position[2]+self._scanning_logic.psf_deltaz/2 ) /self._scanning_logic.psf_deltares

                print('Moving to {0}, {1}, {2}'.format(psf_x, psf_y, psf_z))
                self.crosshair3D.set_pos(psf_x, psf_y, psf_z)

            self.update_slider_xp(x_pos)
            self.update_slider_yp(y_pos)
            self.update_slider_zp(z_pos)

            self.update_input_xp(x_pos)
            self.update_input_yp(y_pos)
            self.update_input_zp(z_pos)


    def roi_xy_bounds_check(self, roi):
        """ Check if the focus cursor is oputside the allowed range after drag
            and set its position to the limit
        """
        h_pos = roi.pos()[0] + 0.5 * roi.size()[0]
        v_pos = roi.pos()[1] + 0.5 * roi.size()[1]

        #print('check bounds and clip to current motor position')
        #print()

        #
        # if self.hline_xy.stepper is False:
        #
        #     new_h_pos = np.clip(h_pos, *self._scanning_logic.piezo_x_range)
        #     new_v_pos = np.clip(v_pos, *self._scanning_logic.piezo_y_range)
        #
        # else:
        #     new_h_pos = np.clip(h_pos, *self._scanning_logic.x_range)
        #     new_v_pos = np.clip(v_pos, *self._scanning_logic.y_range)
        #
        #
        # if h_pos != new_h_pos or v_pos != new_v_pos:
        #     self.update_roi_xy(new_h_pos, new_v_pos)


    def update_roi_xy(self, h=None, v=None):
        """ Adjust the xy ROI position if the value has changed.

        @param float x: real value of the current x position
        @param float y: real value of the current y position

        Since the origin of the region of interest (ROI) is not the crosshair
        point but the lowest left point of the square, you have to shift the
        origin according to that. Therefore the position of the ROI is not
        the actual position!
        """
        roi_h_view = self.roi_xy.pos()[0]
        roi_v_view = self.roi_xy.pos()[1]

        if h is not None:
            roi_h_view = h - self.roi_xy.size()[0] * 0.5
        if v is not None:
            roi_v_view = v - self.roi_xy.size()[1] * 0.5

        self.roi_xy.setPos([roi_h_view, roi_v_view])

    def update_roi_xy_size(self):
        """ Update the cursor size showing the optimizer scan area for the XY image.
        """
        hpos = self.roi_xy.pos()[0]
        vpos = self.roi_xy.pos()[1]
        hsize = self.roi_xy.size()[0]
        vsize = self.roi_xy.size()[1]
        hcenter = hpos + 0.5 * hsize
        vcenter = vpos + 0.5 * vsize

        viewrange = self.xy_image[self.curri].getViewBox().viewRange()
        print('viewrange is ',viewrange)
        newsize = np.sqrt(np.sum(np.ptp(viewrange, axis=1)**2)) / 20
        self.roi_xy.setSize([newsize, newsize])
        self.roi_xy.setPos([hcenter - newsize / 2, vcenter - newsize / 2])





    def update_from_roi_xy(self, roi):
        """The user manually moved the XY ROI, adjust all other GUI elements accordingly

        @params object roi: PyQtGraph ROI object
        """
        h_pos = roi.pos()[0] + 0.5 * roi.size()[0]
        v_pos = roi.pos()[1] + 0.5 * roi.size()[1]


        #print('received at roi ',h_pos, v_pos)


        #How the clipping should work is that the ROI should be constrained to the current motor pos

        motor_position = self._scanning_logic.get_motor_position()

        #print('received at at roi motor pos',motor_position)

        #TODO move sliders of x and y to the right place and blink the move motor blue or similar

        h_pos = h_pos-motor_position[0]
        v_pos = v_pos-motor_position[1]

        self.update_slider_xp(h_pos)
        self.update_slider_yp(v_pos)

        self.update_input_xp(h_pos)
        self.update_input_yp(v_pos)

        self._scanning_logic.set_position('roixy', x=h_pos, y=v_pos)




    def move_motor_button_press(self):

        #propagate values to motor

        x_pos = self._mw.x_current_InputWidget.value()
        y_pos = self._mw.y_current_InputWidget.value()
        z_pos = self._mw.z_current_InputWidget.value()

        print('Moving motor to ', x_pos, y_pos, z_pos)

        self._scanning_logic.change_motor_position(x=x_pos, y=y_pos, z = z_pos)

    def update_fl_from_cursor(self):

        current_pos = self._scanning_logic.get_piezo_position()

        xrange = self._scanning_logic.spx_x_range
        yrange = self._scanning_logic.spx_y_range

        res = self._scanning_logic.spx_size

        m = res/(xrange[1]-xrange[0])
        c  = -xrange[0]
        x = int(m*(current_pos[0] + c))

        m = res/(xrange[1]-xrange[0])
        c = -yrange[0]
        y = int(m * (current_pos[1] + c))

        #xmin = current_pos[0] + self.scan_x_range[0]
        #xmax = current_pos[0] + self.scan_x_range[1]
        #ymin = current_pos[1] + self.scan_y_range[0]
        #ymax = current_pos[1] + self.scan_y_range

        res_range = [0, res]

        xscope = np.clip([x - 2, x + 2], *res_range)
        yscope = np.clip([y - 2, y + 2], *res_range)

        #print('(x,y) = {0},{1}'.format(x,y))

        data = np.sum(self._scanning_logic.xy_image[self._scanning_logic.current_pointer]['fl'][yscope[0]:yscope[1], xscope[0]:xscope[1], :], axis=(0, 1))

        #print(self._scanning_logic.xy_image_fl[yscope[0]:yscope[1], xscope[0]:xscope[1], :])

        #np.sum(scanner.xy_image_fl[135 - sum_size:135 + sum_size, 64 - sum_size:64 + sum_size, :], axis=(0, 1))

        self.fl_track.setData(x=np.arange(0,len(data)+1),y=data)





    def update_from_key(self, x=None, y=None, z=None):
        """The user pressed a key to move the crosshair, adjust all GUI elements.

        @param float x: new x position in m
        @param float y: new y position in m
        @param float z: new z position in m
        """
        if x is not None:


            self.update_roi_xy(h=x)

            self.update_slider_xp(x)
            self.update_input_xp(x)
            self._scanning_logic.set_position('xinput', x=x)

        if y is not None:


            self.update_roi_xy(v=y)

            self.update_slider_yp(y)
            self.update_input_yp(y)
            self._scanning_logic.set_position('yinput', y=y)


        if z is not None:

            self.update_slider_zp(z)
            self.update_input_zp(z)
            self._scanning_logic.set_position('zinput', z=z)

    def update_from_input_x(self):
        """ The user changed the number in the x position spin box, adjust all
            other GUI elements."""
        x_pos = self._mw.x_current_InputWidget.value()
        self.update_roi_xy(h=x_pos)

        self.update_slider_x(x_pos)

        self.x_pos = x_pos



    def update_from_input_y(self):
        """ The user changed the number in the y position spin box, adjust all
            other GUI elements."""
        y_pos = self._mw.y_current_InputWidget.value()
        self.update_roi_xy(v=y_pos)

        self.update_slider_y(y_pos)

        self.y_pos = y_pos

    def update_from_input_z(self):
        """ The user changed the number in the y position spin box, adjust all
            other GUI elements."""
        z_pos = self._mw.z_current_InputWidget.value()

        self.update_slider_z(z_pos)

        self.z_pos = z_pos


    def update_from_input_xp(self):
        """ The user changed the number in the x position spin box, adjust all
            other GUI elements."""
        x_pos = self._mw.xp_current_InputWidget.value() # + self._scanning_logic.superdict[1]

        x_motor = self._scanning_logic.get_motor_position()[0]
        self.update_roi_xy(h=x_pos+x_motor)

        self.update_slider_xp(x_pos)
        self._scanning_logic.set_position('xinput', x=x_pos)
        #self._optimizer_logic.set_position('xinput', x=x_pos)

    def update_from_input_stitch_range(self):

        self._scanning_logic.stitch_range = self._mw.stitch_InputWidget.value()


    def update_from_input_track_wait(self):

        self._track_logic.wait_time = self._mw.track_wait.value()

    def update_from_input_track_av(self):

        self._track_logic.av = self._mw.track_av.value()


    def update_track_timer(self):
        time_text = '{0:.1f} s'.format(self._track_logic.remaining_time)

        self._mw.track_time.setText(time_text)


    def update_from_x_stepper_adjust(self):

        jog_step = self._mw.xstepperadjust.value()

        # change jog step
        self._scanning_logic.change_jog_step({'x' :jog_step})

    def update_from_y_stepper_adjust(self):

        jog_step = self._mw.ystepperadjust.value()

        # change jog step
        self._scanning_logic.change_jog_step({'y' :jog_step})

    def update_from_z_stepper_adjust(self):

        jog_step = self._mw.zstepperadjust.value()

        # change jog step
        self._scanning_logic.change_jog_step({'z' :jog_step})


    def jog_x_up_pressed(self):

        #send jog
        self._scanning_logic.jog(x=1)

        x_motor = self._scanning_logic.get_motor_position()[0]

        #update x slider and input widget
        self.update_slider_x(x_motor)
        self.update_input_x(x_motor)


    def jog_x_down_pressed(self):

        #send jog
        self._scanning_logic.jog(x=-1)

        x_motor = self._scanning_logic.get_motor_position()[0]

        #update x slider and input widget
        self.update_slider_x(x_motor)
        self.update_input_x(x_motor)

    def jog_y_up_pressed(self):

        #send jog
        self._scanning_logic.jog(y=1)

        y_motor = self._scanning_logic.get_motor_position()[1]

        #update y slider and input widget
        self.update_slider_y(y_motor)
        self.update_input_y(y_motor)


    def jog_y_down_pressed(self):

        #send jog
        self._scanning_logic.jog(y=-1)

        y_motor = self._scanning_logic.get_motor_position()[1]

        # update y slider and input widget
        self.update_slider_y(y_motor)
        self.update_input_y(y_motor)

    def jog_z_up_pressed(self):

        #send jog
        self._scanning_logic.jog(z=1)

        z_motor = self._scanning_logic.get_motor_position()[2]

        #update zslider and input widget
        self.update_slider_z(z_motor)
        self.update_input_z(z_motor)


    def jog_z_down_pressed(self):

        #send jog
        self._scanning_logic.jog(z=-1)

        z_motor = self._scanning_logic.get_motor_position()[2]

        # update z slider and input widget
        self.update_slider_z(z_motor)


    def update_from_input_yp(self):
        """ The user changed the number in the y position spin box, adjust all
            other GUI elements."""
        y_pos = self._mw.yp_current_InputWidget.value() #+ self._scanning_logic.superdict[1]

        y_motor = self._scanning_logic.get_motor_position()[1]

        self.update_roi_xy(v=y_pos+y_motor)

        self.update_slider_yp(y_pos)
        self._scanning_logic.set_position('yinput', y=y_pos)
        #self._optimizer_logic.set_position('yinput', y=y_pos)

    def update_from_input_zp(self):
        """ The user changed the number in the z position spin box, adjust all
           other GUI elements."""
        z_pos = self._mw.zp_current_InputWidget.value()

        self.update_slider_zp(z_pos)
        self._scanning_logic.set_position('zinput', z=z_pos)
        #self._optimizer_logic.set_position('zinput', z=z_pos)

    def update_from_input_spx_size(self):
        """ The user changed the number in the z position spin box, adjust all
           other GUI elements."""
        spx_size = self._mw.spx_size_InputWidget.value()
        #self.update_roi_depth(v=z_pos)
        #self.update_slider_z(z_pos)
        self._scanning_logic.set_super_pixel(size= spx_size )
        #self._optimizer_logic.set_position('zinput', z=z_pos)

    def update_from_input_track_res(self):
        points = self._mw.track_res.value()

        self._track_logic.points = points
        self._track_logic.clear_plots()

    def update_from_input_track_delta(self):
        res = self._mw.track_delta.value()
        points = self._track_logic.points
        self._mw.track_Delta.setValue(res*points)
        self._track_logic.res = res
        self._track_logic.clear_plots()


    def update_from_input_track_delta2(self):
        res = self._mw.track_Delta.value()
        res = res/self._track_logic.points
        self._mw.track_delta.setValue(res)
        self._track_logic.res = res
        self._track_logic.clear_plots()


    def update_from_input_psf(self):
        xy = self._mw.psf_xy.value()
        z = self._mw.psf_z.value()
        res = self._mw.psf_res.value()

        self._scanning_logic.psf_deltaz = z
        self._scanning_logic.psf_deltaxy = xy
        self._scanning_logic.psf_deltares = res
        self._scanning_logic.psf_xyres = int(xy/res)
        self._scanning_logic.psf_zres = int(z/res)

    def update_from_input_spx_overlap(self):
        """ The user changed the number in the z position spin box, adjust all
           other GUI elements."""
        spx_overlap = self._mw.spx_overlap_InputWidget.value()
        self._scanning_logic.set_super_pixel(overlap= spx_overlap )

    def update_input_x(self, x_pos):
        """ Update the displayed x-value.

        @param float x_pos: the current value of the x position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.x_current_InputWidget.setValue(x_pos)

    def update_input_xp(self, x_pos):
        """ Update the displayed x-value.

        @param float x_pos: the current value of the x position in m
        """
        # Convert x_pos to number of points for the slider:

        #print('xpos in input xp {0}'.format(x_pos))

        self._mw.xp_current_InputWidget.setValue(x_pos)

    def update_input_y(self, y_pos):
        """ Update the displayed y-value.

        @param float y_pos: the current value of the y position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.y_current_InputWidget.setValue(y_pos)

    def update_input_yp(self, y_pos):
        """ Update the displayed y-value.

        @param float y_pos: the current value of the y position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.yp_current_InputWidget.setValue(y_pos)

    def update_input_z(self, z_pos):
        """ Update the displayed z-value.

        @param float z_pos: the current value of the z position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.z_current_InputWidget.setValue(z_pos)

    def update_input_zp(self, z_pos):
        """ Update the displayed z-value.

        @param float z_pos: the current value of the z position in m
        """
        # Convert x_pos to number of points for the slider:
        self._mw.zp_current_InputWidget.setValue(z_pos)

    def update_from_slider_x(self, sliderValue):
        """The user moved the x position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        x_pos = self._scanning_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_roi_xy(h=x_pos)
        self.update_input_x(x_pos)
        self.x_pos = x_pos

    def update_from_slider_xp(self, sliderValue):
        """The user moved the x position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        x_pos =  self._scanning_logic.piezo_range[0] + sliderValue * self.sliderp_res
        self.update_roi_xy(h=x_pos)
        self.update_input_xp(x_pos)
        #print('from slider xp x pos is',x_pos)
        self._scanning_logic.set_position('piezo', x=x_pos)

    def update_from_slider_y(self, sliderValue):
        """The user moved the y position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        y_pos = self._scanning_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_roi_xy(v=y_pos)
        self.update_input_y(y_pos)
        self.y_pos = y_pos

    def update_from_slider_yp(self, sliderValue):
        """The user moved the y position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        y_pos = self._scanning_logic.piezo_range[0] + sliderValue * self.sliderp_res
        self.update_roi_xy(v=y_pos)
        self.update_input_yp(y_pos)
        self._scanning_logic.set_position('piezo', y=y_pos)

    def update_from_slider_z(self, sliderValue):
        """The user moved the z position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        z_pos = self._scanning_logic.motor_range[0] + sliderValue * self.slider_res
        self.update_input_z(z_pos)
        self.z_pos = z_pos

    def update_from_slider_zp(self, sliderValue):
        """The user moved the z position slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        z_pos = self._scanning_logic.piezo_range[0] + sliderValue * self.sliderp_res
        self.update_input_zp(z_pos)
        self._scanning_logic.set_position('piezo', z=z_pos)


    def update_from_track_res(self,res):
        self._track_logic.points = res

    def update_from_track_delta(self, delta):
        self._track_logic.optimizer_res = delta


    def update_slider_x(self, x_pos):
        """ Update the x slider when a change happens.

        @param float x_pos: x position in m
        """
        self._mw.x_SliderWidget.setValue((x_pos - self._scanning_logic.motor_range[0]) / self.slider_res)

    def update_slider_y(self, y_pos):
        """ Update the y slider when a change happens.

        @param float y_pos: x yosition in m
        """
        self._mw.y_SliderWidget.setValue((y_pos - self._scanning_logic.motor_range[0]) / self.slider_res)

    def update_slider_xp(self, x_pos):
        """ Update the x slider when a change happens.

        @param float x_pos: x position in m
        """
        #print('xpos in slider {0} take piezo range {1}'.format(x_pos,self._scanning_logic.piezo_range[0] ))
        self._mw.xp_SliderWidget.setValue((x_pos- self._scanning_logic.piezo_range[0]) / self.sliderp_res)

    def update_slider_yp(self, y_pos):
        """ Update the y slider when a change happens.

        @param float y_pos: x yosition in m
        """
        #print('updating slider p with {0}', y_pos)
        self._mw.yp_SliderWidget.setValue((y_pos- self._scanning_logic.piezo_range[0] ) / self.sliderp_res)

    def update_slider_z(self, z_pos):
        """ Update the z slider when a change happens.

        @param float z_pos: z position in m
        """
        self._mw.z_SliderWidget.setValue((z_pos - self._scanning_logic.motor_range[0]) / self.slider_res)

    def update_slider_zp(self, z_pos):
        """ Update the z slider when a change happens.

        @param float z_pos: z position in m
        """
        self._mw.zp_SliderWidget.setValue((z_pos - self._scanning_logic.piezo_range[0]) / self.sliderp_res)

    def change_xy_resolution(self):
        """ Update the xy resolution in the logic according to the GUI.
        """
        #self._scanning_logic.xy_resolution = self._mw.xy_res_InputWidget.value()

    def change_z_resolution(self):
        """ Update the z resolution in the logic according to the GUI.
        """
        self._scanning_logic.z_resolution = self._mw.z_res_InputWidget.value()

    def change_x_range(self):
        """ Adjust the image range for x in the logic. """
        self._scanning_logic.scan_x_range = [0,
            self._mw.x_max_InputWidget.value()]

        self._scanning_logic.scan_y_range = [0,
            self._mw.x_max_InputWidget.value()]



    def update_tilt_correction(self):
        """ Update all tilt points from the scanner logic. """
        self._mw.tilt_01_x_pos_doubleSpinBox.setValue(self._scanning_logic.point1[0])
        self._mw.tilt_01_y_pos_doubleSpinBox.setValue(self._scanning_logic.point1[1])
        self._mw.tilt_01_z_pos_doubleSpinBox.setValue(self._scanning_logic.point1[2])

        self._mw.tilt_02_x_pos_doubleSpinBox.setValue(self._scanning_logic.point2[0])
        self._mw.tilt_02_y_pos_doubleSpinBox.setValue(self._scanning_logic.point2[1])
        self._mw.tilt_02_z_pos_doubleSpinBox.setValue(self._scanning_logic.point2[2])

        self._mw.tilt_03_x_pos_doubleSpinBox.setValue(self._scanning_logic.point3[0])
        self._mw.tilt_03_y_pos_doubleSpinBox.setValue(self._scanning_logic.point3[1])
        self._mw.tilt_03_z_pos_doubleSpinBox.setValue(self._scanning_logic.point3[2])

    def update_xy_channel(self, index):
        """ The displayed channel for the XY image was changed, refresh the displayed image.

            @param index int: index of selected channel item in combo box
        """
        self.xy_channel = int(self._mw.xy_channel_ComboBox.itemData(index, QtCore.Qt.UserRole))
        self.refresh_xy_image()


    def shortcut_to_xy_cb_manual(self):
        """Someone edited the absolute counts range for the xy colour bar, better update."""
        self._mw.xy_cb_manual_RadioButton.setChecked(True)
        self.update_xy_cb_range()

    def shortcut_to_xy_cb_centiles(self):
        """Someone edited the centiles range for the xy colour bar, better update."""
        self._mw.xy_cb_centiles_RadioButton.setChecked(True)
        self.update_xy_cb_range()

    def shortcut_to_3d_cb_manual(self):
        """Someone edited the absolute counts range for the 3d colour bar, better update."""
        self._mw.xy_cb_manual_RadioButton_2.setChecked(True)
        self.update_3d_cb_range()

    def shortcut_to_3d_cb_centiles(self):
        """Someone edited the centiles range for the 3d colour bar, better update."""
        self._mw.xy_cb_centiles_RadioButton_2.setChecked(True)
        self.update_3d_cb_range()


    def update_xy_cb_range(self):
        """Redraw xy colour bar and scan image."""

        self.refresh_xy_colorbar()
        self.refresh_xy_image_range()


    def update_3d_cb_range(self):
        """Redraw xy colour bar and scan image."""

        self.refresh_3d_colorbar()
        self.refresh_3d_image_range()


    def refresh_xy_image_range(self):
        for i in range(0, self.curri+1):
            cb_range = self.get_xy_cb_range()
            image = self.xy_image[i]
            image.setLevels((cb_range[0], cb_range[1]), update=True)

    def refresh_3d_image_range(self):


        [cb_min, cb_max] = self.get_3d_cb_range()

        scale = np.linspace(cb_min, cb_max, 2000)

        cube = np.empty(
            (self._scanning_logic.psf_xyres, self._scanning_logic.psf_xyres, self._scanning_logic.psf_zres, 4),
            dtype=np.ubyte)

        for (i, j, k), element in np.ndenumerate(self.psf_data):
            idx = (np.abs(scale - element)).argmin()
            # if element > 1:
            # print('element {0}'.format(element))
            cube[i, j, k, :] = self.my_colors_a.lut[idx]  # [256*r,256*g,256*b,a/16]

        self.psf_render.setData(cube)




    def refresh_xy_image(self):
        """ Update the current XY image from the logic.

        Everytime the scanner is scanning a line in xy the
        image is rebuild and updated in the GUI.
        """
        #self.xy_image.getViewBox().updateAutoRange()

        #xy_image_data = 1000/(1+self.curri)*np.ones(self._scanning_logic.xy_image[:, :, 3 + self.xy_channel].shape)




        cb_range = self.get_xy_cb_range()

        #for i in range(0, self.curri):
        #    self.xy_image[i].setLevels((cb_range[0], cb_range[1]), update=True)

        #update the colours of all the other images

        # This should only be the case if you are building the image
        # i.e. do not want to set the old image to the new image




        if self.curri is len(self.xy_image)-1 :
            # Now update image with new color scale, and update colorbar
           # self.xy_image[self.curri].


            if self._scanning_logic._scan_counter >= 0:
                # take away double flip and cover it in hardware
                xy_image_data = np.array(self._scanning_logic.xy_image[self._scanning_logic.current_pointer]['image'][:, :, 3 + self.xy_channel], copy = True)

                self.xy_image[self.curri].setImage(image=xy_image_data, levels=(cb_range[0], cb_range[1]))
            else:
                xy_image_data = 1000*np.ones(self._scanning_logic.xy_image[self._scanning_logic.current_pointer]['image'][:, :, 3 + self.xy_channel].shape)


                #self.xy_image[self.curri].setImage(image=xy_image_data, levels=(cb_range[0], cb_range[1]))

            #print('scan counter', self._scanning_logic._scan_counter)
            cb_range = self.get_xy_cb_range()
            #self.xy_image[self.curri].setImage(image=xy_image_data, levels =(cb_range[0], cb_range[1]) )


            if self.curri is 0 and self.single_clicked is False:
                #print('setting here curri 0')
                self.xy_image[self.curri].setRect(
                    QtCore.QRectF(
                        self._scanning_logic.spx_grid[0][0]-1e-5,
                        self._scanning_logic.spx_grid[0][1]-1e-5,
                        2e-5,
                        2e-5
                    ))

        self.refresh_xy_colorbar()

        # Unlock state widget if scan is finished
        if self._scanning_logic.module_state() != 'locked':
            self.enable_scan_actions()



    def refresh_refocus_image(self):
        """Refreshes the xy image, the crosshair and the colorbar. """
        ##########
        # Updating the xy optimizer image with color scaling based only on nonzero data
        #xy_optimizer_image = self._optimizer_logic.xy_refocus_image[:, :, 3 + self._optimizer_logic.opt_channel]
        todo = 1
        # # If the Z scan is done first, then the XY image has only zeros and there is nothing to draw.
        # if np.max(xy_optimizer_image) != 0:
        #     colorscale_min = np.min(xy_optimizer_image[np.nonzero(xy_optimizer_image)])
        #     colorscale_max = np.max(xy_optimizer_image[np.nonzero(xy_optimizer_image)])
        #
        #     self.xy_refocus_image.setImage(image=xy_optimizer_image, levels=(colorscale_min, colorscale_max))
        # ##########
        # # TODO: does this need to be reset every time this refresh function is called?
        # # Is there a better way?
        # self.xy_refocus_image.setRect(
        #     QtCore.QRectF(
        #         self._optimizer_logic._initial_pos_x - 0.5 * self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic._initial_pos_y - 0.5 * self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic.refocus_XY_size,
        #         self._optimizer_logic.refocus_XY_size
        #     )
        # )
        # ##########
        # # Crosshair in optimizer
        # self.vLine.setValue(self._optimizer_logic.optim_pos_x)
        # self.hLine.setValue(self._optimizer_logic.optim_pos_y)
        # ##########
        # # The depth optimization
        # # data from chosen channel
        # self.depth_refocus_image.setData(
        #     self._optimizer_logic._zimage_Z_values,
        #     self._optimizer_logic.z_refocus_line[:, self._optimizer_logic.opt_channel])
        # # fit made from the data
        # self.depth_refocus_fit_image.setData(
        #     self._optimizer_logic._fit_zimage_Z_values,
        #     self._optimizer_logic.z_fit_data)
        # ##########
        # # Set the optimized position label
        # self._mw.refocus_position_label.setText(
        #     'µ = ({0:.3f}, {1:.3f}, {2:.3f}) µm   '
        #     'σ = ({3:.3f}, {4:.3f}, {5:.3f}) µm '
        #     ''.format(
        #         self._optimizer_logic.optim_pos_x * 1e6,
        #         self._optimizer_logic.optim_pos_y * 1e6,
        #         self._optimizer_logic.optim_pos_z * 1e6,
        #         self._optimizer_logic.optim_sigma_x * 1e6,
        #         self._optimizer_logic.optim_sigma_y * 1e6,
        #         self._optimizer_logic.optim_sigma_z * 1e6
        #     )
        # )

    def refresh_scan_line(self):
        pass


    def adjust_xy_window(self):
        """ Fit the visible window in the xy scan to full view.

        Be careful in using that method, since it uses the input values for
        the ranges to adjust x and y. Make sure that in the process of the depth scan
        no method is calling adjust_depth_window, otherwise it will adjust for you
        a window which does not correspond to the scan!
        """
        # It is extremly crucial that before adjusting the window view and
        # limits, to make an update of the current image. Otherwise the
        # adjustment will just be made for the previous image.


        #self.refresh_xy_image()
        xy_viewbox = self.xy_image[self.curri].getViewBox()

        xMin = self._scanning_logic.x_range[0]
        xMax = self._scanning_logic.x_range[1]
        yMin = self._scanning_logic.y_range[0]
        yMax = self._scanning_logic.y_range[1]

        if self.fixed_aspect_ratio_xy:
            # Reset the limit settings so that the method 'setAspectLocked'
            # works properly. It has to be done in a manual way since no method
            # exists yet to reset the set limits:
            xy_viewbox.state['limits']['xLimits'] = [None, None]
            xy_viewbox.state['limits']['yLimits'] = [None, None]
            xy_viewbox.state['limits']['xRange'] = [None, None]
            xy_viewbox.state['limits']['yRange'] = [None, None]

            xy_viewbox.setAspectLocked(lock=True, ratio=1.0)
            xy_viewbox.updateViewRange()
        else:
            xy_viewbox.setLimits(xMin=xMin - (xMax - xMin) * self.image_x_padding,
                                 xMax=xMax + (xMax - xMin) * self.image_x_padding,
                                 yMin=yMin - (yMax - yMin) * self.image_y_padding,
                                 yMax=yMax + (yMax - yMin) * self.image_y_padding)

        #self.xy_image.setRect(QtCore.QRectF(xMin, yMin, xMax - xMin, yMax - yMin))

        self.put_cursor_in_xy_scan()

        xy_viewbox.updateAutoRange()
        xy_viewbox.updateViewRange()
        self.update_roi_xy()



    def put_cursor_in_xy_scan(self):
        """Put the xy crosshair back if it is outside of the visible range. """
        view_x_min = self._scanning_logic.x_range[0]
        view_x_max = self._scanning_logic.x_range[1]
        view_y_min = self._scanning_logic.y_range[0]
        view_y_max = self._scanning_logic.y_range[1]

        x_value = self.roi_xy.pos()[0]
        y_value = self.roi_xy.pos()[1]
        cross_pos = self.roi_xy.pos() + self.roi_xy.size() * 0.5

        if (view_x_min > cross_pos[0]):
            x_value = view_x_min + self.roi_xy.size()[0]

        if (view_x_max < cross_pos[0]):
            x_value = view_x_max - self.roi_xy.size()[0]

        if (view_y_min > cross_pos[1]):
            y_value = view_y_min + self.roi_xy.size()[1]

        if (view_y_max < cross_pos[1]):
            y_value = view_y_max - self.roi_xy.size()[1]

        self.roi_xy.setPos([x_value, y_value], update=True)

    def save_xy_scan_data(self):
        """ Run the save routine from the logic to save the xy confocal data."""
        cb_range = self.get_xy_cb_range()

        # Percentile range is None, unless the percentile scaling is selected in GUI.
        pcile_range = None
        if not self._mw.xy_cb_manual_RadioButton.isChecked():
            low_centile = self._mw.xy_cb_low_percentile_DoubleSpinBox.value()
            high_centile = self._mw.xy_cb_high_percentile_DoubleSpinBox.value()
            pcile_range = [low_centile, high_centile]

        #self._scanning_logic.save_xy_data(colorscale_range=cb_range, percentile_range=pcile_range)

        # Save the raw data to file
        filelabel = 'confocal_xy_data'

        # TODO: find a way to produce raw image in savelogic.  For now it is saved here.
        filepath = self._save_logic.get_path_for_module(module_name='Confocal')
        filename = filepath + os.sep + time.strftime('%Y%m%d-%H%M-%S_confocal_xy_scan_array')

        for x in range(0,self.curri+1):
            self.xy_image[x].save(filename +'_' +str(x) + '_raw.png')


        self._scanning_logic.save_xy_data()


        exporter = pg.exporters.ImageExporter(self._mw.xy_ViewWidget.plotItem)
        exporter.export(filename + '.png')

        #glview.grabFrameBuffer().save('fileName.png')


    def save_xy_scan_data_old(self):
        """ Run the save routine from the logic to save the xy confocal data."""
        cb_range = self.get_xy_cb_range()

        # Percentile range is None, unless the percentile scaling is selected in GUI.
        pcile_range = None
        if not self._mw.xy_cb_manual_RadioButton.isChecked():
            low_centile = self._mw.xy_cb_low_percentile_DoubleSpinBox.value()
            high_centile = self._mw.xy_cb_high_percentile_DoubleSpinBox.value()
            pcile_range = [low_centile, high_centile]

        #self._scanning_logic.save_xy_data(colorscale_range=cb_range, percentile_range=pcile_range)

        # Save the raw data to file
        filelabel = 'confocal_xy_data'

        # TODO: find a way to produce raw image in savelogic.  For now it is saved here.
        filepath = self._save_logic.get_path_for_module(module_name='Confocal')
        filename = filepath + os.sep + time.strftime('%Y%m%d-%H%M-%S_confocal_xy_scan_array')

        parameters = OrderedDict()

        parameters['Piezo resolution (samples per piezo scan)'] = self._scanning_logic.spx_size

        parameters['Global  x'] = self._scanning_logic.superdict['x']
        parameters['Global  y'] = self._scanning_logic.superdict['y']
        parameters['Global  z'] = self._scanning_logic.superdict['z']
        parameters['Stitch range'] = self._scanning_logic.stitch_range

        data_raw_counts = OrderedDict()
        for x in range(0,self.curri+1):
            self.xy_image[x].save(filename +'_' +str(x) + '_raw.png')
            data = self.xy_image[x].image

            #save_xy_scan_data
            #data_raw_counts['counts in spx{ fl} = self.xy_image_fl[x].flatten()
            #data # xy information
            # polarisation information
            data_raw_counts['Counts in spx {}'.format(str(x))] = data.flatten()

        self._save_logic.save_data(data_raw_counts,
                                       filepath=filepath,
                                       filelabel=filelabel,
                                       fmt='%.6e',
                                       delimiter='\t',
                                   parameters=parameters)

        exporter = pg.exporters.ImageExporter(self._mw.xy_ViewWidget.plotItem)
        exporter.export(filename + '.png')

        #glview.grabFrameBuffer().save('fileName.png')

    def save_xy_scan_image(self):
        """ Save the image and according to that the data.

        Here only the path to the module is taken from the save logic, but the
        picture save algorithm is situated here in confocal, since it is a very
        specific task to save the used PlotObject.
        """
        self.log.warning('Deprecated, use normal save method instead!')










    def switch_hardware(self):
        """ Switches the hardware state. """
        self._scanning_logic.switch_hardware(to_on=False)

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

    def small_optimizer_view(self):
        """ Rearrange the DockWidgets to produce a small optimizer interface
        """
        # Hide the other dock widgets
        self._mw.xy_scan_dockWidget.hide()
        self._mw.scan_control_dockWidget.hide()
        self._mw.depth_scan_dockWidget.hide()

        # # Show the optimizer dock widget, and re-dock
        # self._mw.optimizer_dockWidget.show()
        # self._mw.optimizer_dockWidget.setFloating(False)

        # Resize the window to small dimensions
        self._mw.resize(1000, 360)

    #####################################################################
    #        Methods for the zoom functionality of confocal GUI         #
    #####################################################################

# FIXME: These methods can be combined to one, because the procedure for the xy
#       and the depth scan is the same. A nice way has to be figured our here.
# FIXME: For the depth scan both possibilities have to be implemented, either
#       for a xz of a yz scan. The image ranges have to be adjusted properly.

    def zoom_clicked(self, is_checked):
        """ Activates the zoom mode in the xy and depth Windows.

        @param bool is_checked: pass the state of the zoom button if checked
                                or not.

        Depending on the state of the zoom button the DragMode in the
        ViewWidgets are changed.  There are 3 possible modes and each of them
        corresponds to a int value:
            - 0: NoDrag
            - 1: ScrollHandDrag
            - 2: RubberBandDrag

        Pyqtgraph implements every action for the NoDrag mode. That means the
        other two modes are not used at the moment. Therefore we are using the
        RubberBandDrag mode to simulate a zooming procedure. The selection
        window in the RubberBandDrag is only used to show the user which region
        will be selected. But the zooming idea is based on catched
        mousePressEvent and mouseReleaseEvent, which will be used if the
        RubberBandDrag mode is activated.

        For more information see the qt doc:
        http://doc.qt.io/qt-4.8/qgraphicsview.html#DragMode-enum
        """

        # You could also set the DragMode by its integer number, but in terms
        # of readability it is better to use the direct attributes from the
        # ViewWidgets and pass them to setDragMode.

        try:
            if is_checked:
                self.xy_image[self.curri].getViewBox().setLeftButtonAction('rect')
            else:
                self.xy_image[self.curri].getViewBox().setLeftButtonAction('pan')
        except IndexError:
            self.log.debug('No image to zoom on')

    def xy_scan_start_zoom_point(self, event):
        """ Get the mouse coordinates if the mouse button was pressed.

        @param QMouseEvent event: Mouse Event object which contains all the
                                  information at the time the event was emitted
        """
        try:
            if self._mw._doubleclicked:
                event.ignore()
                return

            # catch the event if the zoom mode is activated and if the event is
            # coming from a left mouse button.
            if not (self._mw.action_zoom.isChecked() and (event.button() == QtCore.Qt.LeftButton)):
                event.ignore()
                return

            pos = self.xy_image[self.curri].getViewBox().mapSceneToView(event.localPos())

            # store the initial mouse position in a class variable
            self._current_xy_zoom_start = [pos.x(), pos.y()]
            event.accept()

        except IndexError:
            self.log.debug('No image yet')


    def xy_scan_end_zoom_point(self, event):
        """ Get the mouse coordinates if the mouse button was released.

        @param QEvent event:
        """
        try:
            if self._mw._doubleclicked:
                self._mw._doubleclicked = False
                event.ignore()
                return

            # catch the event if the zoom mode is activated and if the event is
            # coming from a left mouse button.
            if not (self._mw.action_zoom.isChecked() and (event.button() == QtCore.Qt.LeftButton)):
                event.ignore()
                return

            # get the ViewBox which is also responsible for the xy_image
            viewbox = self.xy_image[self.curri].getViewBox()

            # Map the mouse position in the whole ViewWidget to the coordinate
            # system of the ViewBox, which also includes the 2D graph:
            pos = viewbox.mapSceneToView(event.localPos())
            endpos = [pos.x(), pos.y()]

            initpos = self._current_xy_zoom_start

            #print(initpos)

            # get the right corners from the zoom window:
            if initpos[0] > endpos[0]:
                xMin = endpos[0]
                xMax = initpos[0]
            else:
                xMin = initpos[0]
                xMax = endpos[0]

            if initpos[1] > endpos[1]:
                yMin = endpos[1]
                yMax = initpos[1]
            else:
                yMin = initpos[1]
                yMax = endpos[1]

            # # set the values to the InputWidgets and update them
            # self._mw.x_min_InputWidget.setValue(xMin)
            # self._mw.x_max_InputWidget.setValue(xMax)
            # self.change_x_range()
            #
            # self._mw.y_min_InputWidget.setValue(yMin)
            # self._mw.y_max_InputWidget.setValue(yMax)
            # self.change_y_range()

            # Finally change the visible area of the ViewBox:
            event.accept()
            viewbox.setRange(xRange=(xMin, xMax), yRange=(yMin, yMax), update=True)
            # second time is really needed, otherwisa zooming will not work for the first time
            viewbox.setRange(xRange=(xMin, xMax), yRange=(yMin, yMax), update=True)
            self.update_roi_xy()
            self._mw.action_zoom.setChecked(False)
        except IndexError:
            self.log.debug('No image')

    def reset_xy_imagerange(self):
        """ Reset the imagerange if autorange was pressed.

        Take the image range values directly from the scanned image and set
        them as the current image ranges.
        """
        # extract the range directly from the image:
        xMin = self._scanning_logic.spx_x_range[0]
        yMin = self._scanning_logic.spx_y_range[0]
        xMax = self._scanning_logic.spx_x_range[1]
        yMax = self._scanning_logic.spx_y_range[1]

        # self._mw.x_min_InputWidget.setValue(xMin)
        # self._mw.x_max_InputWidget.setValue(xMax)
        # self.change_x_range()
        #
        # self._mw.y_min_InputWidget.setValue(yMin)
        # self._mw.y_max_InputWidget.setValue(yMax)
        # self.change_y_range()

    def set_full_scan_range_xy(self):
        #xMin = self._scanning_logic.scan_x_range[0]
        xMax = self._scanning_logic.scan_x_range[1]
        #self._mw.x_min_InputWidget.setValue(xMin)
        self._mw.x_max_InputWidget.setValue(xMax)
        #self.change_x_image_range()

        #yMin = self._scanning_logic.scan_y_range[0]
        #yMax = self._scanning_logic.scan_y_range[1]
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

        # set the values to the InputWidgets and update them
        # self._mw.x_min_InputWidget.setValue(xMin)
        # self._mw.x_max_InputWidget.setValue(xMax)
        # self.change_x_range()

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
        xMin = self._scanning_logic.depth_image[0, 0, 0]
        zMin = self._scanning_logic.depth_image[0, 0, 2]
        xMax = self._scanning_logic.depth_image[-1, -1, 0]
        zMax = self._scanning_logic.depth_image[-1, -1, 2]

        # self._mw.x_min_InputWidget.setValue(xMin)
        # self._mw.x_max_InputWidget.setValue(xMax)
        # self.change_x_range()

        #s#elf._mw.z_min_InputWidget.setValue(zMin)
        #self._mw.z_max_InputWidget.setValue(zMax)
        #s#elf.change_z_image_range()

    def set_full_scan_range_z(self):

        if self._scanning_logic.depth_img_is_xz:
            hMin = self._scanning_logic.x_range[0]
            hMax = self._scanning_logic.x_range[1]
            # self._mw.x_min_InputWidget.setValue(hMin)
            # self._mw.x_max_InputWidget.setValue(hMax)
            # self.change_x_image_range()
        else:
            hMin = self._scanning_logic.y_range[0]
            hMax = self._scanning_logic.y_range[1]
            # self._mw.y_min_InputWidget.setValue(hMin)
            # self._mw.y_max_InputWidget.setValue(hMax)
            # self.change_y_image_range()

        vMin = self._scanning_logic.z_range[0]
        vMax = self._scanning_logic.z_range[1]
        self._mw.z_min_InputWidget.setValue(vMin)
        self._mw.z_max_InputWidget.setValue(vMax)
        self.change_z_image_range()

        for i in range(2):
            self.depth_image.getViewBox().setRange(xRange=(hMin, hMax), yRange=(vMin, vMax), update=True)


    def _set_scan_icons(self):
        """ Set the scan icons depending on whether loop-scan is active or not
        """

        if self._scanning_logic.permanent_scan:
            self._mw.action_scan_xy_start.setIcon(self._scan_xy_loop_icon)
            self._mw.action_scan_depth_start.setIcon(self._scan_depth_loop_icon)
        else:
            self._mw.action_scan_xy_start.setIcon(self._scan_xy_single_icon)
            self._mw.action_scan_depth_start.setIcon(self._scan_depth_single_icon)

    def logic_started_scanning(self, tag):
        """ Disable icons if a scan was started.

            @param tag str: tag indicating command source
        """
        if tag == 'logic':
            self.disable_scan_actions()

    def logic_continued_scanning(self, tag):
        """ Disable icons if a scan was continued.

            @param tag str: tag indicating command source
        """
        if tag == 'logic':
            self.disable_scan_actions()

    def logic_started_refocus(self, tag):
        """ Disable icons if a refocus was started.

            @param tag str: tag indicating command source
        """
        if tag == 'logic':
            self.disable_scan_actions()

