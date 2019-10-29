


from qtpy import QtCore
import numpy as np
import time

from logic.generic_logic import GenericLogic
from core.module import Connector, ConfigOption, StatusVar
from core.util.mutex import Mutex


class saturationLogic(GenericLogic):

    _modclass = 'saturationlogic'
    _modtype = 'logic'

    # declare connectors
    counter = Connector(interface='SlowCounterInterface')
    fitlogic = Connector(interface='FitLogic')
    laserhardware = Connector(interface='LaserInterface')

    sigImageUpdated = QtCore.Signal()

    sigAnother = QtCore.Signal()


    # Connect counter hardware, laser hardware
    def on_activate(self):
        self._counting_device = self.get_connector('counter')
        self._fit_logic = self.get_connector('fitlogic')
        self._laser = self.get_connector('laserhardware')

        self.current_min = 30
        self.current_max = 60
        self.current_step = 1
        self.n_trial = int(3)

        self.values = np.arange(self.current_min, self.current_max, self.current_step)

        # x: Laser power, y: counts

        self.power_vector = np.zeros(np.shape(self.values))

        self.count_vector = np.zeros(np.shape(self.values))

        self.sigAnother.connect(self.do_measurement_point)

        self.initial_current = 30


    # Start saturation measurement
    # Set current 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100
    # At each point, take 5 power readings and 5 counts readings

    def on_deactivate(self):
        pass

    def on_measurement(self):

        if self._laser.connected is False:
            status = self._laser.connect_laser()
            if status is False:
                self.log.info('Cannot connect to laser. Is it in use?')
                return 0

        self.values = np.arange(self.current_min, self.current_max, self.current_step )

        # x: Laser power, y: counts

        self.power_vector = []#np.zeros(np.shape(self.values))

        self.count_vector = []#np.zeros(np.shape(self.values))

        self.point = 0

        self._counting_device.set_up_counter()

        self.initial_current = float(self._laser.get_current())


        self._laser.set_current(self.values[0])
        time.sleep(3)

        self.sigAnother.emit()


    def do_measurement_point(self):
        i = self.point
        current = self.values[i]
        self._laser.set_current(current)
        self.remaining_time = 5
        while self.remaining_time > 0.2:
            time.sleep(0.1)
            self._counting_device.get_counter()
            self.remaining_time = self.remaining_time - 0.2

        self.power_vector.append(float(self._laser.get_power()))
        self.count_vector.append(float(self._counting_device.get_counter()[[0]]))
        # print('current {0} power {1} counts {2}'.format(current,self.power_vector[i],self.count_vector[i]))
        self.point = self.point + 1
        self.sigImageUpdated.emit()

        if self.point > len(self.values)-1:
            self.point = 0
            self._counting_device.close_counter()
            self.log.info('Saturation measurement ended, resetting current to {0} %'.format(self.initial_current))
            self._laser.set_current(self.initial_current)
            return
        else:
            self.sigAnother.emit()

