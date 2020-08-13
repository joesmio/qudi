


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

        self.power_min = 0.01
        self.power_max = 0.16
        self.points = 31

        self.initial_power = 0.03

        self.int_time = 3

        self.values = np.linspace(self.power_min, self.power_max, num = self.points )

        # x: Laser power, y: counts

        self.power_vector = np.zeros(np.shape(self.values))

        self.count_vector = np.zeros(np.shape(self.values))

        self.sigAnother.connect(self.do_measurement_point)

        self.initial_current = 37

        self.stopRequested = False

    def update_param(self, start, end, points, int_time):
        self.power_min = start
        self.power_max = end
        self.points = points
        self.int_time = int_time


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

        self.values = np.linspace(self.power_min, self.power_max, num=self.points )

        # x: Laser power, y: counts

        self.power_vector = []#np.zeros(np.shape(self.values))

        self.count_vector = []#np.zeros(np.shape(self.values))

        self.point = 0

        self._counting_device.set_up_counter()

        #self.initial_current = float(self._laser.get_current())

        self._laser.set_power(self.values[0])
        time.sleep(3)

        self.sigAnother.emit()


    def do_measurement_point(self):
        i = self.point
        power = self.values[i]
        print('Setting laser power to {0}'.format(power))
        self._laser.set_power(power*1e3) # to mW
        self.remaining_time = self.int_time
        counts = []
        while self.remaining_time > 0.2:
            time.sleep(0.1)
            counts.append(float(self._counting_device.get_counter()[[0]])+float(self._counting_device.get_counter()[[1]]))
            #print(counts)
            self.remaining_time = self.remaining_time - 0.2

        self.count_vector.append(np.mean(counts))
        self.power_vector.append(power)
        # print('current {0} power {1} counts {2}'.format(current,self.power_vector[i],self.count_vector[i]))
        self.point = self.point + 1
        self.sigImageUpdated.emit()

        if (self.point > len(self.values)-1) or self.stopRequested is True:
            self.point = 0
            self._counting_device.close_counter()
            self.log.info('Saturation measurement ended, resetting power to {0:.2f} mW'.format(self.initial_power*1e3))
            self._laser.set_power(self.initial_power)
            self.stopRequested = False
            return
        else:
            self.sigAnother.emit()

