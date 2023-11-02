import abc
import time

import numpy as np
import requests

from alr_sim.sims.mujoco.mj_interactive.devices import rt_device
from alr_sim.utils import geometric_transformation as geo_trans


class PHYPHOX_CODES:
    clear = "/control?cmd=clear"
    start = "/control?cmd=start"
    stop = "/control?cmd=stop"
    config = "/config"

    @staticmethod
    def get(keys):
        return "/get?" + "&".join(keys)


class PhyPhoxListener(rt_device.RtDeviceMonitor, abc.ABC):
    """
    Base class to handle connections with a PhyPhox App
    """

    def __init__(self, url, device_type="Phyphox App"):
        self.url = url
        self.sensor_raw = {}
        super(PhyPhoxListener, self).__init__("Phyphox App")

    def send_cmd(self, cmd):
        return requests.get(self.url + cmd).json()

    def load_cfg(self):
        cfg = self.send_cmd(PHYPHOX_CODES.config)
        exported_sensors = cfg["export"]
        for sensor_set in exported_sensors:
            for buffer in sensor_set["sources"]:
                buffer_name = buffer["buffer"]
                self.sensor_raw[buffer_name] = 0

    def monitor(self):
        self.send_cmd(PHYPHOX_CODES.clear)
        self.send_cmd(PHYPHOX_CODES.start)

        self.load_cfg()

        while True:
            x = self.send_cmd(PHYPHOX_CODES.get(self.sensor_raw.keys()))
            payload = x["buffer"]
            for k in self.sensor_raw:
                sensor_value = payload[k]["buffer"][0]
                if sensor_value is None:
                    sensor_value = 0
                self.sensor_raw[k] = sensor_value
            self.process_raw_sensors()

    def check_connection(self) -> bool:
        try:
            r = requests.head(self.url)
            return r.status_code == 200
        except ConnectionError:
            return False

    @abc.abstractmethod
    def process_raw_sensors(self):
        pass


class PhyPhoxIMU(PhyPhoxListener):
    def __init__(self, url):
        super(PhyPhoxIMU, self).__init__(url, "PhyPhox ALR Robot Remote")

        self.xPos = self.yPos = self.zPos = 0
        self.xRot = self.yRot = self.zRot = 0

        self.btnReset = self.btnPlot = 0

        self.gyrTime = self.linTime = 0
        self.speed = np.zeros(3)

    def process_raw_sensors(self):
        gyrDelta = self.sensor_raw["gyrT"] - self.gyrTime

        self.xRot += gyrDelta * self.sensor_raw["gyrX"]
        self.yRot += gyrDelta * self.sensor_raw["gyrY"]
        self.zRot += gyrDelta * self.sensor_raw["gyrZ"]

        acc_vec = np.array(
            [self.sensor_raw["linX"], self.sensor_raw["linY"], self.sensor_raw["linZ"]]
        )
        rot_mat = geo_trans.euler2mat([self.xRot, self.yRot, self.zRot])

        acc_vec = rot_mat @ acc_vec
        linDelta = self.sensor_raw["linT"] - self.linTime
        self.speed += acc_vec * linDelta

        self.xPos += self.speed[0] * linDelta
        self.yPos += self.speed[1] * linDelta
        self.zPos += self.speed[2] * linDelta

        self.btnReset = self.sensor_raw["btnReset"]
        self.btnPlot = self.sensor_raw["btnPlot"]
        self.gyrTime = self.sensor_raw["gyrT"]
        self.linTime = self.sensor_raw["linT"]


if __name__ == "__main__":
    p = PhyPhoxIMU("http://192.168.1.63:8080")
    while True:
        time.sleep(0.1)
