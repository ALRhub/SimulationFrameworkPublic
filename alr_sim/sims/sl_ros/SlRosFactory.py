import alr_sim.sims.SimFactory as Sims
from alr_sim.sims.sl.SlFactory import SlFactory
from alr_sim.sims.sl_ros.RosCamera import RosCamera


class SlRosFactory(SlFactory):
    def create_camera(self, *args, **kwargs):
        return RosCamera(*args, **kwargs)


Sims.SimRepository.register(SlRosFactory(), "sl_ros")
