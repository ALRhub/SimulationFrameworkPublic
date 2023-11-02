import logging

try:
    # import py_at_broker as pab
    import rospy

    from .SlRosFactory import *
except ImportError as e:
    logging.getLogger(__name__).info(e)
    logging.getLogger(__name__).info(
        "No SL or ROS installed. SL_ROS simulation and control is not available."
    )
    pass
