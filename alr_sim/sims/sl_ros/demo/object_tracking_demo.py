import time

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.sl_ros.RosObject import RosObject
from alr_sim.sims.sl_ros.RosScene import RosScene

cams = []
# Create a ROS Camera
cam = SimRepository.get_factory("sl_ros").create_camera(
    "1",
    rgb_topic="/cam_1/rgb/image_raw",  # "/cam_1/rgb/image_rect_color"
    depth_topic="/cam_1/depth/image_raw",
    ir_topic="/cam_1/ir/image_raw",
    cam_params_topic="/cam_1/rgb/camera_info",
    bgr_order=True,
)
cams.append(cam)

# Define key parameters for an object like name and dimensions
name = "lego_box"
object_dims = [0.032, 0.032, 0.064]

# If pose estimation with respect to the robot base is desired,
# the path to the folder containing calibration files of the cameras is required
# if no path is given the pose estimation is with respect to the camera
calib_path = "/home/alr_admin/catkin_ws/"

# Define for which camera a calibration file should be used,
# otherwise pose estimation again with respect to the camera
calibrated_cameras = [1]

# Create a RosObject
ros_object = RosObject(
    name=name,
    object_dims=object_dims,
)
# Add all objects to list
ros_objects = []
ros_objects.append(ros_object)

# Create a ROS Scene
scene = RosScene(
    ros_objects, cam_list=cams, calib_path=calib_path, calib_cams=calibrated_cameras
)

Acc_Time = 0.0
loop_times = 500
for _ in range(loop_times):
    # Get position and rotation (as quaternon) of the RosObject
    startTime = time.time()
    # tvec = scene.get_obj_pos(ros_object)
    # quaternion = scene.get_obj_quat(ros_object)
    time.sleep(0.03)
    tvec, quaternion = scene._get_obj_pos_and_quat(None, ros_object)
    endTime = time.time()

    howMuchTime = endTime - startTime
    Acc_Time += howMuchTime

    print("-" * 20)
    print("updated position: " + str(scene.position_updated()))
    print("object: " + ros_object.get_name())
    print("quaternion: " + str(quaternion))
    print("translation: " + str(tvec))
    print("-" * 20)

print("Average tracking time:" + str(Acc_Time / loop_times))
