import matplotlib.pyplot as plt

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils.point_clouds import PcVisualizer

# Create ROS Camera
cam = SimRepository.get_factory("sl_ros").create_camera(
    "kinect",
    rgb_topic="/cam_1/rgb_to_depth/image_raw",  # "/cam_1/rgb/image_rect_color"
    depth_topic="/cam_1/depth/image_raw",
    ir_topic="/cam_1/ir/image_raw",
    cam_params_topic="/cam_1/rgb/camera_info",
    bgr_order=True,
)
# pcl_topic="/cam_1/points2")

"""
cam = SimRepository.get_factory("sl_ros").create_camera(
    "realsense",
    rgb_topic="/cam_4/color/image_raw",
    depth_topic="/cam_4/depth/image_rect_raw"
)
"""

rgb, d = cam.get_image()
ir = cam.get_ir_image()

K, D = cam.get_cam_params()
print("K: " + str(K))
print("D: " + str(D))

plt.subplot(121)
plt.imshow(rgb)
plt.subplot(122)
plt.imshow(d)
plt.show()
print("finished")
plt.imshow(ir)
plt.show()

pcv = PcVisualizer()
pcv.visualize_point_clouds(*cam.calc_point_cloud())
