# Object Tracking

## Requirements

If you want to track real-world objects with ArUco markers in SimulationFramework, you’ll need the following :

1. A list of ROSCamera objects for every camera you intend using.
2. An identifier for the object.
3. The object dimensions in [x, y, z].
4. The ArUco marker size.
5. ArUco markers with 5X5 encoding.
6. An id dictionary specifying where the markers are located and the associated marker ids (the internal transformations are all relative to a defined object front).
7. (Optional): In case you are interested in retrieving the position of an object with respect to the robot base, the directory of saved calibration data (from  camera to robot base) is also required. The calibration files must be named "calibration_cam{#id}.yaml"

## Object Tracking Demo

In order to track objects in your projects, you have to do the following:

- Source ROS and activate conda environment:
    ```python
    source ./catkin_ws/devel/setup.bash 
    conda activate alr_sim
    ```

- Start up cameras and run aruco_ros node for respective camera and object to track:
    ```python
    roslaunch hl2_teleop azure_cam1.launch 
    alr_admin@CameraHololensPC:~$ roslaunch aruco_ros cam1_board_lego.launch 
    ```

- Import the required classes:
    
    ```python
    from alr_sim.sims.SimFactory import SimRepository
    from alr_sim.sims.sl_ros.RosObject import RosObject
    from alr_sim.sims.sl_ros.RosScene import RosScene
    ```
    

- Create a list of RosCamera objects:
    
    ```python
    cams = []
    cam = SimRepository.get_factory("sl_ros").create_camera(
        "1",
        rgb_topic="/cam_1/rgb/image_raw",  # "/cam_1/rgb/image_rect_color"
        depth_topic="/cam_1/depth/image_raw",
        ir_topic="/cam_1/ir/image_raw",
        cam_params_topic="/cam_1/rgb/camera_info",
        bgr_order=True,
    )
    cams.append(cam)
    ```
    

- Define object characteristics:
    
    ```python
    name = "lego_box"
    object_dims = [0.032, 0.032, 0.064]

    calib_path = "/home/alr_admin/catkin_ws/"
    calibrated_cameras = [1]
    ```
    
    Note: Object dimensions are given as meters. Aditionally, specifying the path to the calibration path and which cameras are calibrated is required if you are interested in object positions with respect to the robot base. Otherwise, the retrieved positions are with respect to the used cameras. The calibration path points to the directory with saved .yaml files containing each camera's translation vector and quaternion. If you have camera calibration files, you can specify a list for which cameras you like to use the calibration data.
    
- Create RosObjects and append them to an object list:
    
    ```python
    ros_object = RosObject(name=name,object_dims=object_dims)
    ros_objects = []
    ros_objects.append(ros_object)
    ```

- Create a RosScene:
    ```python
    scene = RosScene(ros_objects, cam_list=cams, calib_path=calib_path, calib_cams=calibrated_cameras)
    ```
    
- Retrieve object position:
    
    ```python
    tvec, quaternion = scene._get_obj_pos_and_quat(None, ros_object)
    ```
    
    Calling **`scene._get_obj_pos_and_quat()`** returns a translation vector and quaternion (in [w, x, y, z] notation) if an object is detected.                                                                 Otherwise, the function returns **`None`** for both values.

Runnig the provided demo script results the following output:
```console
(alr_sim) alr_admin@CameraHololensPC:~/robot/SimulationFramework/alr_sim/sims/sl_ros/demo$ python object_tracking_demo.py 
pybullet build time: Jan 28 2022 20:17:22
Listening for data...
--------------------
object: lego_block
quaternion: [0.8538496822154071, 0.3864097929030045, -0.21674315798545576, 0.27322261179448043]
translation: [ 0.44488652 -0.00106412 -0.00284662]
--------------------
--------------------
object: lego_block
quaternion: [0.852748169357617, 0.396023022341091, -0.21512863863833262, 0.264018927864421]
translation: [ 0.44642597 -0.00229544 -0.00175167]
--------------------
--------------------
object: lego_block
quaternion: [0.5091166979103661, 0.48199695998281555, 0.4977743037592003, 0.5105877603245229]
translation: [ 0.43455686  0.00807682 -0.0107915 ]
--------------------
--------------------
object: lego_block
quaternion: [0.8549781085055891, 0.3859993466045252, -0.21008680686778075, 0.27546410288308826]
translation: [ 0.44760874 -0.00370686 -0.00069074]
--------------------
...
```
