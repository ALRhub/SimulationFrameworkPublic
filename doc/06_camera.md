# 6. Use Camera for Perception

Our Scenes come with two predefined cameras:
- `rgbd` - an RGB-D camera on the robot's end-effector
- `rgbd_cage` - an RGB-D camera in an over-the-shoulder position

Cameras are RGB-D Cameras, meaning you can also request the depth data for each image.
As an alternative to RGB color, you can also use the `get_segmentation()` function, which will return image data as if a pixel-perfect object segmentation has been performed.

To add a new Camera to your Scene, load it as if it were any other object:

```python
new_cam = Camera(...)
scene.add_object(new_cam)
scene.start()


new_cam.get_image(...)
```
## Adjust the camera parameters
To adjust camera parameters like image-width and -height you can use the `set_cam_params(...)` method.
This also allows to configure deeper camera parameters.

In some functions like `calc_point_cloud` we use the camera parameters to calculate three dimensional positions. It is therefore not recommended to use custom widths and sizes with these functions. You should rather set these params before with `set_caram_params`. 

```python
new_cam = Camera(...)
scene.add_object(new_cam)

new_cam.set_cam_params(
    width,  # the width of the camera images
    height, # the height of the camera images
    near,   # the near plane for the depth image, in the raw depth image the depth values will use `near` as 0
    far,    # the far plane for the depth image, in the raw depth image the depth values will use `near` as 1
    fovy,   # The field of view of the camera in y-direction. fovx will then be calculated based on fovy and image dimensions
)

scene.start()


new_cam.get_image(...)


```

### 6.1 ROS Camera control
TBA ...
[Back to Overview](./)