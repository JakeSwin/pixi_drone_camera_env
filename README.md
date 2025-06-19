# README

To integrate into other systems just copy and paste the src/uav_image folder into your ros2 package directory and build

Requires ffmpeg to be installed and accessable on the system. Along with the following ros2 packages

- cv_bridge
- image_transport
- sensor_msgs
- OpenCV

## Install

- Make sure you have pixi installed: https://pixi.sh/dev/installation/
- run `pixi install` in the project root
- install ffmpeg with your system package manager e.g: `sudo apt install ffmpeg`
- compile with `pixi run build`
- run with `pixi run run`

## ROS2 Topics

### uav_image package (C++)

#### uav_image node

- Topic: `/uav_image`
    - Message type: `sensor_msgs/msg/image`
    - CvImage in BGR8 format.
- Topic: `/uav_camera`
    - Message type: `sensor_msgs/msg/camera_info`
    - Data:
        - `height`: in pixels
        - `width`: in pixels 
        - `d`: distortion (list length 5), plumb bob model
        - `k`: intrinsic matrix, 3x3 row-major
        - `p`: projection matrix, 3x4
    - The matrices can be used to calculate FOV etc.


### TODO package

- Topic: `/uav_gps`
    - Message type: `sensor_msgs/msg/NatSatFix`
    - Data:
        - `latitude`
        - `longitude`
        - `altitude`
    - Altitude may be inaccurate. If this is the case we will spoof the number and set the drone altitude manually.
- Topic: `/uav_direction`
    - Message type: `std_msgs/msg/Float32`
    - Data:
        - `data`: Float32. A heading has a range of [-180, 180] degrees, where 0 represents True North. 

### map_publisher package

#### map_publisher node (Python)

- Topic: `/field_boundary`
    - Message type: `geometry_msgs/msg/Polygon` (array of `Point32`)
    - Data:
    - `x`: latitude
    - `y`: longitude
    - `z`: unused, default to 0


