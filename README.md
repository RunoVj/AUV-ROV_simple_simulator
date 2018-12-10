# AUV-ROV_simple_simulator
Simple simulator based on Gazebo for testing control algorithms 

## Cameras
UV has 3 cameras:

- 2 front cameras (stereo_vision)
- 1 bottom camera

Cameras' topics:

	>/stereo/camera/left/image_raw
	>/stereo/camera/right/image_raw
	>/ROV_model_URDF/camera_bottom/image_raw

To run cameras write in terminal:

```sh
$ rosrun image_view image_view image:=/stereo/camera/left/image_raw
$ rosrun image_view image_view image:=/stereo/camera/right/image_raw
$ rosrun image_view image_view image:=/ROV_model_URDF/camera_bottom/image_raw
```

## Sensors
UV has IMU sensor 

## Custom Gazebo plugins
model_move_plugin was written to move UV. Through publishing twist messages you can control robots's planar movements and its hight. Odom message can inform you about robot's position, orientation and twist parameters.

## Will be added
UE4 integration 

## Will be fixed
Bad physics. Buoyancy and hydrodynamic plugins will be fixed.


