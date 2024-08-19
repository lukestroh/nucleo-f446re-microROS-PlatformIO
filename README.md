# nucleo-f446re-microROS-PlatformIO

## Adding custom messages to the project
Clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils into your project. Navigate to `<proj>/micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/`.

Recommended: Add your custom message package to extra_packages.repos. Alternatively, packages can be added directly to this directory (not recommended).

## Building `micro_ros_stm32cubemx_utils`

Follow the directions [here](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils).

## Building extra_packages
```
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```