
## Dependencies
 **Ubuntu**

 **ROS 2** ex :  Humble.

## setup-micro

This is about getting your microcontroller project set up with micro-ROS.

install pio core 
    [PlatformIO Core Installation](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html#super-quick-macos-linux)

you may want to check out their official setup guide
    [micro-ROS PlatformIO GitHub](https://github.com/micro-ROS/micro_ros_platformio)

clone this repo

build the library 

```
pio lib install
pio run         
pio run --target upload
```

select the environment

## setup-agent
install ros humble
```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash
```
Build the Agent

```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```
run  the agent.

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --agent-ip 192.168.x.x
```

## test

```bash
ros2 topic list

ros2 topic info /micro_ros_platformio_node_publisher #for default hello-world node

ros2 topic echo /imu/data

rqt_plot /imu/data/orientation/x:y:z
#or use
rviz2
```

## custom Msgs


1.  **Go to ROS 2 Workspace:**

    ```bash
    cd ~/your_ros2_ws/src
    ```


2.  **New Package for Messages:**

    ```bash
    ros2 pkg create custom_msgs --build-type ament_cmake --dependencies rosidl_default_generators std_msgs
    ```

3.  **Define  Message File**

    ```bash
    cd custom_msgs
    mkdir msg
    touch msg/ImuInfo.msg
    ```

    Open `msg/ImuInfo.msg` and add (example):

    ```
    std_msgs/Header header

    float64 roll
    float64 pitch
    float64 yaw
    ```

4.  **Edit `CMakeLists.txt`:**
     `custom_msgs/CMakeLists.txt` and add these lines

    ```cmake
    find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/ImuInfo.msg"
      DEPENDENCIES std_msgs
    )

    ament_export_dependencies(rosidl_default_runtime std_msgs)
    ament_package()
    ```

5.  **Edit `package.xml`:**
     `custom_msgs/package.xml` and add 

    ```xml
    <buildtool_depend>ament_cmake</buildtool_depend>
    <build_depend>rosidl_default_generators</build_depend>
    <build_depend>std_msgs</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

6.  **Build Custom Message Package**

    ```
    colcon build --packages-select custom_msgs
    ```

7.  **copy to project**

    copy the ws that built the custom msgs into `extra_packages` directory, which is usually :
        `.pio/libdeps/<pio_env>/micro_ros_platformio/extra_packages`

    add this to extra_packages.repos
    ```
    custom_msgs:
    type: local
    path: custom_msgs
    version: humble
    ```
    copy that `extra_packages` folder itself into main PlatformIO project's root directory

8.  **Rebuild the Library:**

    ```
    pio run 
    ```

## Using Custom Messages


```c++
#include <custom_msgs/msg/imu_info.h> 

rcl_publisher_t euler_pub;

custom_msgs__msg__ImuInfo euler_msg;

euler_msg.roll  = roll_value;  
euler_msg.pitch = pitch_value; 
euler_msg.yaw   = yaw_value; 

// BTW, you can find the exact struct definition  here:
// .pio/libdeps/h743-micro-ros-test/micro_ros_platformio/libmicroros/include/custom_msgs/msg/detail/imu_info__struct.h

// ... then you'd publish it, something like RCLC_PUBLISH(&euler_pub, &euler_msg);
