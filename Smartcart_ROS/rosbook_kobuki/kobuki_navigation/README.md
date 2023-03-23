# 실행

* 가상환경 navigation 실행

    $ roscore
    $ roslaunch testbot_gazebo kobuki.launch
    $ roslaunch kobuki_navigation kobuki_navigation.launch


* 실제환경 navigation 실행
    * gl lidar 연결

        $ sudo chmod 777 /dev/ttyUSB1
        $ roslaunch gl_ros_driver gl_ros_driver.launch

    $ roslaucnh kobuki_node minimal.launch
    $ roslaunch kobuki_navigation kobuki_navigation.launch
    
* 런치 파일에 추가
    
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find kobuki_navigation)/rviz/kobuki_nav.rviz"/>

~~$ rosrun rviz rviz -d `rospack find kobuki_navigation`/rviz/kobuki_nav.rviz~~


# error

### 1. $ roslaunch kobuki_navigation kobuki_navigation.launch

    Warning: Invalid argument "/map" passed to canTransform argument target_frame in tf2 frame_ids cannot start with a '/' like: at line 134 in /tmp/binarydeb/ros-melodic-tf2-0.6.5/src/buffer_core.cpp

* sol

param/global_costmap_params.yaml, local_costmap_params.yaml

    global_frame: /map
    robot_base_frame: /base_footprint 

수정: '/'제거

    global_frame: map
    robot_base_frame: base_footprint

