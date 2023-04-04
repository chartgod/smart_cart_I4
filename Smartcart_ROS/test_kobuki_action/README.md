# 실행

action 실행

    $ rosrun test_kobuki_action action_client


# src

### 1. A_B_action_client.cpp
특정 좌표 이동
* 좌표 : (x , y)
    * A : (6.0, 0.5)
    * B : (6.5, 5.5)
    * C : (2.0, 5.8)
    * D : (0.0, 0.0) 원점

* 이동 정지

    $ rostopic pub /move_base/cancelctionlib_msgs/GoalID <tab두번>

### 1. action_server.cpp
수정중