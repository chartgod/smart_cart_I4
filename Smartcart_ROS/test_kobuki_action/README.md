# 실행

action 실행

    $ rosrun test_kobuki_action action_client


# src

### 1. action_client.cpp
특정 좌표 이동
* 이동 정지

    $ rostopic pub /move_base/cancelctionlib_msgs/GoalID <tab두번>

### 1. action_server.cpp
수정중