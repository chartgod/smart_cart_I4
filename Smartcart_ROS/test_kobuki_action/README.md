# 실행

action 실행

    $ rosrun test_kobuki_action action_client


# src

### 1. A_B_action_client.cpp
특정 좌표 이동
<img width="50%" src="https://cafeptthumb-phinf.pstatic.net/MjAyMzA0MDRfMTgy/MDAxNjgwNTg1NjIzNjcy.LjRG_hCai8s3dCLndQ-ysIKYg_i0mbHbdFjLW9Wsn-kg.rX4XAfCS8pFVJcaVDyoXl9K1PAGxSpBi0T0yiMgRdYgg.PNG/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%2C_2023-04-04_14-14-38.png?type=w1600"/>
<img width="50%" src="https://cafeptthumb-phinf.pstatic.net/MjAyMzA0MDRfNjQg/MDAxNjgwNTgxNTQ2MjE3.VRlX8l-LCanrUw0z7fpPP0fy8WlmlAijuZmKg2-E_NUg.u7_kwihCBZbkhoE_FY5quk3YW51Q7-jaZcj-JNEUkPcg.JPEG/1680581540212.jpg?type=w1600"/>
* 좌표 : (x , y)
    * A : (6.0, 0.5)
    * B : (6.5, 5.5)
    * C : (2.0, 5.8)
    * D : (0.0, 0.0) 원점

* 이동 정지

    $ rostopic pub /move_base/cancelctionlib_msgs/GoalID <tab두번>

### 1. action_server.cpp
수정중