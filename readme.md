# Go2 - Local Path Planner Algorithm Test Simulator 개발
- Go2 로봇을 Gazebo 환경에서, 다양한 Local Path Planner Algorithm을 검증하기 위해, 테스트 시뮬레이터 개발

# 진행한 일
- Go2 & Gazebo 환경 로봇 업로드
- ...

# 해야할 일
- 센서 배치 
    - 다른 센서 고려
    - velodyne lidar
    - 리복스 mid360
    - 아우스터 ouster
    - hesai 
    - depth camera (d455)
        - go2에 적용 

- planner 알고리즘 수정
    - 장애물 관련 설정 추가 필요 

- costmap 계산 수정 

- 고려
    - 최소 거리 측정해서 logger 찍기 (장애물)
    - velodyne lidar 관련하여 확인하기 
        - lidar 거리에서 최소거리 파악 
        - lidar callback 수정


## 전체적으로 logger를 찍어서 디버깅을 해보자 ! 


# 실행 코드
ros2 launch go2_config gazebo_velodyne.launch.py 

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch local_path_planner integrated_planner_launch.py

# 참고 자료

https://github.com/anujjain-dev/unitree-go2-ros2

https://www.youtube.com/watch?v=PjWvf90l4cg / https://arxiv.org/abs/2306.14874

https://github.com/abizovnuralem/go2_omniverse

https://github.com/VincidaB/pfe


# 한거
- logger 찍어서 하나씩 디버깅
- velodyne lidar -> frame_id 변환하기

# 문제해결 
- costmap의 data가 0이 나옴 
- goal로 지금 변환이 안됨 / path_planner.py 수정해야함
- planner 의 속도값 확인 / cmd_vel 속도 올리기
- costmap이 scan 속도 만큼 나오지 않는 문제 

# 1/2 일 해야할 일 ! 
-> 지금, costmap 변환 테스트 진행중, 이후에 cmd_vel이 0이 나옴, 이쪽 문제를 해결해야함
-> 가제보 관련 에러도 해결하자 ! 


# Reference 찾기 
- nav2 / 3d planning 가능한 형태 - local planner / local costmap만 있으면 가능한 정도 
    - leg, 사족 , wheel    / quadruped planning
    - 다른 시뮬레이터나, 다른 쪽도 고려 / ros2 


https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning
-> nav2 기반

https://github.com/DrWillway/rescue_robots_sim/tree/4feb46aaafbcae1a5a3b5b8f06742952c74a0602
-> 드론 / nav2 기반

https://soohwan-justin.tistory.com/40
-> nav2 기반 / turtlebot3

https://github.com/strawlab/navigation/tree/master
-> ros1 기반의 naviagation pkg 기반

https://github.com/Project-MANAS/xdwa_local_planner
-> c++ 기반 / nav2 사용하지 않은 것 같기도 함

https://github.com/AmeyDeshpande97/Motion-planning-for-an-Autonomous-Vehicles-under-Dynamic-Obstacle-Uncertainty
-> nav2 사용하지 않음 - ros2 아닌, pygame 기반 / motion planning of an autonomous vehicle under uncertainty 논문
-> rrt*를 글로벌 플래너 (최적의 경로 반환)-> dwa algorithm + 로컬 플래너로 구현 (불확실한 동적 이동 장애물을 성공적을 피하기)
    -> dwa에 odom + rrt*에서 제공한 글로벌 경로의 일부 + 장애물 정보 (pointcloud)