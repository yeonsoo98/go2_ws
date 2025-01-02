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



# isaac gym - go2 강화학습
- go2 로봇 띄우기