# 좌표 변환 모듈 사용법
## 빌드 환경 : Ubuntu 22.04, ROS2 Humble
### 다음 순서대로 터미널에 입력
---

#### Moveit2, control, controllers 패키지 install 
    sudo apt update && sudo apt upgrade
    sudo apt install ros-humble-moveit
    sudo apt install ros-humble-moveit-*
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-ros2-controllers
    
---

#### 좌표변환 코드 클론 및 빌드
    git clone https://github.com/JeonB/transform.git
    cd transform
    colcon build --mixin release
    source ~/transform/install/local_setup.bash
    
    # 'source ~/transform/install/local_setup.bash'는 터미널1,2 모두 입력해야 합니다.
    # .bashrc 파일에 입력해서 매번 입력하는 번거로움을 피할 수 있습니다. 
    # gedit .bashrc
    
---

 #### 실행
    #터미널 1
    ros2 launch quick_start demo.launch.py
    
    #터미널 2
    ros2 run move_coordinate move_coordinate
    
---

#### 주의사항
시뮬레이션이 작동하다가 멈추거나 joint값이 제대로 안 나올 시 왼쪽 하단의 reset 버튼 누를 것
    
