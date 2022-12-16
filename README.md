# 좌표 변환 모듈 사용법
##빌드 환경 : Ubuntu 22.04, ROS2 Humble
###다음 순서대로 터미널에 입력할 것
---

    sudo apt update && sudo apt upgrade
    sudo apt install ros-humble-moveit
    sudo apt install ros-humble-moveit-*
    git clone https://github.com/JeonB/transform.git
    cd transform
    colcon build --mixin release
    source ~/transform/install/local_setup.bash
    
    //터미널 1
    ros2 launch quick_start demo.launch
    
    //터미널 2
    ros2 run move_coordinate move_coordinate
    
    
    
---
시뮬레이션이 작동하다가 멈추거나 joint값이 제대로 안 나올 시 왼쪽 하단의 reset 버튼 누를 것
    
