# osp_explorer

TurtleBot4 ArUco 마커 인식 (OSP 기말 프로젝트 6조)

## 설치

```bash
mkdir -p ~/osp_ws/src && cd ~/osp_ws/src
git clone https://github.com/DAEDUN/OSP_EXPLORER.git osp_explorer
cd ~/osp_ws
colcon build --symlink-install
source install/setup.bash
```

## 실행

로봇 연결 후 터미널 4개를 열고, 각 터미널에서 `source ~/osp_ws/install/setup.bash`

```bash
# 1. SLAM
ros2 launch turtlebot4_navigation slam.launch.py

# 2. RViz (Add → By topic → /aruco_markers 추가)
ros2 launch turtlebot4_viz view_navigation.launch.py

# 3. 마커 인식
ros2 run osp_explorer marker_mapper

# 4. 키보드로 로봇 이동
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

teleop 키: `i` 앞 / `,` 뒤 / `j` `l` 회전 / `k` 정지 / `z` 속도 낮추기

마커가 인식되면 3번 터미널에 `*** FOUND marker ID n`이 뜨고 RViz 지도에 표시됩니다.
결과는 `~/osp_result/markers.csv` (id, x, y)에 저장됩니다.
