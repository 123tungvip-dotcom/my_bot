# Hướng dẫn chạy

```bash
# Step 0: Chuẩn bị
# Sau khi tải pkg từ GitHub, đứng ở thư mục lưu pkg để chạy

# Step 1: Build môi trường
colcon build --symlink-install

# Step 2: Mở Gazebo và RViz
source install/setup.sh
ros2 launch my_bot launch_sim.launch.py world:=worlds/<điền_tên_map>
rviz

# Kích hoạt vật thể động
# Map_1:
python3 scripts/obstacle_mver.py
# Map_2:
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/plugins
ros2 launch my_bot launch_sim.launch.py world:=worlds/<điền_tên_map>

# Step 3: Xây dựng bản đồ với SLAM Toolbox
# Chỉnh ./config/mapper_params_online_async.yaml thành "mode: mapping"
source install/setup.sh
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p map_file_name:=<thay tên map file> -p map_start_at_dock:=true --params-file ./config/mapper_params_online_async.yaml

# Step 4: Localization sau khi có bản đồ
# Chỉnh ./config/mapper_params_online_async.yaml thành "mode: localization"
source install/setup.sh
ros2 launch my_bot localization_launch.py map:=<đường dẫn đến file yaml> use_sim_time:=true
# Sau đó set pose ban đầu cho robot

# Step 5: Điều hướng robot với RViz
source install/setup.sh
ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subcribe_transient_local:=true
# Bây giờ có thể set navigation goal cho robot

# Step 6: Chạy script tự động để lưu dữ liệu 10 lần di chuyển robot
python3 result_<điền số thứ tự map>/automated_test.py
