Hướng dẫn chạy
Step 0: Sau khi tải pkg từ github, đứng ở thư mục lưu pkg để chạy
Step 1: Build môi trường
  $colcon build --symlink-install

Step 2: Mở gazebo và rviz
  $source install/setup.sh
  $ros2 launch my_bot launch_sim.launch.py world:=worlds/<điền_tên_map>
  $rviz
Để kích hoat vật thể đông:
Map_1: Mở một terminal mới và chạy lệnh: $python3 scripts/obstacle_mver.py
Map_2: Trước chạy $ros2 launch my_bot launch_sim.launch.py world:=worlds/<điền_tên_map>, phải chạy lệnh export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/plugins


Step 3: Xây dựng bản đồ với SLAM toolbox
Chỉnh chế độ chạy trong ./config/mapper_params_online_async.yaml thành "mode:mapping". Sau đó chạy lệnh ở một terminal mới
  $source install/setup.sh
  $ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p map_file_name:=<thay tên map file> -p map_start_at_dock:=true --params-file ./config/mapper_params_online_async.yaml

Step 4: Chạy localiztion sau khi có bản đồ
Chỉnh chế độ chạy trong ./config/mapper_params_online_async.yaml thành "mode:localization". Sau đó chạy lệnh ở một terminal mới
  $source install/setup.sh
  $ros2 launch my_bot localization_launch.py map:=<đường dẫn đến file yaml> use_sim_time:=true
Sau đó set pose ban đầu cho robot

Step 5: Điều hướng robot với rviz
Mở một terminal và chạy lệnh
  $source install/setup.sh
  $ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subcribe_transient_local:=true
Bây giờ bạn đã có thể set navigation goal cho robot. 

Step 6: Chạy script tự động để lưu lại dữ liệu 10 lần di chuyển robot
  $python3 result_<điền số thứ tự map>/automated_test.py
