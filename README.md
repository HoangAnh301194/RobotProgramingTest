# B23DCDK007 — Panda Pick-and-Place trong Gazebo

**Sinh viên:** Nguyễn Hữu Hoàng Anh — **Mã SV:** B23DCDK007  
**Đề:** Xây dựng hệ thống Pick-and-Place trong Gazebo; sử dụng MoveIt 2 lập kế hoạch tiếp cận, gắp, vận chuyển; đồng bộ attach/detach với Planning Scene và Gazebo.

## Quyết định thiết kế

- **Dùng trực tiếp mô hình Panda do MoveIt cung cấp** (`moveit_resources_panda_description` và SRDF/config của `moveit_resources_panda_moveit_config`). Không viết lại URDF hoặc sửa mesh/khớp của robot.
- ROS 2 **Humble** + Ubuntu **22.04** + **Gazebo Classic 11** (chọn để tương thích IFRA LinkAttacher, plugin fixed-joint ROS 2 đã có cho Humble).
- Trong launch, Panda xacro nguyên bản được nạp rồi thêm phần **ros2_control và gazebo plugin vào XML ở bộ nhớ**, không tạo bản URDF Panda mới và không thay đổi tệp mô hình upstream.
- Khi gắp, vật thể được gắn vào `panda_hand` đồng thời trong Gazebo (fixed joint) và MoveIt Planning Scene (attached collision object). Khi thả, tháo cả hai và cập nhật vị trí vật thể đã đặt.

> **Trạng thái kiểm thử:** mã nguồn đã được bổ sung; cần build/chạy nghiệm thu trực tiếp trên máy Ubuntu ROS 2 Humble có Gazebo. Không tuyên bố đã chạy thành công trong môi trường mô phỏng.

## Cài đặt từ đầu

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/HoangAnh301194/RobotProgramingTest.git
bash RobotProgramingTest/scripts/install_deps.sh
cd ~/panda_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

*Lưu ý:* `scripts/install_deps.sh` dùng sudo/apt và clone dependency IFRA vào đúng thư mục `src` của workspace; kiểm tra script trước khi chạy.

## Chạy mô phỏng

**Terminal 1:**

```bash
source /opt/ros/humble/setup.bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_pick_place panda_gazebo.launch.py
```

**Terminal 2:**

```bash
source /opt/ros/humble/setup.bash
source ~/panda_ws/install/setup.bash
ros2 run panda_pick_place pick_place_node
```

Có thể cho tự chạy node sau khi khởi tạo (không thuận tiện khi gỡ lỗi):

```bash
ros2 launch panda_pick_place panda_gazebo.launch.py run_task:=true
```

## Kiểm tra trước khi chạy

```bash
ros2 node list
ros2 control list_controllers
ros2 action list | grep -E 'follow_joint_trajectory|gripper_cmd'
ros2 service list | grep -E 'ATTACHLINK|DETACHLINK'
ros2 topic echo /joint_states --once
ros2 service call /gazebo/get_model_state gazebo_msgs/srv/GetModelState "{model_name: pick_box, relative_entity_name: world}"
```

Kỳ vọng controllers `joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller` đều `active`; các service `/ATTACHLINK`, `/DETACHLINK` tồn tại; trong Gazebo có model `panda` và `pick_box`.

## Trình tự node

1. Đưa Panda về tư thế `ready` theo SRDF upstream.
2. Mở tay gắp, đưa vật thể và mặt sàn vào MoveIt Planning Scene.
3. **PRE_GRASP** → **GRASP**: MoveIt lập kế hoạch cho `panda_link8`.
4. Khép gripper, gọi Gazebo `/ATTACHLINK`, rồi attach vật thể vào `panda_hand` trong MoveIt.
5. **LIFT** → **TRANSPORT** → **PLACE**: thực thi quỹ đạo khi vật thể được attach ở cả hai hệ.
6. Gazebo `/DETACHLINK`, mở gripper, detach trong MoveIt rồi đọc vị trí thật của vật thể trong Gazebo để cập nhật Planning Scene.
7. **RETREAT** → `ready`.

Các tọa độ demo (m, hệ `world`): pick `(0.48,0,0.025)`, place `(0.35,0.30,0.025)`; sửa hằng số trong `src/pick_place.cpp` nếu điều kiện thực nghiệm khác.

## Tiêu chí nghiệm thu

- [ ] Gazebo và RViz hiển thị **cùng** Panda và robot di chuyển trong cả hai.
- [ ] MoveIt thành công lập kế hoạch và thực thi đến PRE_GRASP, GRASP, LIFT, TRANSPORT, PLACE.
- [ ] Khi GAZEBO `ATTACHLINK` thành công: cube đi theo gripper **vật lý**; Planning Scene có object ở danh sách attached.
- [ ] Khi `DETACHLINK`: cube trở lại model độc lập, rơi/được đặt trên nền; object trở lại world collision objects.
- [ ] Không tạo URDF Panda mới hoặc dùng MoveIt fake/mock hardware cho Gazebo.

## Phụ thuộc và giới hạn

- Panda geometry/SRDF/config từ [moveit/moveit_resources](https://github.com/moveit/moveit_resources), license theo upstream.
- Plugin attach/detach từ [IFRA-Cranfield/IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher), Apache-2.0, branch `humble`.
- Gazebo Classic đã EOL; lựa chọn này để làm bài trên Humble với plugin cố định, không nên sử dụng làm nền tảng sản phẩm mới.
- Vị trí/tư thế IK, điều khiển ngón và độ ổn định tiếp xúc cần tinh chỉnh sau thử nghiệm. Bài này giả định vật ở vị trí xác định, không có camera/phát hiện vật.
