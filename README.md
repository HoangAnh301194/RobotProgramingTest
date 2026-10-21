# Panda Pick-and-Place — ROS 2 Humble, MoveIt 2 và Gazebo Classic

Hệ thống mô phỏng tay máy **Franka Emika Panda** gắp một khối hộp ở vị trí A, đặt tại B, sau đó gắp từ B về A và **lặp liên tục** cho đến khi dừng chương trình. MoveIt 2 đảm nhiệm lập kế hoạch/quản lý vật cản; Gazebo Classic mô phỏng chuyển động và liên kết vật lý khi gắp/thả.

**Môi trường đã sử dụng:** Ubuntu 22.04 · ROS 2 Humble · MoveIt 2 · Gazebo Classic 11 · `gazebo_ros2_control` · IFRA LinkAttacher.

> **Trạng thái:** Đã chạy thực tế trên máy Ubuntu: Panda xuất hiện trong Gazebo và RViz, các controller hoạt động, các bước gắp–nâng–vận chuyển–thả hoàn tất. Đã triển khai vòng lặp đổi chiều A ↔ B. Đây là kết quả nghiệm thu trên môi trường phát triển, không phải bảo đảm chạy ngay trên mọi máy khác.

## 1. Chức năng và kiến trúc

~~~text
                    pick_place_node (C++)
                            |
            +---------------+----------------+
            |                                |
        MoveIt 2                         Gazebo Classic
   - OMPL / RRTConnect              - Panda / pick_box / ground
   - Cartesian path                 - gazebo_ros2_control
   - Planning Scene                 - /ATTACHLINK, /DETACHLINK
   - AttachedCollisionObject        - Fixed joint vật lý
            |                                |
            +---------- Đồng bộ -------------+
                            |
               A → B → A → B → ... (liên tục)
~~~

- **Động học và hình học Panda:** tái sử dụng mô hình của `moveit_resources_panda_description`; không viết lại URDF tay máy.
- **Điều khiển:** bảy khớp Panda bằng `panda_arm_controller`; hai ngón qua `panda_hand_controller`; trạng thái khớp qua `joint_state_broadcaster`.
- **Kế hoạch chuyển động:** OMPL/RRTConnect cho di chuyển giữa các vùng; đường đi Cartesian cho GRASP, LIFT, PLACE và RETREAT.
- **Đồng bộ vật thể:** khi gắp, tạo liên kết vật lý trong Gazebo **và** Attached Collision Object trong MoveIt; khi thả, tháo cả hai rồi cập nhật pose thực tế từ Gazebo.
- **Đầu vào bài toán:** vị trí vật thể xác định trước; **không sử dụng camera, nhận dạng vật hoặc thuật toán tìm vị trí vật**.

### Cấu trúc repository

~~~text
RobotProgramingTest/
├── README.md                          # Tài liệu hướng dẫn chính
├── scripts/
│   └── install_deps.sh                # Cài phụ thuộc và tải IFRA
└── panda_pick_place/
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/controllers.yaml        # ros2_control controllers
    ├── launch/panda_gazebo.launch.py  # Gazebo + Panda + MoveIt + RViz
    ├── worlds/panda_pick_place.world  # Hộp đỏ, vị trí đích xanh, plugins
    └── src/pick_place.cpp             # Chương trình gắp–đặt lặp hai chiều
~~~

Các tệp `*.bak` hiện có trong lịch sử repository là bản sao lưu mã khi thử nghiệm, **không phải chương trình được launch hay build**.

## 2. Cài đặt từ đầu trên Ubuntu 22.04

### 2.1. Yêu cầu

Máy phải cài ROS 2 Humble và có khả năng mở Gazebo Classic/RViz. Các lệnh sau giả định workspace là `~/panda_ws` và repository được clone vào `~/panda_ws/src/RobotProgramingTest`.

~~~bash
source /opt/ros/humble/setup.bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/HoangAnh301194/RobotProgramingTest.git

# Đọc script nếu cần kiểm tra các gói apt trước khi chạy.
bash RobotProgramingTest/scripts/install_deps.sh
~~~

Script cài đặt dùng `sudo apt-get`, tải các gói MoveIt/Gazebo/ros2_control cần thiết và clone `IFRA_LinkAttacher` (nhánh `humble`) vào thư mục `src`.

### 2.2. Bắt buộc dùng Panda description có thông số quán tính

**Đây là bước quan trọng để tái tạo kết quả.** Phiên bản `moveit_resources_panda_description` cài sẵn từ apt từng khiến Gazebo spawn model nhưng **không có joint vật lý**, làm tất cả controller không hoạt động. Cấu hình đã nghiệm thu sử dụng package Panda description từ nhánh `ros2` của `moveit/moveit_resources` được overlay trong workspace.

~~~bash
cd ~/panda_ws/src

git clone --depth 1 --filter=blob:none --sparse --branch ros2 \
  https://github.com/moveit/moveit_resources.git \
  moveit_resources_source

cd moveit_resources_source
git sparse-checkout set panda_description
~~~

Nếu đã clone `moveit_resources_source` trước đó, không clone trùng; kiểm tra thư mục `panda_description` và file `urdf/panda.urdf.xacro`.

### 2.3. Cài dependencies và build

~~~bash
cd ~/panda_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --symlink-install \
  --allow-overriding moveit_resources_panda_description

source install/setup.bash

ros2 pkg prefix panda_pick_place
ros2 pkg prefix moveit_resources_panda_description
~~~

**Yêu cầu:** cả hai package được nhận diện; `moveit_resources_panda_description` phải nằm dưới `~/panda_ws/install/` thay vì trỏ về `/opt/ros/humble`.

Xác nhận Gazebo có đủ bảy joint Panda từ mô hình chính thức:

~~~bash
PANDA_DESC=$(ros2 pkg prefix moveit_resources_panda_description)

xacro \
  "$PANDA_DESC/share/moveit_resources_panda_description/urdf/panda.urdf.xacro" \
  > /tmp/panda_test.urdf

gz sdf -p /tmp/panda_test.urdf > /tmp/panda_test.sdf

grep -E "panda_joint[1-7]" /tmp/panda_test.sdf | head
~~~

## 3. Chạy mô phỏng

Mỗi terminal mới đều cần source ROS 2 và workspace. **Không chạy đồng thời nhiều `pick_place_node` để tránh xung đột lệnh điều khiển.**

### Terminal 1 — Gazebo, MoveIt, RViz và controllers

~~~bash
source /opt/ros/humble/setup.bash
source ~/panda_ws/install/setup.bash

ros2 launch panda_pick_place panda_gazebo.launch.py
~~~

Chờ Panda xuất hiện và ba controller được kích hoạt. Trong Gazebo có khối hộp đỏ ở A và dấu đích màu xanh tại B.

### Terminal 2 — Pick-and-Place liên tục A ↔ B

~~~bash
source /opt/ros/humble/setup.bash
source ~/panda_ws/install/setup.bash

ros2 run panda_pick_place pick_place_node --ros-args \
  -p stop_after_pregrasp:=false \
  -p stop_after_grasp:=false \
  -p stop_after_close:=false \
  -p grasp_z:=0.160
~~~

Đây là **lệnh chạy thực tế của phiên bản hiện tại**. Chương trình tự đổi chiều sau mỗi lượt thành công:

~~~text
LEG 1: A -> B
LEG 2: B -> A
LEG 3: A -> B
...
~~~

Dừng bằng **Ctrl+C** tại terminal đang chạy node. Nếu xảy ra lỗi gắp, va chạm, sai số đặt vượt giới hạn hoặc quỹ đạo không đầy đủ, node dừng thay vì tiếp tục các lượt sau.

**Không dùng mỗi `ros2 run panda_pick_place pick_place_node` mà bỏ các tham số trên:** trong phiên bản hiện tại, cả ba `stop_after_*` mặc định là `true` và hàm `run()` sẽ từ chối chế độ chạy liên tục. Tương tự, `run_task:=true` trong launch hiện chưa truyền ba tham số `false`, nên **không phải cách khởi chạy chu trình được khuyến nghị**.

### Build lại sau khi sửa C++ hoặc launch

~~~bash
cd ~/panda_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select panda_pick_place
source install/setup.bash
~~~

Để thử nghiệm sạch sau khi node dừng trong trạng thái vật còn attached hoặc hộp lệch khỏi vị trí dự kiến, dừng launch rồi khởi động lại Gazebo trước khi chạy lại node.

## 4. Thông số mô phỏng đã hiệu chỉnh

| Tham số | Giá trị | Ý nghĩa |
|---|---:|---|
| A (PICK ban đầu) | `(0.48, 0.00)` m | Tâm khối hộp đỏ theo X/Y |
| B (PLACE ban đầu) | `(0.35, 0.30)` m | Vị trí đích theo X/Y |
| Kích thước khối | `0.05 × 0.05 × 0.05` m | Hộp lập phương |
| Tâm khối khi nằm trên sàn | `z ≈ 0.025` m | Trong hệ `world` |
| `kPreZ` | `0.30` m | Cao độ PRE_GRASP |
| `grasp_z` khi chạy | `0.160` m | Cao độ GRASP của **panda_link8** |
| `kLiftZ` | `0.24` m | Cao độ LIFT/TRANSPORT/RETREAT |
| Mở gripper | `0.038` m | Vị trí mục tiêu của một ngón |
| Đóng gripper | `0.025` m | Vị trí mục tiêu của một ngón |
| Vận tốc/gia tốc kế hoạch | `0.20 / 0.20` | Hệ số so với giới hạn |
| Dung sai kiểm tra XY mỗi lượt | `0.015` m | 15 mm; vượt sẽ dừng |

**Lưu ý:** `grasp_z` là tọa độ của frame `panda_link8`, **không phải** độ cao đầu mút ngón. Tại tư thế hướng xuống đã đo được, trung điểm hai frame ngón cách `panda_link8` khoảng 58 mm dọc trục cục bộ. Không giảm `grasp_z` tùy ý vì có thể làm ngón chạm sàn hoặc hất vật.

Giới hạn kiểm tra trong chương trình: `0.14 ≤ grasp_z ≤ 0.22` m. Tham số mặc định của node là `0.18` m nhưng **giá trị đã dùng trong lệnh chạy nghiệm thu là 0.160 m**.

### Tại sao Gazebo dùng STL để hiển thị Panda?

Trên môi trường đã thử, mesh visual định dạng `.dae` không hiển thị, trong khi các collision mesh `.stl` chính thức hiển thị bình thường. Launch mặc định dùng STL làm **visual geometry** với:

~~~python
PANDA_DEBUG_STL_VISUALS=1
~~~

Đây là biến môi trường; mã launch đặt mặc định bằng `1`. Nếu muốn kiểm tra lại DAE:

~~~bash
PANDA_DEBUG_STL_VISUALS=0 \
  ros2 launch panda_pick_place panda_gazebo.launch.py
~~~

Chế độ này chỉ thay URI mesh dùng cho hiển thị trong XML được tạo ở bộ nhớ; không sửa file URDF gốc của Panda hay collision geometry.

## 5. Trình tự Pick-and-Place

Mỗi lượt được xử lý bởi `runOnce(pick_x, pick_y, place_x, place_y)`:

1. **PREFLIGHT:** kiểm tra dịch vụ Gazebo, gripper và vị trí khối hộp.
2. **READY:** mở ngón và đưa Panda về cấu hình `ready`.
3. **PRE_GRASP:** OMPL lập kế hoạch đến phía trên vật.
4. **GRASP:** hạ theo đường Cartesian, kiểm tra hộp không bị xê dịch đáng kể.
5. **CLOSE:** điều khiển vị trí hai ngón.
6. **ATTACH:** IFRA tạo fixed joint trong Gazebo; MoveIt đánh dấu `pick_box` là vật thể attached trong Planning Scene.
7. **LIFT:** nâng theo Cartesian đến `kLiftZ`.
8. **TRANSPORT:** OMPL chuyển vật từ vị trí gắp tới vị trí đặt.
9. **PLACE:** hạ theo Cartesian.
10. **DETACH:** tháo fixed joint, mở ngón, detach trong MoveIt; đọc pose thật từ Gazebo và cập nhật collision object.
11. **RETREAT:** rút lên theo Cartesian đến `kLiftZ` rồi trở về `ready`.
12. **VERIFY & REVERSE:** kiểm tra sai số XY, đợi ngắn để ổn định, đảo A/B và lặp lại.

### Hai cơ chế attach/detach độc lập

| Bên Gazebo | Bên MoveIt 2 |
|---|---|
| `/ATTACHLINK`, `/DETACHLINK` của IFRA | `attachObject`, `detachObject` và Planning Scene |
| Model `panda`, link **`panda_link7`** | Link **`panda_hand`** |
| Model `pick_box`, link `box_link` | Collision object ID `pick_box` |
| Tạo/xóa fixed joint vật lý | Bảo đảm vật được xét trong lập kế hoạch tránh va chạm |

**Vì sao tên link khác nhau?** Khi URDF được chuyển sang SDF, các link nối bằng fixed joint có thể bị gộp. Gazebo thực tế có `panda_link7` nhưng không có link vật lý `panda_hand`. MoveIt vẫn nhận biết `panda_hand` trong mô hình động học, vì vậy không được tự ý đổi cả hai bên về cùng một tên.

Vật thể được attach vật lý bằng plugin, **không phải một mô hình gắp dựa hoàn toàn trên lực ma sát/ngưỡng tiếp xúc**. Trong bước hạ cuối, chương trình tạm gỡ collision object của hộp trong Planning Scene để cho phép tiếp cận và thêm lại khi attach; đây là đơn giản hóa hiện tại, chưa phải cấu hình Allowed Collision Matrix có chọn lọc.

### Giới hạn an toàn khi tính Cartesian

Các giai đoạn GRASP, LIFT, PLACE và RETREAT sử dụng `computeCartesianPath()` với bước 5 mm, bật kiểm tra va chạm. Nếu `fraction < 0.98`, chương trình **không thực thi quỹ đạo một phần**.

Riêng LIFT có thêm phép tính chẩn đoán khi tắt collision checking. Quỹ đạo chẩn đoán **chỉ được tính để so sánh, không bao giờ được execute**.

## 6. Kiểm tra hoạt động

Giữ Gazebo đang chạy và mở terminal mới:

~~~bash
source /opt/ros/humble/setup.bash
source ~/panda_ws/install/setup.bash

# Cả ba controller phải ở trạng thái active.
ros2 control list_controllers -c /controller_manager

# Có đủ giao diện khớp Panda.
ros2 control list_hardware_interfaces -c /controller_manager

# Nhận được vị trí khớp có timestamp hợp lệ.
ros2 topic echo /joint_states --once

# Hai dịch vụ gắn / tháo vật thể.
ros2 service list | grep -E 'ATTACHLINK|DETACHLINK'

# Danh sách model Gazebo.
gz model -l

# Tọa độ thực tế của hộp.
gz model -m pick_box -p
~~~

Dấu hiệu thành công trong log:

~~~text
ATTACHED in Gazebo and Planning Scene
LIFT Cartesian execution: OK
TRANSPORT: OK
PLACE Cartesian execution: OK
GAZEBO DETACH: DETACHED
RETREAT Cartesian execution: OK
Leg N completed, placement error = ... mm
===== LEG N+1: B -> A =====
~~~

Ở một lần thử đơn hướng trước đó, chương trình báo sai số XY khoảng **0.1 mm**. Đây là số đo của một lần chạy cụ thể, không phải sai số bảo đảm cho mọi chu kỳ. Để đánh giá độ ổn định cần ghi số lượt liên tiếp và phân bố sai số theo thời gian.

## 7. Lỗi thường gặp

| Biểu hiện | Nguyên nhân/cách xử lý |
|---|---|
| Gazebo có model Panda nhưng không có khớp/controller | Panda description cũ thiếu thông số vật lý; build overlay `moveit_resources_panda_description` từ nguồn như mục 2.2 |
| `None of requested interfaces exist` | Gazebo không nạp được khớp/hardware interfaces; kiểm tra `gazebo_ros2_control` và model SDF |
| Panda không nhìn thấy trong Gazebo, RViz vẫn có | Dùng chế độ visual STL mặc định; DAE từng không render trên máy thử |
| MoveIt báo self-collision ngay khi bắt đầu | Kiểm tra `/joint_states`, tư thế khởi tạo và giới hạn khớp; launch có `initial_value` cho tư thế `ready` |
| `Failed to find link ... panda_hand` | Gazebo không có link vật lý đó; IFRA phải dùng `panda_link7` |
| `Cartesian fraction = 0.600/0.800` | Đường đi chưa hoàn chỉnh (IK, giới hạn khớp hoặc va chạm); không ép execute; kiểm tra cao độ LIFT/RETREAT và log chẩn đoán |
| `Cube left attached; stop safely` | Không chạy node mới trên trạng thái cũ; reset Gazebo và xác nhận Planning Scene |
| `Package 'panda_pick_place' not found` | Source `~/panda_ws/install/setup.bash` trong terminal hiện tại |
| Robot ép vật xuống nền | Kiểm tra frame `panda_link8`, offset ngón và `grasp_z`; tránh giảm độ cao theo cảm tính |
| Chạy node không truyền tham số thì dừng ngay | Bắt buộc truyền cả ba `stop_after_*:=false` trong chế độ liên tục |

Lưu ý khi gõ Bash nhiều dòng: dấu `\` phải là ký tự **cuối cùng của dòng**, không có dấu cách sau nó.

## 8. Phụ thuộc, hạn chế và hướng phát triển

- **Panda description / MoveIt:** [moveit/moveit_resources](https://github.com/moveit/moveit_resources); giữ nguyên giấy phép và điều kiện sử dụng của upstream.
- **Gazebo attach/detach:** [IFRA-Cranfield/IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher), nhánh `humble`.
- **Gazebo Classic 11:** sử dụng do tương thích với ROS 2 Humble và IFRA; Gazebo Classic đã hết vòng đời hỗ trợ.
- Bộ gắp dùng vị trí vật xác định sẵn; chưa có nhận dạng bằng camera hay điều chỉnh theo sai lệch đo được.
- Chưa có cơ chế phục hồi tự động khi quỹ đạo không khả thi hoặc trạng thái vật đang attached không đồng nhất.
- Nên cải thiện Allowed Collision Matrix, kiểm soát tiếp xúc ngón và ghi log nhiều chu kỳ trước khi mở rộng sang vật có kích thước khác.

## 9. Cập nhật mã từ GitHub

~~~bash
cd ~/panda_ws/src/RobotProgramingTest
git status
git pull --ff-only

cd ~/panda_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select panda_pick_place
source install/setup.bash
~~~

Nếu `git pull --ff-only` báo diverged hoặc có sửa đổi chưa commit, cần kiểm tra trạng thái Git trước, không dùng `reset --hard` để bỏ thay đổi đang làm.

---

**Tài liệu duy nhất để cài đặt, vận hành và mô tả hệ thống:** `README.md` ở thư mục gốc repository.
