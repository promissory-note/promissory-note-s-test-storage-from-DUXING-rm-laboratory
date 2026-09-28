# hik_camera_ros2

海康机器人（HIKROBOT）MVS SDK 的 **ROS 2 Humble** 封装功能包。
把海康工业相机接入 ROS 2：自动发现/连接相机、采集并发布 `sensor_msgs/msg/Image`、通过 ROS 2 参数动态配置相机、断线自动重连。

## 一、功能
- **设备连接**：按序列号或 IP 自动发现并连接；断线后自动重连并恢复已配置参数。
- **图像发布**：连续采集，转换为标准 `sensor_msgs/msg/Image`，发布到可配置话题（默认 `image_raw`）。
- **参数配置**（ROS 2 parameter，运行中可改）：
  - 曝光时间 `exposure_time`（us，设置前自动关闭自动曝光 `ExposureAuto=Off`）
  - 增益 `gain`（dB，设置前关闭自动增益 `GainAuto=Off`）
  - 帧率 `frame_rate`（fps，使能 `AcquisitionFrameRateEnable` 后设置）
  - 像素格式 `pixel_format`（`Mono8` / `RGB8_Packed` / `BGR8_Packed`，切换前后暂停/恢复采集）
  - 参数更新会校验范围（读取 SDK 的 min/max）与 SDK 返回码，失败会拒绝并说明原因。

## 二、环境依赖
- Ubuntu 22.04 + **ROS 2 Humble**（已安装 `ros-dev-tools`）
- OpenCV 非必需；本包仅依赖 `rclcpp / rcl_interfaces / sensor_msgs / std_msgs`
- **海康 MVS SDK**（rosdep 无法安装，需手动安装，见下）
- 硬件：海康 USB3/GigE 工业相机（本机为 MV-CS016-10UC，USB3）

## 三、安装 MVS SDK（必须手动，rosdep 不支持）
1. 从海康机器人官网“服务与支持 → 下载中心”下载 **MVS V5.1.0 (Linux)(x86_64)**（约 385 MB）。
   本机已下载：`~/Downloads/MVS_510/MVS-5.1.0_x86_64_20260909.deb`
2. 安装（需要 sudo）：
   ```bash
   sudo apt install -y ~/Downloads/MVS_510/MVS-5.1.0_x86_64_20260909.deb
   # 或： sudo dpkg -i ~/Downloads/MVS_510/MVS-5.1.0_x86_64_20260909.deb
   ```
   安装脚本会把 SDK 放到 `/opt/MVS`（`include/` 头文件、`lib/64/` 动态库、`bin/`、`Samples/`），并配置 udev/环境。
3. 验证：
   ```bash
   ls /opt/MVS/include/MvCameraControl.h /opt/MVS/lib/64/libMvCameraControl.so
   ```
> 说明：MVS 无对应 rosdep 规则，故 **必须提供版本(5.1.0)、下载与安装方式、库路径(/opt/MVS/lib/64)**，不能承诺 `rosdep` 自动安装。CMake 通过 `-DMVS_ROOT=/opt/MVS` 定位（默认即 `/opt/MVS`）。

## 四、构建
```bash
mkdir -p ~/ros2_ws/src
cp -r hik_camera_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y     # 只会安装 ROS 依赖，不含 MVS
colcon build --packages-select hik_camera_ros2
source install/setup.bash
```

## 五、运行
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hik_camera_ros2 hik_camera.launch.py
# 或直接运行节点
ros2 run hik_camera_ros2 hik_camera_node --ros-args --params-file config/camera_params.yaml
```
查看图像：
```bash
ros2 topic list
ros2 topic hz /image_raw
rviz2          # 添加 Image，话题选 /image_raw
# 或 rqt_image_view
```

## 六、动态设置参数（示例）
```bash
ros2 param list /hik_camera
ros2 param set /hik_camera exposure_time 5000.0    # 5000 us
ros2 param set /hik_camera gain 10.0               # 10 dB
ros2 param set /hik_camera frame_rate 30.0         # 30 fps
ros2 param set /hik_camera pixel_format RGB8_Packed
```
设置失败（越界/相机不支持）会返回失败原因，例如 `exposure_time 超出范围 [..]`。

## 七、参数说明
| 参数 | 类型 | 说明 |
|---|---|---|
| `serial_number` | string | 相机序列号（优先匹配），本机 `DB0178696` |
| `camera_ip` | string | GigE 相机 IP（序列号为空时使用） |
| `topic` | string | 图像话题，默认 `image_raw` |
| `frame_id` | string | 图像 header.frame_id，默认 `camera` |
| `exposure_time` | double | 曝光 us，<0 不设置 |
| `gain` | double | 增益 dB，<0 不设置 |
| `frame_rate` | double | 帧率 fps，<0 不设置 |
| `pixel_format` | string | `Mono8`/`RGB8_Packed`/`BGR8_Packed`，空不切换 |

## 八、常见问题
- **找不到相机 / 打不开**：确认相机已插好（`lsusb | grep 2bdf`），并已通过 .deb 安装 SDK（含 udev 规则）；USB 相机建议直连 USB3 口。
- **`libMvCameraControl.so` 找不到**：确认已装 MVS，或 `cmake -DMVS_ROOT=<路径>`；运行时 rpath 已指向 `/opt/MVS/lib/64`。
- **帧率上不去**：默认分辨率下受 USB3/曝光时间限制；降低曝光或设 `frame_rate` 后观察 `ros2 topic hz`。
- **像素格式切换失败**：相机不支持该格式（用 `MV_CC_GetEnumValue` 查询支持列表），或需先停流再切换（本节点已自动处理停流/恢复）。

## 九、许可
Apache-2.0
