# pointcloud_fusion

ROS 1 / catkin 多雷达处理包。MID360 是父雷达，左、前、右 RoboSense 是子雷达 1、2、3。所有处理中间话题均为 `sensor_msgs/PointCloud2`。

| 配置 | 原始输入 | 转换或过滤后 |
|---|---|---|
| `parent_pc_topic` | `/livox/lidar`，`livox_ros_driver2/CustomMsg` | `/livox/lidar_pointcloud2` → `/livox/lidar_filter` |
| `child_pc_topic1` | `/left_rslidar_points` | `/left_rslidar_points_filter` |
| `child_pc_topic2` | `/front_rslidar_points` | `/front_rslidar_points_filter` |
| `child_pc_topic3` | `/right_rslidar_points` | `/right_rslidar_points_filter` |

在工程的 ROS 1 工作空间中，若 Livox 驱动目录尚无 `package.xml`，先按驱动 ROS 1 配置将 `package_ROS1.xml` 复制为 `package.xml`。按原构建方式执行 `catkin_make --cmake-args -DROS_EDITION=ROS1`，然后加载 `devel/setup.bash`。先启动 `roslaunch livox_ros_driver2 msg_MID360.launch` 和发布三个 RoboSense ROS 话题的节点，或回放含这些话题的 rosbag。仓库内的 `rs_driver` 是驱动库，三路 ROS 发布由整车驱动配置提供。

## 标定

```bash
roslaunch pointcloud_fusion calibration.launch algorithm:=icp child_pc_topic:=/left_rslidar_points \
  initial_pose:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

依次指定左、前、右话题。算法支持 `icp` 和 `ndt`。把示例初值换成各传感器的安装测量值。初值是子雷达到 Livox 的 XYZ（米）和 RPY（弧度），旋转按 `Rz(yaw) * Ry(pitch) * Rx(roll)`。在车辆静止且两路视场有足够重叠时取样，核对评分及实际点云对齐。标定同步窗口沿用原代码的 1.0 秒，可通过 sync_slop 缩小。将日志中的 `Child-to-parent YAML` 对应填入 `config/fusion.yaml` 的 `cpc1_to_ppc`、`cpc2_to_ppc`、`cpc3_to_ppc`，或复制配置并通过 `config:=/path/to/fusion.yaml` 指定。标定节点还会打印静态 TF 命令，默认广播标定 TF。已有相同子 frame 的 TF 时使用 `broadcast_tf:=false`。

## 过滤

```bash
roslaunch pointcloud_fusion filter.launch
```

保留无效 XYZ 点清理、按轴 `±max_range` 裁剪，以及对指定话题保留 `ground_z_threshold <= z <= max_range` 的过滤。独立过滤入口默认保留原来 3 m 裁剪和三个子雷达的 Z 阈值过滤。转换和过滤都保留输入消息的时间戳和坐标系。Livox 转换节点复制 XYZ 并将 `reflectivity` 转成浮点 `intensity`。

## 融合和扫描

```bash
roslaunch pointcloud_fusion fusion.launch extrinsics_ready:=true
```

在三组外参填入并验证后再设置 `extrinsics_ready:=true`。默认启动转换与过滤，过滤只清理无效 XYZ。可通过 `enable_range_filter` 和 `enable_ground_filter` 恢复单雷达裁剪。若过滤已单独启动，使用 `start_filter:=false start_converter:=false`。若直接接未过滤点云，使用 `use_filtered:=false`。已有转换节点时，各入口可用 `start_converter:=false` 复用。

节点按消息时间戳做四路近似同步，要求同步组的最大时间差不超过 `sync_slop`。三路 RoboSense 与 MID360 的时间基准必须一致。输出 `/pointcloud_fused` 采用 MID360 的时间戳和实际 frame。融合配置保留 2/3/4 路、开关坐标变换、外框保留及内框剔除。关闭坐标变换时，输入点云必须已表达在同一坐标系。通过 `set_params:=true open_tuning_gui:=true` 打开在线外参及边界调节和 RViz。在线调参影响当前进程，验证后需写回 YAML。

在线调参沿用原包范围。外参平移 X 为 0–1 m，Y 为 0–0.5 m，Z 为 0–0.3 m，roll、pitch、yaw 为 0–3.15 rad。边界滑条 X、Y 为 ±50 m，Z 为 ±20 m。默认外框为 X、Y ±50 m、Z ±5 m，内框初值为六项 0。实测外参或边界超出滑条范围时，融合继续使用 YAML 值，并禁用该组在线调参。

`fusion.launch` 将融合点云传入 `lm_bringup/PointsCloud2toLaserscan.launch`，发布导航使用的 `/scan`（`sensor_msgs/LaserScan`）。默认按 `livox_frame` 执行高度切片，`scan_time` 默认 0.1 秒。已有对应 TF 时可用 `scan_target_frame:=body_2d` 等参数改变切片和扫描坐标系。与定位一起运行时，用 `roslaunch lm_bringup moma_localize.launch publish_scan:=false` 关闭定位入口的扫描发布，保持 `/scan` 只有一个发布源。只看融合点云时可设 `publish_scan:=false`。

联调时检查四路话题的类型、时间戳和 frame，再用 `rostopic hz /pointcloud_fused`、`rostopic hz /scan` 检查输出。转换节点在有扫描订阅方时才订阅点云，`rostopic hz /scan` 本身会建立订阅。

ICP/NDT 标定源自随项目提供的 `multi_lidar_calibration`，其 Apache-2.0 许可随包保留。单雷达滤波沿用提供的 `pointcloud_filter` 的 MIT 标注。
