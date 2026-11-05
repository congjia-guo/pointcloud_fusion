# pointcloud_fusion

ROS 1 / catkin 多雷达处理包。MID360 是父雷达，左、前、右 RoboSense 是子雷达 1、2、3。节点之间使用 sensor_msgs/PointCloud2。

| 配置 | 点云输入 |
|---|---|
| parent_pc_topic | /livox/lidar_pointcloud2，由 MID360 CustomMsg 转换得到 |
| child_pc_topic1 | /left_rslidar_points |
| child_pc_topic2 | /front_rslidar_points |
| child_pc_topic3 | /right_rslidar_points |

在工程的 ROS 1 工作空间中，若 Livox 驱动目录尚无 package.xml，先按驱动 ROS 1 配置将 package_ROS1.xml 复制为 package.xml。按原构建方式执行以下命令，并加载 devel/setup.bash。

~~~bash
catkin_make --cmake-args -DROS_EDITION=ROS1
~~~

先启动 roslaunch livox_ros_driver2 msg_MID360.launch 和发布三个 RoboSense ROS 话题的节点，或回放含这些话题的 rosbag。rs_driver 是底层驱动库，三路 ROS 发布由整车驱动配置提供。

## 标定

~~~bash
roslaunch pointcloud_fusion calibration.launch algorithm:=icp child_pc_topic:=/left_rslidar_points \
  initial_pose:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
~~~

依次指定左、前、右话题。算法支持 icp 和 ndt，标定直接使用各路输入点云。把示例初值换成各传感器的安装测量值。初值是子雷达到 Livox 的 XYZ（米）和 RPY（弧度），旋转按 Rz(yaw) * Ry(pitch) * Rx(roll)。在车辆静止且两路视场有足够重叠时取样，核对评分及实际点云对齐。标定同步窗口沿用原代码的 1.0 秒，可通过 sync_slop 缩小。

将日志中的 Child-to-parent YAML 对应填入 config/fusion.yaml 的 cpc1_to_ppc、cpc2_to_ppc、cpc3_to_ppc。标定节点会打印静态 TF 命令，默认广播标定 TF。已有相同子 frame 的 TF 时使用 broadcast_tf:=false。

## 测试融合

~~~bash
roslaunch pointcloud_fusion fusion_test.launch extrinsics_ready:=true
~~~

填入并验证三组外参后，再设置 extrinsics_ready:=true。使用单独的配置文件时加 config:=/path/to/fusion.vehicle.yaml。

测试入口默认开启过滤、在线调参、RViz 和 rqt_reconfigure。过滤由融合节点内部执行。先在每路雷达坐标系中清理无效 XYZ 点，执行距离及指定话题的地面高度过滤，再变换到父坐标系并融合，最后应用内外边界。地面过滤的话题列表由 ground_filter_topics 配置，默认是左、前、右三路。

在 rqt_reconfigure 中选择 /pointcloud_fusion 即可调整下表参数，修改即时用于后续融合帧。

| 参数 | 用途 | 默认值或调整范围 |
|---|---|---|
| enable_filter | 全部点云过滤的总开关 | 测试入口 true，正式入口 false |
| enable_range_filter | 在每路雷达坐标系内裁剪 XYZ | true |
| max_range | 保留各轴 ±max_range 内的点 | 3 m，滑条 0.01–50 m |
| enable_ground_filter | 对 ground_filter_topics 中的话题执行 Z 阈值过滤 | true |
| ground_z_threshold | 地面过滤下限，保留 ground_z_threshold ≤ z ≤ max_range | 0 m，滑条 ±20 m |
| set_params_external_bounds | 保留父坐标系外框内部 | true |
| set_params_internal_bounds | 剔除父坐标系内框内部 | true |

展开 /pointcloud_fusion/external_bounds 和 /pointcloud_fusion/internal_bounds 可调整六个边界值。边界滑条沿用原包的 X、Y ±50 m 和 Z ±20 m，默认外框为 X、Y ±50 m、Z ±5 m，内框初值为六项 0。

外参在线调参位于 /pointcloud_fusion/cpc1_to_ppc 等节点。平移滑条沿用原包的 X 0–1 m、Y 0–0.5 m、Z 0–0.3 m，三个旋转角为 0–3.15 rad。实测外参或边界超出对应滑条范围时，融合继续使用 YAML 值，并禁用该组在线调参。在线调参影响当前进程，验证后需写回 YAML。

## 正式融合和扫描

~~~bash
roslaunch pointcloud_fusion fusion.launch extrinsics_ready:=true
~~~

正式入口默认关闭 enable_filter，直接对输入点云执行同步、坐标变换和拼接。总开关关闭时，距离、地面及内外边界过滤全部旁路。已有转换节点时可用 start_converter:=false 复用。需要在该入口打开在线调参时，传入 set_params:=true open_tuning_gui:=true，并在 rqt 中确认总开关的状态。

节点按消息时间戳做近似同步，同步组的最大时间差由 sync_slop 配置，默认 0.05 秒。三路 RoboSense 与 MID360 的时间基准必须一致。输出 /pointcloud_fused 采用 MID360 的时间戳和实际 frame。配置保留 2/3/4 路和坐标变换开关。关闭坐标变换时，输入点云应已表达在同一坐标系。

两个融合入口都通过 lm_bringup/PointsCloud2toLaserscan.launch 发布导航使用的 /scan，类型为 sensor_msgs/LaserScan。扫描默认按 livox_frame 执行高度切片，scan_time 默认 0.1 秒。已有对应 TF 时可用 scan_target_frame:=body_2d 等参数改变切片和扫描坐标系。与定位一起运行时，使用 roslaunch lm_bringup moma_localize.launch publish_scan:=false 关闭定位入口的扫描发布。只看融合点云时可设 publish_scan:=false。

联调时先检查四路输入，再用 rostopic hz /pointcloud_fused 和 rostopic hz /scan 检查输出。扫描转换节点在有扫描订阅方时才订阅点云，rostopic hz /scan 本身会建立订阅。

ICP/NDT 标定源自随项目提供的 multi_lidar_calibration，其 Apache-2.0 许可随包保留。单雷达滤波沿用提供的 pointcloud_filter 的 MIT 标注。
