# pointcloud_fusion

**ROS 1 / catkin 四激光雷达标定与点云融合功能包**

- **父雷达** — Livox MID360。
- **子雷达** — 左、前、右三个 RoboSense 雷达。
- **点云输出** — /pointcloud_fused，消息类型为 sensor_msgs/PointCloud2。
- **扫描输出** — /scan，消息类型为 sensor_msgs/LaserScan。
- **使用顺序** — 标定 → 测试融合与 rqt 调参 → 正式融合。

## 1. 数据从哪里来，到哪里去

~~~mermaid
flowchart LR
    MID["MID360<br/>/livox/lidar<br/>CustomMsg"] --> CONV["MID360 转换节点<br/>XYZ + intensity"]
    CONV --> PARENT["/livox/lidar_pointcloud2"]
    LEFT["左雷达<br/>/left_rslidar_points"] --> FUSION
    FRONT["前雷达<br/>/front_rslidar_points"] --> FUSION
    RIGHT["右雷达<br/>/right_rslidar_points"] --> FUSION
    PARENT --> FUSION["融合节点<br/>同步 / 可选过滤 / 外参变换"]
    FUSION --> CLOUD["/pointcloud_fused<br/>PointCloud2"]
    CLOUD --> SCAN_NODE["moma_perception<br/>点云转扫描"]
    SCAN_NODE --> SCAN["/scan<br/>LaserScan"]

    classDef input fill:#e8f1ff,stroke:#3974b8,color:#17334f
    classDef process fill:#fff3d6,stroke:#b9892e,color:#544015
    classDef output fill:#e7f5ec,stroke:#438962,color:#20492e
    class MID,LEFT,FRONT,RIGHT,PARENT input
    class CONV,FUSION,SCAN_NODE process
    class CLOUD,SCAN output
~~~

| 角色 | 融合参数 | 输入话题 | 消息类型 |
|---|---|---|---|
| MID360 原始数据 | mid360_msg_topic | /livox/lidar | livox_ros_driver2/CustomMsg |
| 父雷达 | parent_pc_topic | /livox/lidar_pointcloud2 | sensor_msgs/PointCloud2 |
| 左侧子雷达 | child_pc_topic1 | /left_rslidar_points | sensor_msgs/PointCloud2 |
| 前侧子雷达 | child_pc_topic2 | /front_rslidar_points | sensor_msgs/PointCloud2 |
| 右侧子雷达 | child_pc_topic3 | /right_rslidar_points | sensor_msgs/PointCloud2 |

- MID360 转换节点保留原始时间戳和 frame。
- reflectivity 转换为浮点 intensity。
- 融合点云采用父雷达的时间戳和 frame，默认 frame 为 livox_frame。
- 四路时间戳需要使用相同的时间基准。

## 2. 三个启动入口

| 入口 | 用途 | 点云过滤 | rqt / RViz |
|---|---|---|---|
| [calibration.launch](launch/calibration.launch) | 两路雷达 ICP / NDT 标定 | 直接使用标定输入 | 通过日志查看结果 |
| [fusion_test.launch](launch/fusion_test.launch) | 验证融合与调整参数 | 默认开启 | 默认开启 |
| [fusion.launch](launch/fusion.launch) | 正式运行融合与扫描转换 | 默认关闭 | 默认关闭 |

~~~mermaid
flowchart TD
    PREP["准备驱动、工作空间与四路输入"] --> CAL["calibration.launch<br/>分别标定左 / 前 / 右"]
    CAL --> YAML["填写并核对三组外参<br/>config/fusion.yaml"]
    YAML --> TEST["fusion_test.launch<br/>开启过滤、rqt、RViz"]
    TEST --> CHECK{"点云对齐、同步与扫描通过验收？"}
    CHECK -- "继续调整" --> YAML
    CHECK -- "通过" --> RUN["fusion.launch<br/>默认关闭全部点云过滤"]

    classDef step fill:#e8f1ff,stroke:#3974b8,color:#17334f
    classDef test fill:#fff3d6,stroke:#b9892e,color:#544015
    classDef production fill:#e7f5ec,stroke:#438962,color:#20492e
    class PREP,CAL,YAML step
    class TEST,CHECK test
    class RUN production
~~~

## 3. 编译与驱动准备

- 环境使用 ROS 1 Noetic、catkin、PCL 和 Eigen。
- 需要 livox_ros_driver2 提供 MID360 驱动与 CustomMsg。
- 扫描转换需要 lm_bringup 和 pointcloud_to_laserscan。
- 调参界面需要 rqt_reconfigure 和 RViz。
- 下列命令以 ~/catkin_ws 为工作空间示例，按实际路径替换。

~~~bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make --cmake-args -DROS_EDITION=ROS1
source devel/setup.bash
rospack find pointcloud_fusion
~~~

**启动 MID360 驱动**

~~~bash
roslaunch livox_ros_driver2 msg_MID360.launch
~~~

- 如果驱动目录尚无 package.xml，按驱动的 ROS 1 说明准备 package_ROS1.xml。
- 用整车现有驱动配置启动三个 RoboSense 雷达。
- rs_driver 是底层库，三个 ROS 点云话题需要整车发布节点提供。
- 每个新终端都要加载工作空间的 devel/setup.bash。

> [!IMPORTANT]
> 默认融合入口会向 lm_bringup 的 PointsCloud2toLaserscan.launch 传入 cloud_topic、scan_topic、target_frame 和 scan_time。
>
> - perception 中的扫描 launch 必须声明这些参数。
> - 本项目本地提交 4146403 包含该接口改动。
> - 旧版固定订阅 /cloud_registered 的 launch 需要先更新。
> - 仅验证融合点云时，可传入 publish_scan:=false。

## 4. 分别标定左、前、右雷达

**标定前**

- 车辆保持静止。
- 两路雷达的视场有足够重叠。
- 根据安装测量值准备初始 XYZ 和 RPY。
- 平移单位为米，角度单位为弧度。
- 旋转顺序为 Rz(yaw) × Ry(pitch) × Rx(roll)。

**以左雷达 ICP 标定为例**

~~~bash
roslaunch pointcloud_fusion calibration.launch \
  algorithm:=icp \
  child_pc_topic:=/left_rslidar_points \
  initial_pose:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
~~~

- initial_pose 中六个零是填写格式示例，运行前替换成该雷达的安装初值。
- 依次修改 child_pc_topic，分别完成三路标定。
- 使用 NDT 时，将 algorithm 改为 ndt。
- 已有相同父子 frame 的 TF 时，添加 broadcast_tf:=false。

| 待标定雷达 | child_pc_topic | 结果填写位置 |
|---|---|---|
| 左 | /left_rslidar_points | cpc1_to_ppc |
| 前 | /front_rslidar_points | cpc2_to_ppc |
| 右 | /right_rslidar_points | cpc3_to_ppc |

**保存结果**

1. 观察是否收敛，以及评分和连续多帧外参是否稳定。
2. 从日志中找到 Child-to-parent YAML。
3. 将 XYZ / RPY 填入 [config/fusion.yaml](config/fusion.yaml) 的对应项。
4. 结合实际点云重叠情况核对结果。
5. 完成实测外参核对后，启动融合时设置 extrinsics_ready:=true。

- fusion.yaml 内现有外参需要按实际车辆确认，换车或调整安装位置后重新标定。
- 标定同步窗口默认 1.0 秒。
- ICP 默认最多迭代 100 次，NDT 默认最多迭代 10 次。
- 标定结果同时提供变换矩阵、评分及静态 TF 命令。

## 5. 测试融合与 rqt 在线调参

~~~bash
roslaunch pointcloud_fusion fusion_test.launch extrinsics_ready:=true
~~~

**只查看融合点云**

~~~bash
roslaunch pointcloud_fusion fusion_test.launch \
  extrinsics_ready:=true publish_scan:=false
~~~

- 默认打开过滤总开关、rqt_reconfigure 和 RViz。
- 在 RViz 中检查父坐标系下的点云是否重合。
- 在 rqt 中选择 /pointcloud_fusion 调整过滤参数。
- rqt 修改即时用于后续帧，确认后将参数写回配置文件。

~~~mermaid
flowchart TD
    INPUT["同步后的各路点云"] --> ENABLE{"enable_filter 开启？"}
    ENABLE -- "开启" --> CLEAN["每路雷达坐标系<br/>清理无效 XYZ"]
    CLEAN --> RANGE["距离裁剪<br/>各轴限制在 ±max_range"]
    RANGE --> GROUND["指定话题的地面过滤<br/>ground_z_threshold ≤ Z ≤ max_range"]
    GROUND --> TF["子雷达变换到父坐标系"]
    TF --> BOX["父坐标系内外边界裁剪"]
    BOX --> MERGE["点云拼接与发布"]

    ENABLE -- "关闭" --> DIRECT["坐标变换<br/>全部点云过滤旁路"]
    DIRECT --> MERGE

    classDef filter fill:#fff3d6,stroke:#b9892e,color:#544015
    classDef bypass fill:#e8f1ff,stroke:#3974b8,color:#17334f
    classDef output fill:#e7f5ec,stroke:#438962,color:#20492e
    class CLEAN,RANGE,GROUND,BOX filter
    class DIRECT,TF bypass
    class MERGE output
~~~

**过滤总开关和数值**

| rqt 参数 | 作用 | 默认值 / 滑条范围 |
|---|---|---|
| enable_filter | 控制全部点云过滤 | 测试 true，正式 false |
| enable_range_filter | 每路点云 XYZ 距离裁剪 | true |
| max_range | 各轴绝对坐标上限 | 默认 3 m，范围 0.01–50 m |
| enable_ground_filter | 启用指定话题的 Z 阈值过滤 | true |
| ground_z_threshold | 地面过滤的 Z 下限 | 默认 0 m，范围 −20–20 m |
| set_params_external_bounds | 保留外框内部 | true |
| set_params_internal_bounds | 剔除内框内部 | true |

- 距离裁剪按 XYZ 各轴构成盒状范围。
- 地面过滤默认作用于左、前、右三个子雷达。
- ground_filter_topics 在 launch 中配置。
- enable_filter 关闭时，距离、地面和内外边界开关全部旁路。

**展开 rqt 中的子项**

| rqt 位置 | 调整内容 | 范围 |
|---|---|---|
| /pointcloud_fusion/external_bounds | 外框的 XYZ 最小值和最大值 | X、Y ±50 m，Z ±20 m |
| /pointcloud_fusion/internal_bounds | 内框的 XYZ 最小值和最大值 | X、Y ±50 m，Z ±20 m |
| /pointcloud_fusion/cpc1_to_ppc | 左雷达到父雷达的外参 | 见下表 |
| /pointcloud_fusion/cpc2_to_ppc | 前雷达到父雷达的外参 | 见下表 |
| /pointcloud_fusion/cpc3_to_ppc | 右雷达到父雷达的外参 | 见下表 |

| 外参滑条 | 范围 |
|---|---|
| X | 0–1 m |
| Y | 0–0.5 m |
| Z | 0–0.3 m |
| Roll / Pitch / Yaw | 0–3.15 rad |

- 默认外框为 X、Y ±50 m，Z ±5 m。
- 默认内框的六个边界值均为 0。
- 外参及边界滑条范围沿用原始融合代码。
- YAML 实测值超出对应滑条范围时，继续使用 YAML 值，并关闭该组在线调参。
- 实测外参包含负平移时，可直接编辑 YAML 后重启验证。

## 6. 正式融合与 /scan

~~~bash
roslaunch pointcloud_fusion fusion.launch extrinsics_ready:=true
~~~

- 正式入口默认 enable_filter:=false。
- 节点执行时间同步、坐标变换和点云拼接。
- 融合结果发布到 /pointcloud_fused。
- 扫描转换读取 /pointcloud_fused 并发布 /scan。
- 下游扫描转换的高度切片、有效距离等规则仍由 perception 的 launch 配置。

**常用启动参数**

| 参数 | 默认值 | 用途 |
|---|---|---|
| config | 包内 config/fusion.yaml | 指定实测外参和融合配置 |
| extrinsics_ready | false | 确认外参已填写和验证 |
| start_converter | true | 启动 MID360 消息转换 |
| fusion_pc_topic | /pointcloud_fused | 融合输出话题 |
| fusion_pc_frame_id | livox_frame | 校验父雷达 frame |
| set_params | 正式 false，测试 true | 启用在线调参服务 |
| open_tuning_gui | 正式 false，测试 true | 打开 rqt 和 RViz |
| publish_scan | true | 启动点云转扫描 |
| scan_topic | /scan | 扫描输出话题 |
| scan_target_frame | 空 | 默认沿用点云 frame |
| scan_time | 0.1 s | 扫描消息的周期参数 |

**指定车辆配置**

~~~bash
roslaunch pointcloud_fusion fusion.launch \
  config:=/path/to/fusion.vehicle.yaml \
  extrinsics_ready:=true
~~~

**与定位一起运行**

- 使用支持 publish_scan 参数的 moma_localize.launch。
- 在原定位启动命令中加入 publish_scan:=false。
- 由融合入口统一发布 /scan。
- 设置 scan_target_frame:=body_2d 前，先确认该 frame 与 livox_frame 之间有可用 TF。

**配置文件中的同步项**

| 参数 | 默认值 | 含义 |
|---|---|---|
| fusion_lidar_num | 4 | 支持 2 / 3 / 4 路输入 |
| sync_slop | 0.05 s | 同步组内允许的最大时间戳跨度 |
| sync_queue_size | 10 | 同步队列长度 |
| set_params_tf | true | 将子雷达点云变换到父坐标系 |

- 三路子雷达按照 child_pc_topic1、2、3 的顺序加入融合。
- 关闭 set_params_tf 时，各路输入应已表达在同一坐标系。
- 任一路无法提供匹配帧时，会影响融合持续输出。

## 7. 上车验收与问题定位

**检查输入类型**

~~~bash
rostopic type /livox/lidar
rostopic type /left_rslidar_points
rostopic type /front_rslidar_points
rostopic type /right_rslidar_points
~~~

**检查输入和输出频率，观察后按 Ctrl+C**

~~~bash
rostopic hz /livox/lidar /left_rslidar_points /front_rslidar_points /right_rslidar_points
~~~

~~~bash
rostopic hz /pointcloud_fused /scan
~~~

**核对扫描链路**

~~~bash
rostopic info /scan
rostopic echo -n 1 /scan/header
~~~

- 四路输入都有持续频率，时间戳非零。
- 融合点云在静止场景下连续输出。
- /scan 的发布来源唯一。
- RViz 中墙面重叠处没有明显双影。
- 分别在车左、前、右放置障碍，扫描方向与实际方位相符。

| 现象 | 优先检查 |
|---|---|
| 四路有数据，融合没有输出 | extrinsics_ready、四路时间基准、sync_slop、父雷达 frame |
| 测试时点云大量减少 | enable_filter、max_range、地面阈值、内外裁剪框 |
| rqt 中找不到参数 | set_params 是否开启、是否选中 /pointcloud_fusion |
| 某组外参没有调参项 | YAML 值是否超出原包滑条范围，查看节点警告 |
| 启动时报 unused args | perception 扫描 launch 是否支持传入的四个参数 |
| 融合有数据，扫描没有有效回波 | 扫描高度切片、有效距离、目标 frame 和 TF |
| /scan 的发布来源冲突 | 定位入口的 publish_scan 是否关闭 |

## 8. 文件索引与许可

| 文件 | 职责 |
|---|---|
| [launch/calibration.launch](launch/calibration.launch) | ICP / NDT 标定入口 |
| [launch/fusion_test.launch](launch/fusion_test.launch) | 开启过滤和调参的测试入口 |
| [launch/fusion.launch](launch/fusion.launch) | 正式融合入口 |
| [config/fusion.yaml](config/fusion.yaml) | 外参、同步和裁剪配置 |
| [config/Filter.cfg](config/Filter.cfg) | 过滤开关与数值滑条 |
| [config/Extrinsics.cfg](config/Extrinsics.cfg) | 外参滑条范围 |
| [config/Bounds.cfg](config/Bounds.cfg) | 内外边界滑条范围 |
| [src/mid360_to_pointcloud2.cpp](src/mid360_to_pointcloud2.cpp) | MID360 CustomMsg 转 PointCloud2 |
| [src/pointcloud_fusion_node.cpp](src/pointcloud_fusion_node.cpp) | 同步、过滤、外参变换和融合 |

- ICP / NDT 标定源自 multi_lidar_calibration，Apache-2.0 许可见 [LICENSE.Apache-2.0](LICENSE.Apache-2.0)。
- 单雷达过滤逻辑沿用提供的 pointcloud_filter 的 MIT 标注。
