# 无需录线的连续闭环导航

针对本次 141.134 秒五圈成绩及 TEB infeasible 重置，增加一套独立的连续控制算法。旧 TEB 导航仍可用于对照。新入口不启动 move_base，也不运行 waypoint_laps；它不属于修改 TEB 二进制，而是替换点到点的局部导航控制流程。

## 算法改变

- 静止时从当前 ROS 地图和原来的四个有序引导航点生成整圈路线。A* 障碍距离代价优先选择宽裕的走廊，未知格视为障碍。
- 周期平滑覆盖首尾接缝；每个候选检查完整线段的障碍距离、转向曲率和航点顺序。规划失败就不启动运动。不是直接用直线连接四个航点。
- 后轴参考的速度自适应预瞄跟踪，50 Hz 输出，限制转向变化率。轮速前馈处理内外轮速度差。没有倒车纠偏或每圈重新发送起点目标。
- `v <= sqrt(lateral_accel / |curvature|)`，同时约束航向变化率、转向变化率，并在闭环上做向后制动/向前加速传播。直线和紧弯共用连续的速度包络，接缝不是零速边界。
- 激光检查沿预规划弯道的走廊，避免将当前转角无限外推到弯道出口的墙上；另有静态地图距离、跟踪误差、传感器时效、控制超时保护。出现故障会锁定停止，需要排查并重新启动。

`lateral_accel` 是控制预算，不是已经测出的侧翻极限。当前地图路线最紧曲率约 1.7/m，预算 5 m/s² 对应约 1.7 m/s 紧弯速度；直线可以达到 `max_speed`。极限还受轮胎摩擦、重心、转向响应和定位误差影响，不能仅凭圈速算出“保证不翻车”的最大速度。暂不改原车轮 PID 或官方模型。

## 更新并确认代码路径

停止旧仿真、旧 waypoint_laps 和其他控制节点，重新启动整套环境。每个新终端都加载工作空间。不要直接执行 setup.bash，也不要只 cd 到源码目录来切换 ROS 包。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
sudo apt-get update
sudo apt-get install -y python3-scipy python3-numpy python3-yaml
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack profile
rospack find tianracer_gazebo
ls "$(rospack find tianracer_gazebo)/launch/planned_race_demo.launch"
ls "$(rospack find tianracer_gazebo)/scripts/planned_race.py"
```

`rospack find` 应指向 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。指向 tianbot_ws 表示加载了其他工作空间，先修正环境再启动。

## 运行顺序

终端 A：官方赛道、车辆、定位及底盘控制。不发送导航任务。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：裁判。检测、绑定和正式环境校验按赛事教程进行。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

终端 C：第一次用 1.8 验证路径跟踪，成功后再重跑 2.0，最后按实测考虑 2.1。提高直线速度时先保持横向加速度预算不变，不要再直接把所有弯道速度抬到 2.2。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planned_race.launch max_speed:=1.8 lateral_accel:=5.0
```

等待 `Ready` 和 `Idle`，此时车辆仍静止。RViz 添加 Path 显示 `/tianracer/race_path` 可检查整圈路线。计划保存到 `~/ican_race_logs/planned_course.csv`，列为后轴 x、y、曲率、速度上限，不能当作裁判结果。

裁判点击“准备发车”后，终端 D 发车：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/start "{}"
```

必须收到 `success: True`；否则按返回信息处理，车辆不会自动倒车寻找路线。**本流程不再启动 waypoint_laps.launch。** 同时存在其他 cmd_vel 发布者会拒绝启动。

裁判锁定五圈后，立即停止车辆：

```bash
rosservice call /tianracer/continuous_race/stop "{}"
```

默认最多行驶六个参考闭环，给官方判圈留出起始位置差异；裁判锁定结果不自动给车辆制动。参考闭环圈数不能代替官方五圈。必须以官方显示 5/5、校验和完赛用时判断效果，再按教程提交。

## 比较及排查

同样的正式赛道、初始位置和定位条件下，每个速度档重复至少三次，比较官方五圈总用时、完赛成功率、碰撞和各圈差异。目标低于 120 秒；尚未在 Gazebo 实测达成。

保留日志：

```bash
mkdir -p ~/ican_race_logs
rosbag record -O ~/ican_race_logs/planned_test.bag /clock /tf /tf_static /tianracer/scan /tianracer/odom /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/race_path
```

若 `Race stopped`，保留原因与日志。传感器失效、定位偏离或墙体遮挡的制动不能通过关闭保护来消除。若仍出现 TEB infeasible 或 GOAL Reached，说明旧 move_base 还在运行，检查 `rosnode list` 并关闭旧启动终端。

## 已完成和未完成的验证

已在仓库赛道地图完成整圈几何检查、周期速度约束、官方初始位置的理想自行车模型五圈连续跟踪、地图模拟激光检查和 ROS 假传输的启动/停止检查。理想模型不包含轮胎侧滑、真实转向延迟、车身倾斜或 AMCL 定位噪声，不能证明实际不撞墙、不翻车或两分钟内完赛。

云端无 ROS/Gazebo，不能执行正式仿真或裁判判圈。官方赛道、地图、航点坐标、车辆模型、底盘 PID 和裁判没有修改。路线平滑能否通过所有官方检测线仍需实际裁判确认。

## 弯道激光锁定停车修正

若旧版出现笼统的 `Obstacle/invalid scan`，更新后保护使用扫描采集时刻的 map→laser TF，避免把过去的扫描套在当前转弯车姿上。障碍走廊改为随方向旋转的矩形，半长 0.23 米、半宽 0.16 米，覆盖原 costmap 半边 0.15 米正方形及 TEB 半长 0.18 米、半宽 0.10 米的轮廓。与原圆形比较，修正侧向误判并增加纵向覆盖；不关闭激光、定位或超时保护。

新日志 `Laser speed restriction` / `Laser protection` 包含具体原因：

- `invalid_scan_metadata`：扫描参数无效。
- `invalid_scan_coverage`：前方有效/无回波光束不足 90%；附无效值数量。
- `obstacle_in_corridor`：保护轮廓中有实际激光点；附预测路径上的命中距离 `hit_arc`。

同时记录 `scan_age`、`speed_limit` 和 `tracking_error`。若更新后仍停车，请提供完整的这条 JSON 日志；不要关闭保护或反复启动来强行通过。时间配准修正及侧向误判已离线验证，但尚不能确认之前那次停车属于哪一种，也未在 Gazebo 实测通过该弯道。

## 仿真时钟与调度间隔修正

旧版 `Invalid feedback or stale control cycle` 把非有限输入、零 dt 和大于 0.2 秒的控制周期合并为同一错误。本次实测前三圈 24.727、24.540、24.451 秒后出现该错误，原日志没有 dt，无法确认是重复时钟还是调度延迟。

更新后重复 `/clock`（dt=0）只跳过本次计算，保留闭环进度，仿真恢复时间推进后继续计算。生成路线模式允许不超过 0.3 秒的调度间隔，但仍要求激光、里程计、定位各自通过原有新鲜度检查。路线投影使用实际经过的时间，速度与转向积分最多使用 0.04 秒，避免一次追补过大的控制变化。不会重复使用过期反馈继续前进。

时钟倒退、非有限输入、超过 0.3 秒的控制中断和传感器失效仍会锁定零速。调度延迟超过 0.1 秒输出具体 dt；中断停车输出 `Control scheduling gap: dt=...`。先保持 max_speed=1.8、lateral_accel=5.0，完整完赛后再考虑提高直线速度。当前前三圈合计 73.718 秒，不是五圈完赛成绩，也不能证明已低于两分钟。

## 起步偏离排查与低速对齐

出现 `Tracking error exceeds safe corridor` 时，旧日志没有车姿、实际误差和指令，不能仅凭这一行判断定位跳变、局部投影范围或实际偏离。更新后错误包含实际/全局投影误差、车姿、路径进度、实测速率、最近速度/转向指令及 dt；启动时也记录车姿。

新增一次性的起步对齐阶段：偏离超过 3.5 厘米或航向偏差超过 0.15 弧度时，将前进速度限制在 0.35 m/s，先建立跟踪再加速。每圈接缝不会重新进入这个阶段。0.15 米误差阈值、地图距离和激光保护不变，起步阶段也不能越过保护阈值。离线验证包含初始横向/航向偏差和假设 120ms 执行器响应；不代表已确定本次故障根因或已在 Gazebo 验证修复。

## 发车被拒绝或服务不存在

先保持仿真终端 A 和导航终端 C 打开，Gazebo 解除暂停；只 source 环境不会启动节点。终端 C 必须正在运行 `planned_race.launch` 且已打印 Ready。另一个终端调用：

```bash
rosparam get /use_sim_time
rosnode ping -c 1 /tianracer/continuous_race
rosservice call /tianracer/continuous_race/status "{}"
```

`/use_sim_time` 应为 true。status 不发送运动指令。返回 True 表示当前反馈通过时效检查；正式发车仍需裁判准备，并调用 start。返回 False 时会指出 scan、odom 或定位 TF 的具体时间差，不能通过关闭时效保护发车。现在允许仅 20ms 的未来传输时差，300ms 的过期上限不变。旧版只显示笼统 timestamp stale，无法区分旧数据与未来时间戳。

若 ping 不通、status/start 服务不可用，需要重新启动终端 C 的导航进程，而不是反复调用 start。查看终端 C 的退出码和 Traceback；没有退出日志时无法判断是关闭终端、ROS shutdown 还是进程异常。发车检查返回 False 的正常路径不会退出进程，也不会注销服务，该行为有假 ROS 传输测试覆盖。

仍无法发车时，请保留 status 的完整 message，以及导航终端 C 最后 20 行。必要时在终端中查看一次 `/clock`、`/tianracer/scan/header` 和 `/tianracer/odom/header`，用于比对发布时钟，不要修改比赛计时器。

## 92.463 秒完赛后的进一步优化

本次上传日志实际使用 `max_speed=4.0 lateral_accel=5.0`，官方圈时为 21.302、17.708、17.819、17.814、17.820 秒。已低于两分钟；首圈约多 3.5 秒。日志显示起步约两秒仍被限制在 0.35 m/s。地图准备和整圈规划都在发车前完成，不计入官方比赛时间。

新增两个可选开关，默认关闭，以保留本次成功配置：

- `fast_start:=true`：起步对齐速度随横向与航向误差改善，从 0.35 渐增到最多 1.0 m/s，再接入整圈速度包络。误差较大时仍回到 0.35 m/s；不改变 0.15 米误差、激光和地图保护，也不会每圈重新对齐。
- `optimize_time:=true`：在发车前比较四种走廊距离代价和不同周期平滑宽度，以完整闭环速度包络的预计圈时选线。包络包含横向加速度、转向变化率、航向变化率及加减速限制。保留障碍距离下限和航点顺序检查；不能减少比赛要求的检测线或穿墙抄近路。

保持本次成功的最大速度与弯道预算，先分别比较开关，再组合测试：

```bash
# 第一步：旧路线，渐进快速起步
roslaunch tianracer_gazebo planned_race.launch max_speed:=4.0 lateral_accel:=5.0 fast_start:=true

# 第二步：渐进快速起步，加预计圈时选线
roslaunch tianracer_gazebo planned_race.launch max_speed:=4.0 lateral_accel:=5.0 fast_start:=true optimize_time:=true
```

每条命令属于独立测试轮次，先停止上一轮导航，按前文流程准备和发车；不要同时启动。新选线在云端地图上将后轴参考路线由约 43.83 米缩短到 43.50 米，速度包络模型预计稳态圈时由 17.176 到 16.852 秒。这些是规划模型数值，不是新 Gazebo 成绩。多候选规划约三秒，等待 Ready 再准备发车。

假设 80ms 执行器响应、使用日志中的起步车姿，离线五圈跟踪验证了渐进起步与候选路线组合相对基线用时减少、地图距离和跟踪误差合格；没有模拟轮胎侧滑或真实定位噪声，不能把其绝对用时当作正式成绩。新路线是否通过所有官方检测线仍需确认裁判 5/5。

组合配置稳定完赛后，可单独尝试 `acceleration:=2.5`（原为 2.0），其余参数不变；新模块支持该参数并按同样加速度重新计算整圈速度包络。尚未实测证明车辆能稳定实现 2.5 m/s²，不同时提高最大速度、制动或横向加速度。比较至少三轮正式总用时和成功率；若变慢或保护触发，回到已完赛配置。`braking` 参数现也可传入，默认仍为 3.0，不能填写未经实测的更强制动能力来延后刹车。
