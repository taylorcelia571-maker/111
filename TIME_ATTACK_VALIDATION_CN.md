# 从 87.546 秒继续提速：导航控制与制动规划修正

**本次已实现代码并完成可复现离线验证，尚未完成官方五圈小于 60 秒实测，不能保证“最大速度且不翻车、不碰撞、不保护制动”。** 这些条件必须在官方车辆动力学、相同赛道与实际反馈下验证，不能由速度参数或理想模型代替。

## 最新实测与真正的瓶颈

你提供的官方五圈为 87.546 秒：19.224 / 16.963 / 17.082 / 17.044 / 17.233 秒。后四圈平均 17.0805 秒，首圈只多约 2.14 秒；消除这部分启动损失，总用时仍约 85.4 秒。60 秒还需要约 30% 的持续圈时改善，不能只加快发车。

实际使用 quick_start_race：max_speed=10、lateral_accel=6.5、acceleration=6.6、braking=3.6、yaw_rate=3.8、steering_feedback_gain=0.25、response_delay=0.25、route_margin=0.34、refine_line=true。规划峰值只有 6.55m/s，max_speed 已不约束路线；提高到 11 或 12 不会提高这个包络峰值。

日志中的反复降速主要包括 response_preview、steering_slew、outward_error_recovery。起步 1.18 秒左右曾从约 1.80m/s 实测速度请求降到 0.20m/s；弯道也多次有实测速度高于目标 0.5–1m/s 的警告。不能通过取消限速或减小墙距来消除这些警告。稀疏 INFO 和触发 WARN 的抽样不是全周期统计，不能用它们准确测出电机响应时间或车辆极限。

## 你补充的全周期统计

该报告是六个参考闭环、104.917 秒，official_result=false、last_fault=null；不能与截图的官方五圈 87.546 秒混成一个计时区间。

| 实测项 | 本轮数据 | 对修改的意义 |
| --- | ---: | --- |
| 控制计算平均/峰值 | 1.12 / 3.08 ms | 20ms 周期内计算余量充足，继续精简 CPU 不是主攻方向 |
| 控制周期平均/峰值 | 20.00 / 24.00 ms | 没有这份报告证明的严重调度卡顿 |
| response_preview | 51.24s，48.8% | 应检查制动包络与反馈；这不是可直接删除的 51 秒 |
| 加速限制 | 16.713s，15.9% | 既有起步，也有每次降速后的重新加速，不能全部算首圈 |
| 转向跟随限速 | 6.64s，6.3% | 验证新的横向控制与实际关节跟随 |
| 弯道行驶 | 78.80s，75.1% | 需要提高持续过弯能力，单独追求直道峰值不足 |
| 弯道/直道及缓弯平均实测速率 | 2.33 / 3.23m/s | 当前均速远低于 max_speed=10，峰值已到 5.97m/s |
| 启动连接限速 | 2.20s，2.1% | 有改进空间，但不能填补 27.55s 目标差距 |
| 实测启动等待 | 0.077s | start 接受后车辆立即起步，不是长时间等待服务器 |
| 最大参考偏差 | 12.69cm | 距 15cm 停车保护仅约 2.3cm，无法据此证明翻倍过弯预算安全 |

reason 分类是在上一个指令生效的时间段归属，曲线/圈次/阶段分类与 reason 分类有重叠，不能相加。虽然 0.25 秒预留占用很多周期，它仍承担真实制动风险处理；不会为凑目标直接设为零。约 44.7 米路线在 60 秒内完成五圈需要平均约 3.725m/s，相比报告整体平均约 2.554m/s 需要约 46% 提升。汇总报告没有同步指令/关节/速度原始序列，仍不能准确辨识执行器响应或真实制动上限。

## 本次算法和规划器修改

* **新增 race_feedback 横向控制**：使用参考曲率前馈和连续航向、横向误差反馈，支持实际静止位置到参考线的 C2 连接轨迹。保留实际关节补偿、转角与转角变化率限制；与之前 curvature_feedback 相比，支持起步连接轨迹，避免强行先纠正起点约 7cm 偏移。
* **起步按真实连接轨迹投影**：此前按名义线投影进度，再在偏移连接线上计算控制误差，两者可能不一致。现在连接阶段投影到实际 C2 轨迹，同时仍按原始名义线检查 15cm 偏差保护。
* **条件释放启动限速**：新入口至少行进 0.6m，且跟踪误差 <2.5cm、航向误差 <0.05rad、目标与实测转向误差 <0.05rad，持续 0.3 秒后释放起步限速。此时可使用正常加速度，不再一律等待前 5m。未满足条件保留 1.8m/s、2m/s² 的起步预算；再次 start 会清除就绪状态，不会在每个圈缝重做启动。旧入口的 5m 保守策略保留。
* **新增 delayed_braking 制动可达包络**：从曲率/偏航/转向变化率原始上限出发，按每个未来约束保留 `v*response_delay+(v²-u²)/(2*braking)` 的滑行和制动距离，再传播加速度限制。在线不再对已经保留这段距离的包络再做完整相同位移预留；实测速率超过该点计划速度时，仍对额外行驶距离提前限速。旧 legacy 模式保留作对照。这里仍假设配置的 braking 可被车辆实现，不能仅由模型确定其物理能力。
* **速度预算入口**：time_attack_race 默认上限仍为 10m/s，横向预算 8、加速 8、制动 4.5、偏航 4.5；与已完赛预算相比提高实际生效的项。它是需要实际验证的候选预算，不是已测得的无碰撞最大档，更不是已验证的一分钟档。

原版车辆/赛道/裁判、0.34m 路线净空、0.23m 靠墙停车、0.15m 偏差停车、雷达及反馈新鲜度检查保留。新模式需要有效转向关节反馈。正常算法不主动紧急制动，但风险或反馈异常时保护仍可能请求强减速/停车；本轮不能保证消除所有这种情况。

## 已完成的验证与未通过的目标

使用仓库实际地图、你的实际启动位置 (-0.1665m 后轴 y)、相同闭环/0.25s 预留，分别模拟 80/150/200ms 一阶执行器响应，包含关节反馈。以下均为五个参考闭环模型时间，**不是 Gazebo 或官方五圈时间**：

| 假设响应 | 你的旧配置模型 | 新算法同预算 | 新算法提高预算 |
| --- | ---: | ---: | ---: |
| 80ms | 86.40s | 83.50s | 76.26s |
| 150ms | 88.32s | 85.50s | 78.66s |
| 200ms | 94.84s | 89.06s | 83.50s |

150ms 对照首圈：19.06 → 18.06 → 16.70 秒。启动限速释放约 2.26 → 1.20 秒。同预算单周期指令降速峰值在该模型约从 27.6 降到 7.3m/s²，但这不是实测车身制动减速度，也不是无紧急制动证明。提高预算在部分响应条件下仍有明显指令降速，必须检查真实日志再决定是否保留。

218 项非 ROS 全集测试通过，Python 3.8 语法和启动 XML 检查通过；航点 ROS 消息测试与 catkin/Gazebo 验证未执行。新增回归检查制动距离、圈缝连续性、加速度约束、初始连接段限速/连续就绪/状态重置、硬偏差停车、延迟关节和三种执行器响应下五圈跟踪。对照中还试过更长的转向预瞄及额外前馈，模型部分条件反而更慢，因此没有把它们作为新入口默认项。

更激进 14m/s² 横向预算、7m/s² 制动预算的**未认证能力假设**可将包络计算压到 59.87 秒，但闭环模型仍为 61.60 / 66.84 / 82.04 秒；不包括轮胎侧滑、真实 AMCL 和翻车动力学，不能把这组假设当作一分钟方案。本次没有找到满足你全部条件、且已验证五圈 <60 秒的最大参数组。

尝试建立独立 ROS Noetic/Gazebo 环境做原版动力学测试；精简 ROS 容器已运行，但 snapshots.ros.org 返回环境网络策略 403，Gazebo 依赖无法安装。因此本次 Gazebo 与官方赛道动态实测未执行，不能引用历史 Gazebo 报告替代本次验证。完整模型证据见 [TIME_ATTACK_OFFLINE_RESULTS.json](TIME_ATTACK_OFFLINE_RESULTS.json)。

## 更新与实测顺序

先保留已经获得的正式成绩，结束上一轮、停止 C 和旧仿真，更新仓库：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
```

每个新终端加载 Noetic 与该工作空间。终端 A、B 沿用你的已验证官方环境，重新生成默认静止起点：

```bash
# 终端 A
roslaunch tianracer_gazebo planned_race_demo.launch
# 终端 B
roslaunch tianracer_sim_referee referee.launch
```

终端 C **先以同预算对照新算法**，保持窗口打开，等待 Ready：

```bash
roslaunch tianracer_gazebo time_attack_race.launch lateral_accel:=6.5 acceleration:=6.6 braking:=3.6 yaw_rate:=3.8 run_report:=$HOME/ican_race_logs/time_attack_same.json
```

D 检查参数及预转向：

```bash
rosparam get /tianracer/continuous_race/lateral_control
rosparam get /tianracer/continuous_race/speed_planning
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

预期 race_feedback、delayed_braking、prepare=True，status 中 steering_ready=True。保持静止，在裁判确认官方环境和有效绑定并“准备发车”后执行：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

官方五圈完成后立即 stop，确认车体和里程计静止：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rostopic echo -n 1 /tianracer/odom/twist/twist
```

相同预算新算法连续三次有效完赛、实际首圈与持续圈速改善、无翻车/碰撞/保护停车后，再从全新起点测试提高预算的候选：

```bash
roslaunch tianracer_gazebo time_attack_race.launch run_report:=$HOME/ican_race_logs/time_attack_raised.json
```

这组参数不宣称可达到 60 秒。出现风险限速、速度大幅超出目标或控制停车时保留报告、status 和完整控制日志，停止本轮并重置场景；不要继续增加预算或在故障车姿重复 start。回退为你已获得 87.546 秒的配置：

```bash
roslaunch tianracer_gazebo quick_start_race.launch legacy_control:=true steering_feedback_gain:=0.25 acceleration:=6.6 braking:=3.6 refine_line:=true run_report:=$HOME/ican_race_logs/slim_tuned.json
```

## 留证与目标验收

每轮保存官方各圈/总时间、实际启动命令、完整 C 日志、run_report、planned_course_report.json、planned_course.csv。控制数据可录制下列话题，不要额外订阅或录制 /gazebo/model_states：

```bash
rosbag record -O time_attack_run1.bag /clock /tf /tf_static /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/odom /tianracer/joint_states /tianracer/scan /tianracer/amcl_pose /tianracer/imu /rosout_agg
```

IMU 话题以实际发布名为准；当前源代码不读取 Gazebo 模型真值作为导航输入。cmd_vel.angular.z 在本仓库为转角指令。测量真实制动和转向滞后需要同步指令、实测关节与速度，不能只用 WARN 抽样列推断，也不能把 commanded max_speed 当实测均速。

根据本次完整 per-cycle 报告的 time_by_state、startup、cycle_wall_timing 确定下一步。为了确定可实现的最大稳定预算，还需要真实车辆的减速能力、转向跟随、车身姿态和持续跟踪误差；这次提供的稀疏日志不足以证明更高横向预算不会翻车或滑出赛线。

最终一分钟验收仍是官方有效 5/5、原始用时 `<60000000` 微秒、原版环境通过校验、有效绑定且平台受理；建议同组参数连续三次满足所有运行条件。模型包络 <60、参考闭环 <60 或只快首圈都不算实现你的目标。

## 离线复现

在源码仓库运行（numpy、scipy、PyYAML）：

```bash
python3 tianracer_gazebo/scripts/time_attack_audit.py --output ~/ican_race_logs/time_attack_offline.json
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_time_attack.py' -v
```

实际 ROS 环境再运行全部测试：

```bash
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_*.py' -v
```

云端非 ROS 全集需排除 test_waypoint_laps.py，真实航点消息测试应在 ROS2GO 单独确认；测试通过不代表上述正式目标已经达标。
