# 111 优化版：五圈 60 秒目标与完整验证流程

本轮以你提供的中途停车日志和 `finish_race` 配置为起点。**已完成代码优化和离线回归；尚未实现或验证官方五圈小于 60 秒。** 云环境没有 ROS Noetic/Gazebo，也未连接你的比赛设备，无法代替 ROS2GO 上的官方实测。

官方依据：[比赛规则](https://docs.tianbot.com/competition/roboracer_online/2026-ican/contest-rules)、[运行与测试](https://docs.tianbot.com/competition/roboracer_online/2026-ican/run-and-test)。正式赛道为 `tianracer_racetrack`（2026.09.1），车辆与赛道须通过官方校验；裁判使用仿真时钟，按顺序经过检测线才计圈。导航参考闭环不等于官方有效圈。本轮没有修改官方裁判、世界、车辆模型或比赛结果。裁判完成五圈后不会替你停车。

## 1. 本次改动与验证结果

路线打分此前忽略了控制器的 `response_delay` 提前减速预留。本版将这一预留用于选线与逐弯细调：按名义速度计算前方预留距离，取该段最低包络速度，用它比较候选路线。控制循环仍按实测速度提前减速；没有降低预留、提高偏差停车阈值或改变启动限速。这个打分是保守比较指标，不是执行器模型，也不是正式成绩预测。

新增 `optimized_race.launch`，以 `finish_race` 相同速度预算和 `curvature_feedback` 横向控制启用选线及逐弯细调。关闭起点连接线和姿态预测，保留新鲜关节反馈、激光、墙距与 15 cm 跟踪偏差保护。`finish_race.launch` 保留作回退基线。

新增 `race60_audit.py` 可在没有 ROS 的环境复现真实地图选线、速度报告和假设执行器响应的自行车模型。报告始终标记 `official_result=False`，不读取或改写裁判结果。

本轮在相同 2.2/3.2/1.5/2.0/2.5 预算下得到：

| 项目 | finish 基线 | optimized 候选 |
| --- | ---: | ---: |
| 后轴参考路线长度 | 43.83 m | 43.07 m |
| 原速度包络五圈，未含起步/在线预留 | 113.07 s | 109.35 s |
| 0.25 s 提前减速打分代理五圈，未含起步 | 118.86 s | 114.19 s |
| 假设 80 ms 一阶响应：五个参考闭环 | 119.52 s | 115.06 s |
| 假设 150 ms 一阶响应：五个参考闭环 | 127.58 s | 120.96 s |
| 假设 250 ms 一阶响应：五个参考闭环 | 155.28 s | 143.66 s |

三种假设响应均完成五个模型闭环，候选峰值参考偏差不超过 8.1 cm，车辆中心地图净空保持至少 0.23 m；候选中心参考线净空至少 0.26 m，原航点顺序检查通过。模型不含轮胎侧滑、真实 AMCL 收敛、Gazebo 动力学及裁判检测线，因此这些数字不能声称为真实提速或比赛成绩。

43.07 m 路线五圈在 60 秒内要求平均速度超过约 3.59 m/s，含起步时要求更高。2.2 m/s 恢复档仅长度/速度下界已约 97.9 秒，数学上不能达标。旧高预算 8/6.5/5/3/3.8 即便模型也仍约 81.54 秒；8/9/6/6/5.5 的试验预算在同一 0.25 秒预留及 150 ms 假设响应下仍约 72.56 秒。本轮不把这些高预算设置作为恢复完赛命令，也没有已证实的亚 60 秒参数组。

本轮 166 项非 ROS 测试全部通过（含 6 项新增路线/响应回归），Python 3.8 语法及 launch XML 检查通过。`test_waypoint_laps.py` 因缺少真实 ROS 消息模块未运行，catkin 构建和 Gazebo/官方实测也未运行。机器可读证据见 [RACE60_OFFLINE_RESULTS.json](RACE60_OFFLINE_RESULTS.json)。

## 2. 更新、构建和离线回归

先停车、结束失败轮，退出旧导航和仿真。若已有工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git status --short
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
rospack find tianracer_sim_referee
```

有本地修改或 pull/build 失败时先解决，不能忽略错误启动旧代码。两个包路径应指向此工作空间。首次安装可在独立 `~/ican_race_ws/src` 克隆 `https://github.com/taylorcelia571-maker/111.git` 为 `tianracer`，按官方环境说明补齐依赖并构建；不要同时加载两份同名包。裁判二进制要求官方指定架构和 Python 3.8 环境。

比赛 ROS 环境完整测试：

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_*.py' -v
```

无 ROS 的开发环境，仅运行非航点 ROS 消息测试：

```bash
python3 - <<'PY'
from pathlib import Path
import unittest
suite = unittest.TestSuite()
loader = unittest.TestLoader()
for path in sorted(Path('tianracer_gazebo/test').glob('test_*.py')):
    if path.name == 'test_waypoint_laps.py':
        continue  # 此项须在真实 ROS 消息环境另行运行
    suite.addTests(loader.discover(str(path.parent), pattern=path.name))
result = unittest.TextTestRunner(verbosity=1).run(suite)
raise SystemExit(not result.wasSuccessful())
PY
```

重现本版地图与响应模型报告（需要 numpy、scipy、PyYAML）：

```bash
python3 tianracer_gazebo/scripts/race60_audit.py --simulate --output ~/ican_race_logs/offline_audit.json
```

`response_preview_proxy_five_seconds` 为新增打分代理；旧 `model_five_seconds` 仍保留未包含在线预留的原始包络含义。两者均未包含起步、动态误差限速和官方判圈。`scoring_response_delay` 应与运行参数一致。本地库版本差异可能造成路线指纹或最后小数差异。

## 3. 四终端启动与消息检查

每个终端先加载同一环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

终端 A，从默认静止起点启动官方场景、定位和原控制桥（此入口关闭 move_base 导航）：

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

终端 C，保持窗口打开，先规划，不发车：

```bash
roslaunch tianracer_gazebo optimized_race.launch
```

等待 `Ready`；应显示 `curvature_feedback`、2.2 m/s 预算及关节反馈 required=True。规划失败不得 start。首次路线试验也必须让裁判检查全部有效过线，不能仅凭绿色路径判断合规。

终端 D 逐条检查，收到真实消息后继续：

```bash
rosparam get /use_sim_time
rosparam get /tianracer/continuous_race/lateral_control
rosparam get /tianracer/continuous_race/max_speed
rosparam get /tianracer/continuous_race/response_delay
rostopic echo -n 1 /clock
rostopic echo -n 1 /gazebo/model_states/name
rostopic echo -n 1 /tianracer/scan/header
rostopic echo -n 1 /tianracer/odom/twist/twist
rostopic echo -n 1 /tianracer/joint_states
rostopic info /tianracer/cmd_vel
```

预期 sim_time=true、控制方式 curvature_feedback、速度 2.2、预留 0.25；模型列表含官方赛道和车辆；雷达、里程计和转向关节有连续数据。cmd_vel 只能有本次控制器这个运动来源，不能同时运行 waypoint_laps、rapid_race、旧 move_base、遥控等。转向关节必须包含左右 steering_hinge_joint 并有位置；数据失效不能通过取消关节检查解决。

## 4. 准备、发车、官方完赛与停车

在 D 静止预转向（仅静止改变转角）：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
```

success=True 后，确认车辆仍在起点附近静止；如车轮抖动或模型校验异常先排查。正式成绩应在发车前完成正确赛事/队伍绑定与官方环境校验；本地未绑定练习不能赛后补绑提交。裁判点击“准备发车”，进入“等待发车”后才执行：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

同时观察 Gazebo、路径/定位和裁判事件：起步后裁判自动计时，全部检测线按序推进，直至裁判锁定有效 5/5。控制器允许六个参考闭环，避免参考五圈先结束；六圈仍不保证官方五圈。发现漏线必须修正导航路线，不能改裁判或补写结果。

裁判锁定五圈后立即停止：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rostopic echo -n 1 /tianracer/odom/twist/twist
rosservice call /tianracer/continuous_race/status '{}'
```

观察车体并确认里程计速度接近零。如果服务不可用，Ctrl+C 退出 C，确认没有其他速度来源，再显式零速并检查停车：

```bash
rostopic pub -1 /tianracer/ackermann_cmd_stamped ackermann_msgs/AckermannDriveStamped '{drive: {steering_angle: 0.0, speed: 0.0}}'
```

发生保护停车时保存完整错误和 status，先在裁判结束本轮；不要从故障位置反复 start。停止 A/C，重启官方场景回到默认起点，清除本轮并重新检测、准备。回退只需在全新场景把 C 换成 `roslaunch tianracer_gazebo finish_race.launch`。

## 5. 留证与基线比较

准备发车前在额外终端录制；每轮使用独立目录，避免覆盖控制报告：

```bash
mkdir -p ~/ican_race_logs/run01
cd ~/ican_race_logs/run01
rosbag record -O telemetry.bag /clock /gazebo/model_states /tf /tf_static /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/odom /tianracer/scan /tianracer/amcl_pose /tianracer/joint_states /rosout_agg
```

cmd_vel.angular.z 在本仓库是转角指令，不能当作实测偏航角速度。记录关节实际转角和里程计角速度，才能区分速度/转向滞后、侧滑与定位误差。新控制启动也可指定 `run_report:=/绝对路径/run01/control_run_report.json`。

完赛或停车后停止录包，保存：实际 Git 提交号与启动命令、`rosparam dump` 参数、`planned_course.csv`、`planned_course_report.json`、控制报告、完整终端日志、裁判界面/原始保存结果及事件记录。每轮至少记录：配置、五圈时间、各圈时间、有效完赛/失败、最大偏差、故障、碰撞/漏线。

按相同官方场景、默认起点、相同机器条件分别跑 finish 基线和 optimized 候选，各至少三次；同一次只运行一套仿真和一个控制器。比较有效完赛率、五圈最快值和中位数。候选必须全部按序过线，官方结果快于基线，且无保护停车、碰撞或漏线，才称为实测优化有效。离线模型测试不替代这一步。

## 6. 分档提速到 60 秒目标

先验证候选恢复档三轮有效完赛，再仅提高一个预算。示例第一档保持弯道、加速和制动预算：

```bash
roslaunch tianracer_gazebo optimized_race.launch max_speed:=2.4
```

每档先在静止离线审核相同参数，然后按上面整套流程在全新场景跑三次。2.4 档通过后可单独试 2.6，不能同时提高多个预算。若 `speed_ceiling_binding=False`，继续提高 max_speed 不一定获益，应检查报告中横向、偏航、转向变化率、加减速及在线 response_preview/steering_alignment/outward_error_recovery 限速原因。

你提供的日志显示实测 3.18 m/s 而指令只有 1.10 m/s。这说明更高制动预算没有现成证据支持，不能为了模型时间变小直接调到 braking=6。制动能力必须从同步实测速度及仿真时间核对，转向滞后必须从指令角与关节角核对；0.25 秒预留不是已测得的电机响应时间，不应直接减到 0.15 来“达标”。转弯能力也需看关节、里程计横向速度与偏航速率、跟踪偏差和实际碰撞。

针对已完成实测的一组参数，用审核工具评估改一个预算是否有收益：

```bash
python3 tianracer_gazebo/scripts/race60_audit.py --max-speed 2.4 --lateral-accel 3.2 --acceleration 1.5 --braking 2.0 --yaw-rate 2.5 --response-delay 0.25 --simulate --output ~/ican_race_logs/audit_24.json
```

模型任何延迟条件失败，先查看故障，不能直接把它当作已验证配置。如果模型打分已超过 60 秒，这组预算没有支持亚 60 秒的证据；即便模型小于 60，仍需实际五圈验证。要进入平均约 3.6 m/s 的目标区间，必须获得车辆在原版模型上的真实过弯、转向与制动能力数据。现有失败日志不足以确定这组参数；目前不提供“保证 60 秒”的高预算命令。

最终达标证据必须是官方校验通过、正确绑定、有效五圈、官方整数微秒总用时 **<60000000**，并保存原始结果；建议同参数再连续验证三次，确认可重复性。正式成绩需手动提交并在赛事平台确认受理；官方截止为 **2026-10-15 23:59（北京时间）**。录屏时长、模型五圈或导航 Completed lap 都不能代替正式成绩。
