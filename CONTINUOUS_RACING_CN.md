# 第二版：连续闭环跟踪与弯前速度规划

上一版改的是 TEB 配置和输出平滑。它仍会换航点，低通与保守限速也可能抵消提速。
这一版新增独立的闭环 Pure Pursuit 控制器，以录制的完整有效路线为参考：

- 连续跟踪整条路线，不再发送四个 move_base 目标，每圈起点也不停车换目标。
- 用前方路线曲率生成整圈速度上限，以前向加速、反向制动约束提前减速。
- 直线加大预瞄以减轻左右摆动，弯道缩短预瞄，出弯按加速约束提速。
- 定位/传感器失效、偏离路线或近距离障碍时停止；指令断流由独立 watchdog 停车。

没有改变官方赛道、模型、裁判、检测线、成绩数据或物理参数。
这符合用户提供文档中保留官方环境并完成有效五圈的要求；是否正式有效仍由官方校验和裁判决定。
本模块不会识别官方检测线，也不会自行生成官方成绩。

## 0. 确认正在运行新版代码

用户日志中的路径缩写 `/home/tianbot/tia...` 可能指向旧的 `tianbot_ws`。
请在每个新终端执行以下加载命令，确认实际包路径：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

应为 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。
若仍指向旧目录，检查是否克隆到正确的 src 目录、构建是否成功；必要时运行 `rospack profile`。
不要同时加载两份同名 ROS 包或运行两套仿真。

更新与构建（已有本地修改时先保存或合并，勿强制覆盖）：

```bash
cd ~/ican_race_ws/src/tianracer
 git pull --ff-only
cd ~/ican_race_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make
source devel/setup.bash
cd src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_race_*.py' -v
```

需要 numpy、tf、rosgraph 和标准 ROS 消息，已加入包依赖声明。
所有命令仍使用比赛的 ROS1 Noetic / Gazebo 11 / Python 3.8 环境。

## 1. 先录一圈有效路线（只需准备一次）

不要将示例四航点直接插值成竞速路线。四点之间可能存在弯道和多条可选路线，
直连会抄近路、穿墙或漏检测线。本模块不预置未经验证的路线。

先按原流程启动 TEB Demo、裁判和 `waypoint_laps.launch`，使用能有效完赛的配置。
让小车进入稳定跑圈；在完成首次起点接近之后，尽量在起终点附近启动下面的录制。
可先在一轮本地练习中准备路线，不要占用正式成绩轮次进行首次调试。

新终端加载上述环境，然后：

```bash
rosrun tianracer_gazebo record_race_line.py __ns:=/tianracer _output:=$HOME/ican_race_logs/valid_lap.csv
```

录制器只读取已有定位 TF；不控制车、不修改裁判。
它在行驶超过 20 m、接近记录起点 0.25 m 内且朝向一致时保存闭环 CSV 并退出。
跳变、失去定位或重置仿真时钟会拒绝录线。文件已存在时拒绝覆盖，重录请换名字。

**必须观察裁判：录下来的这段路线应覆盖一整圈，并且裁判增加一个有效圈。**
若不是一整圈、包含恢复倒车/小环绕/漏线，丢弃该 CSV，重录干净的有效路线。
自动几何闭合只是文件检查，不能证明比赛路线有效。
记录起点若无法再次接近，录制不会完成；选择稳定圈中会再次经过的位置重录。
保留 `valid_lap.csv`，以后运行都要使用此文件。它不在 GitHub 中，必须由你本机录制。

## 2. 启动连续控制（替代多航点程序）

停止本轮旧的跑圈和仿真。在新一轮发车前，让车在录制路线上的起终点附近停稳，朝向与路线一致。
官方车型、赛道和模型校验应正常。只在准备发车前调整位置，不能计时中重置车辆。

终端 A，启动官方原场景、定位和底盘，但关闭 move_base：

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch world:=tianracer_racetrack robot_name:=tianracer start_navigation:=false race_profile:=false command_watchdog:=true
```

这里必须 `race_profile:=false`，避免叠加上一版的转向低通与曲率限速。
`command_watchdog:=true` 在 0.5 个仿真秒没有新指令时发布零速，独立于平滑模块。
本模式不要再启动 `waypoint_laps`、TEB 或其他 cmd_vel/ackermann 控制程序。

终端 B，启动裁判并按官方步骤绑定、检查环境：

```bash
roslaunch tianracer_sim_referee referee.launch
```

终端 C，启动连续控制器；此时它保持等待，不主动开车：

```bash
roslaunch tianracer_gazebo continuous_race.launch path_file:=$HOME/ican_race_logs/valid_lap.csv max_speed:=2.4 lateral_accel:=3.0 laps:=5
```

裁判点击“准备发车”，显示等待发车后，终端 D 也加载环境，然后：

```bash
rosservice call /tianracer/continuous_race/start
```

服务应返回 `success: True`。否则根据 message 处理传感器、朝向、离线路线或其他控制节点问题。
启动会检查其他 `cmd_vel` 发布者；直接向 Ackermann 输出的其他节点仍须手动停止。
运行中不得再启动其他控制节点。

手动停止命令（裁判的“结束本轮”不会刹车）：

```bash
rosservice call /tianracer/continuous_race/stop
```

到达五个参考闭环会停车，但参考圈数不等同于裁判有效圈数。
如果裁判先锁定五圈，可直接停止；如果参考五圈结束但裁判不足五圈，说明路线或起点不合适，
应修正后重新准备一轮，不能修改结果补圈。
异常停车后先诊断原因；为可比较的成绩测试建议准备新轮次，不在同一轮不断重启累计。

## 3. 逐步提速，验证实际收益

先以 2.4 m/s 检查稳定完整五圈，再试 2.7、3.0。这只是本模块速度上限，弯道仍由曲率和制动距离决定。
若旧版在 1.8 m/s 已不能稳定完赛，先从 1.8 验证路线和追踪，不直接升速。
`lateral_accel` 是规划的横向加速度预算，默认 3.0 m/s²，不是官方规则上限或已测得轮胎极限。
不得仅提高它来消除限速；出现转弯滑移、冲墙或误差增大时降低速度和该预算。

新控制器对障碍采取降速/停车，没有 TEB 的动态绕障恢复能力；本方案针对固定赛道的干净有效路线。
如果路线经过狭窄区域、录线质量差或定位噪声较大，可能更慢甚至停车，需要先改善录线与定位。

每个档位记录至少三轮官方五圈用时及有效完赛率，比较中位数和最快值。
观察终端打印的路线偏差 error、指令速度 v，并录 rosbag：

```bash
mkdir -p ~/ican_race_logs
cd ~/ican_race_logs
rosbag record -O continuous_24_run1.bag /clock /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/odom /tianracer/scan /tianracer/amcl_pose /tf /tf_static
```

同时保存裁判的有效圈数、最终用时与终端错误。不能用导航打印的参考圈数或电脑墙钟替代官方成绩。
如果仍然慢，提供 `valid_lap.csv` 和 rosbag，可区分路线曲率、碰撞限速、定位误差和实际车速问题。

## 验证范围与回退

本次共 29 项离线回归测试通过，新增验证包括：闭环速度包络的加速/制动约束、弯前减速、
简化自行车模型的连续两圈跟踪、路线偏离停车、错误朝向、激光堵塞、输入超时和控制节点冲突。
ROS 适配测试使用假消息与假时间；它们不是 ROS/Gazebo 集成测试。
源码按 Python 3.8 语法检查，launch/XML 和参数传递也经过检查。

当前云环境没有 ROS/Gazebo，未实测你的路线，未验证正式模型资格和官方五圈成绩。
没有宣称该版本已节省多少秒，也不能用离线圆形路线仿真证明官方赛道更快。

回退：停止连续控制，恢复 Demo 的 `start_navigation:=true`，按上一版教程启动多航点。
未修改原有航点或官方模型，无需重录地图；保留原版以便做同条件对比。
