# 本地克隆与验收

这是基于用户上传代码制作的优化版，不是官方发行版。
正式成绩仍要求官方赛道 `tianracer_racetrack` 版本 `2026.09.1`、原版车辆及有效裁判校验。
云端没有 ROS/Gazebo；当前仅通过 12 项离线控制测试和配置检查，尚未实测圈速。

## 1. 克隆与构建

使用比赛指定的 ROS2GO：Ubuntu 20.04 x86-64、ROS Noetic、Gazebo 11、Python 3.8。
已有工作空间依赖时，将本仓库用于源码目录。不要把原版与本版同时放在同一工作空间，
也不要叠加加载两份同名 ROS 包。下面用独立工作空间避免覆盖原来的源码：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/ican_race_ws/src
cd ~/ican_race_ws/src
git clone --branch main https://github.com/taylorcelia571-maker/111.git tianracer
cd ~/ican_race_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
rospack find tianracer_sim_referee
```

`rosdep` 需要已初始化；若缺少源代码依赖，先依据构建错误和官方环境说明补齐，
不要忽略构建失败直接运行。新工作空间不保证具备 ROS2GO 原工作空间的全部依赖。
两条 `rospack find` 都应指向 `~/ican_race_ws/src/tianracer/`。
裁判二进制必须与比赛指定的 Python 3.8、系统架构及官方版本匹配。

## 2. 离线逻辑测试

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_race_*.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_waypoint_laps.py' -v
```

第一条应运行 12 项并全部通过。第二条依赖 ROS 消息；云端没有运行，需在本机验证。
任何失败都先解决；逻辑测试通过不代表官方模型校验和圈速通过。

## 3. 三个终端运行

每个新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

终端 A（先以 1.8 m/s 验证优化后的转向）：

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch world:=tianracer_racetrack robot_name:=tianracer race_profile:=true race_max_speed:=1.8
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

确认车辆静止、官方环境校验通过、赛事绑定有效；点击“准备发车”。
裁判显示等待发车后，终端 C：

```bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

运行前退出旧的多航点和其他控制节点。
以裁判显示的五个有效圈及完赛状态为准；导航打印五圈并不足以验收。
正式提交还需在提交窗口内点击提交并确认平台受理。
本届文档给出的截止时间为 2026-10-15 23:59 北京时间。

## 4. 验收标准与前后对比

| 验证项 | 合格证据 |
| --- | --- |
| 项目可运行 | catkin 构建成功，ROS 包路径正确，启动无持续错误 |
| 符合正式环境 | 裁判确认官方赛道、原版车辆、有效绑定与可上报资格 |
| 路线有效 | 按顺序过全部检测线，裁判锁定五个有效圈 |
| 提速 | 同一官方环境、同一起点条件，优化版五圈用时小于基线 |
| 过弯流畅 | 车辆少左右摆动、少贴墙及停车；结合转角和速度记录比较 |
| 稳定性 | 每档至少三次有效完赛，同时记录失败、漏点和碰撞，不能只挑最快一次 |

基线：在终端 A 使用 `race_profile:=false`，其他条件一致。
注意本仓库基础控制频率已修正为 10 Hz；若要与上传原版的 5 Hz 完全对比，
请在另一独立工作空间使用原始 ZIP，同一时间只运行一套仿真。
优化档：先 1.8，再依次 2.0、2.2、2.4。每次调速重启自己运行的环境并准备新一轮。
记录每次官方五圈总用时，报告中位数、最快值和有效完赛次数。
圈速由仿真时钟计算，不能以录屏时长或电脑墙钟代替。

运行时检查频率与参数（新终端也要加载环境）：

```bash
rosparam get /tianracer/move_base/controller_frequency
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_vel_x
rosparam get /tianracer/nav_sim/race_profile
rostopic hz /tianracer/cmd_vel
rostopic hz /tianracer/ackermann_cmd_stamped
```

优化配置目标为规划 20 Hz、输出 50 Hz；以实际仿真运行负载为准。
可在跑圈前记录控制、激光和定位数据，用于复查：

```bash
mkdir -p ~/ican_race_logs
cd ~/ican_race_logs
rosbag record -O optimized_18_run1.bag /clock /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/odom /tianracer/scan /tianracer/amcl_pose
```

确保日志 topic 与本机实际发布的一致。用不同文件名保存每次测试，完赛后 Ctrl+C 停止录制。
将裁判圈数、五圈用时、配置档位和异常现象一起记录；日志本身不能替代官方成绩。
详细改动、平滑参数和限制见 [RACING_OPTIMIZATION_CN.md](RACING_OPTIMIZATION_CN.md)。
