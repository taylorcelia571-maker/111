# 无需录线的全局规划 / TEB 轨迹优化模式

用户提供的正式五圈基准为 **167.955 秒**，各圈 33.155、33.392、34.673、33.320、33.413 秒。
该结果是对比基线，不是新模式的实测结果。

录线的 `Connection refused` 表示无法连接 ROS master。先启动并保持仿真终端运行，
才能录 TF 路线；单独启动录制器不会启动仿真。当前这条替代方案不需要录制文件。

## 本次改动

1. 显式启用 `global_planner/GlobalPlanner`。此前只有 GlobalPlanner 参数，未选择插件；
   move_base 可能使用默认 Navfn，使这些参数没有生效。
2. A* 配合二次插值与连续路径提取，开启前向路径朝向；调整膨胀代价梯度引导路线离墙。
3. TEB 预瞄 5 m、时间分辨率 0.15 s，内/外迭代由 3×2 改为 4×3；
   同时考虑路径参考点、时间、加速约束和避障，不强制追逐最短贴墙路线。
4. 默认速度测试档 2.4 m/s，可调 2.6、2.8；提高转动速度预算，允许局部规划器重新协调入弯速度。
5. 关闭第一版额外输出平滑与曲率限速，让 TEB 直接控制；保留指令断流停车。
6. 修正局部激光 sensor_frame 中多出来的 `l` 前缀。

TEB 在运行时优化当前局部轨迹，不是离线生成整圈最优竞速线。
默认四航点、官方地图、赛道、车辆、裁判和成绩规则保持不变。
基础足迹、最小转弯半径、TEB 障碍间距也保留原值。
新参数是待实测配置，不是已经证明的最优设置；避障权重与平滑梯度也可能使部分路段变慢。

## 更新与依赖

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
```

你已经 catkin 构建成功，但 rosdep 本地缓存还没有准备好。
Noetic 已停止支持，更新缓存时需包含已结束支持的发行版；不要 sudo 执行 rosdep update：

```bash
rosdep update --include-eol-distros
```

若明确提示 `/etc/ros/rosdep/sources.list.d/20-default.list` 不存在，先执行一次 `sudo rosdep init`，
然后再执行上面的 update。已有该文件时不重复 init。网络失败应解决网络，不把失败当作成功。

```bash
cd ~/ican_race_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
rospack find global_planner
```

如果仅缺少 GlobalPlanner 包，可安装：

```bash
sudo apt install ros-noetic-global-planner
```

tianracer_gazebo 应来自 `~/ican_race_ws/src/tianracer/`。每个新终端都重新 source 两个 setup.bash。

## 三个终端直接跑圈

先退出上一轮自己启动的跑圈节点和仿真；不得同时运行连续路径控制器和多航点控制。

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planner_race_demo.launch max_speed:=2.4
```

等待 Gazebo、定位、move_base 正常。终端 B：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

完成绑定与官方环境校验；车辆在起终点附近停稳后点击“准备发车”。终端 C：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

第一轮保持 0.5 m 换点阈值，单独检验规划参数收益。
在路线和官方检测线都正常的前提下，再测试 `switch_distance:=0.8`，减少到点前停顿。
提高阈值可能改变路线或漏线；若漏线，立即回到 0.5，不继续增大。
首个目标和最后闭环依然要求导航成功，中间点仍沿用已有换点机制。
五次航点序列不自动等于五个官方有效圈，以裁判锁定五圈为准。

## 确认配置生效与逐档提速

启动后在另一个加载环境的终端检查：

```bash
rosparam get /tianracer/move_base/base_global_planner
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_vel_x
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /tianracer/nav_sim/race_profile
rostopic hz /tianracer/cmd_vel
```

应分别为 `global_planner/GlobalPlanner`、2.4、5.0、false，控制目标频率 20 Hz。
如果实际频率明显不足，更多 TEB 迭代可能超出电脑预算，先降低速度并检查计算负载。

每个速度档至少测三轮官方五圈总用时和有效完赛率。
先测试 2.4，再重启 A 测 2.6、2.8；不要同时改速度和换点距离，以便定位收益。
出现冲墙、左右摆动加剧、漏检测线或无法规划时，先回到已验证档位。
目标是比 167.955 秒更快且稳定有效完赛；没有保证某一速度一定达到该目标。

记录 `/clock`、`/tianracer/cmd_vel`、`/tianracer/odom`、`/tianracer/scan`、`/tianracer/amcl_pose`
以及裁判最终用时。如仍没有改善，可基于日志区分规划限速、跟踪误差和物理车速。

## 验证与回退

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_race_*.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_planner_profile.py' -v
```

本次通过已有 29 项离线控制测试及 4 项配置测试，检查参数覆盖、插件选择和模式隔离。
云端没有 ROS/Gazebo，未执行真实插件加载、catkin 构建或实测官方圈速。
你提供的构建和 29 项测试成功属于此前版本，不代表新规划模式已经实测通过。

回退时停止仿真，使用原来的 `demo_tianracer_teb_nav.launch` 入口，`planner_profile` 默认 false。
无需录线，也无需改动或替换官方模型。
