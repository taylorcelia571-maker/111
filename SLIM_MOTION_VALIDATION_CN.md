# 精简激光计算和指令转换，并验证提速

用户最新有效五圈约 91 秒，入口是 quick_start_race、steering_feedback_gain=0.25。附件中首个参考闭环 20.121 秒，没有控制周期超时或规划失败；日志抽样主要为 response_preview，随后为 acceleration、planned_envelope、steering_slew、tracking_lateral。抽样次数不等于各限速占用时间，需结合控制报告的 time_by_state。

## 排查结论与修改

此入口是闭环赛线追踪；planned_race_demo 的 start_navigation=false，不逐弯调用 TEB 在线优化器。赛线在车辆静止时生成。四轮运动学公式使用原版车辆几何，在 Noetic Python 3.8 下每次约 1.26 微秒，不是主要计算瓶颈，保留其滚动前馈公式。

真正热点在 reference_clearance：原算法在每个 50 Hz 控制周期用 Python 逐点采样预测车身，再对所有有效激光点与全部预测矩形计算距离。速度越高、制动视野越长，成本越大。本次修改：

* ClosedPath.sample_many 与 C2 起步连接段使用批量插值，代替逐点循环。
* 使用所有预测车身矩形的保守外包框，剔除不可能与任一矩形相交的远点；外包半径包含浮点余量。没有减小预测距离、采样密度、矩形大小或墙距要求，没有屏蔽侧面激光。
* transform.py 的原始控制模式立即转发新速度/转角，不再等下一次 20 毫秒定时器；定时器保留超时停车检测，不重复重放有效指令。重复的仿真时钟不把仍有效的原始指令误置零，时钟倒退仍停车。旧的速度平滑和弯道控制模式维持原流程。
* 控制报告新增 cycle_wall_timing，分开记录传感器/TF、追踪、激光的平均值、最大值、最近 256 周期的 P95。compute_to_command 是生成指令之前的墙钟计算耗时，不是官方比赛时间；simulation_interval 是仿真控制间隔。

209 项非 ROS 回归和 11 项真实 Noetic 航点测试通过；新增的指令即时转发时钟倒退保护另通过转换器 13 项复验，总计 221 项不同测试。180 组随机激光扫描、不同航向/传感器偏移/C2 连接段与旧版保护算法对照，首次命中位置、速度限制和诊断一致。回归覆盖无效扫描、真实超时、重复时钟、关节反馈、线程安全、接近赛线与起终点循环。

## 实际计算耗时

实际 Noetic/Gazebo 原版场景、相同 10/6.5/5.5/3/3.8 预算，各一个首圈：

| 测量 | 修改前 | 精简后 |
| --- | ---: | ---: |
| 生成指令前平均计算耗时 | 6.43 ms | 2.40 ms |
| 激光保护平均耗时 | 5.31 ms | 1.34 ms |
| 控制算法平均耗时 | 0.54 ms | 0.54 ms |
| 首个参考闭环 | 20.607 s | 19.877 s |

计算到指令的平均耗时减少约 63%，但圈速只改善约 0.73 秒；计算成本不是剩余全部圈速损失的原因。不同速度的合成扫描核测试也从约 3–15 ms 降至约 0.37–0.51 ms，但这不等于完整 Gazebo 或官方成绩。

## 赛线和速度调整

另测试 acceleration=6.6、braking=3.6、refine_line=true，保持 lateral_accel=6.5、max_speed=10、yaw_rate=3.8、response_delay=0.25、route_margin=0.34。逐弯细调接受 10 次几何修改；路线长 45.065 → 44.699 米，最低中心路线净空约 0.341 米，参考路径指纹也改变。仍保持同一航点顺序和赛道通道，正式检测线由裁判验证。

计划速度峰值 **6.04 → 6.55 m/s**。这次改变的是实际约束下的速度包络，而不是继续增大已经不生效的 max_speed。未经认证的加减速预算需本机对照，不能把一次仿真通过称为车辆物理极限。

该档实际完成五个参考闭环：**87.740 秒**，各圈 19.500 / 16.860 / 17.140 / 17.060 / 17.180 秒，没有保护停车，最大参考偏差 0.1202 米。此前相同云端原配置五个参考闭环为 92.679 秒；各配置单次测试，差异没有统计保证。公开 IMU 和里程计没有翻车姿态或低于 -0.1 m/s 的倒车样本。

另外测试移除额外曲率提前项，前三个参考圈没有明显收益，且遇到一次激光采样时间领先 TF 14 毫秒的异常。该对照未完成，不把停车归因于曲率参数，也没有把这项实验作为默认修改。已针对反馈获取修正：选取原 300 毫秒时效范围内、其采样时刻 TF 已验证的最新历史扫描；可用性检查与查找之间发生竞争时继续尝试其他时间戳，短时等待失败后再检查一次历史。原短缓存 100 毫秒、传感器 300 毫秒时限不延长；无有效变换或数据过期仍停车。测试覆盖 170 毫秒的可验证历史可用、超过 300 毫秒拒绝和 TF 短时竞争。

最终反馈修复后的复验也完成五个参考闭环：87.777 秒，各圈 19.537 / 17.040 / 17.100 / 16.900 / 17.200 秒；没有保护停车，最大参考偏差 0.1062 米。两次提速档结果接近，但仍只是两次云端参考闭环试验。

完整原始指标及最终复验见 [SLIM_MOTION_RESULTS.json](SLIM_MOTION_RESULTS.json)，包括失败实验。**所有云端时间是参考闭环，不是官方五圈成绩。**

## 更新与复验

先停止本轮控制，结束上一轮，退出旧控制、仿真和裁判。保留本地修改后更新，每行分别执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

包路径应指向 ican_race_ws。**必须重启仿真终端 A，才加载新的 transform.py。** 每个新终端先 source Noetic 和此工作空间。

终端 A：

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

确认官方环境、车辆校验和正式测试绑定条件有效。终端 C，提速对照档：

```bash
roslaunch tianracer_gazebo quick_start_race.launch steering_feedback_gain:=0.25 acceleration:=6.6 braking:=3.6 refine_line:=true run_report:=$HOME/ican_race_logs/slim_tuned.json
```

等待 Ready；终端 D：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

等 steering_ready=true 且车仍静止，点击裁判“准备发车”，再：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

以官方裁判五圈锁定为准，随后立即 stop 保存当轮报告：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
cat ~/ican_race_logs/slim_tuned.json
cat ~/ican_race_logs/planned_course_report.json
```

默认六个参考闭环只是留出官方判圈余量。需要录公开消息和完整更新说明，见 [完整发车和录制流程](STARTUP_GAZEBO_VALIDATION_CN.md)。本机对照旧预算与提速档各三轮，分别记录官方总时、首圈、后四圈、有效完赛率和保护停车；每轮重建默认静止起点并换报告文件名。回退预算使用 acceleration=5.5、braking=3.0、refine_line=false，精简计算和反馈修复仍然生效。

## 一分钟目标

本次改进有实际计算和行驶证据，**还没有达到一分钟**。提速档不含起步/执行器响应的五圈包络模型约 74.25 秒，加名义响应预瞄约 84.99 秒；模型不是绝对物理下界，但说明仅释放算力或提高 max_speed 没有证据达到 60 秒。该路线一分钟五圈需要平均约 3.72 m/s，当前紧弯仍受横向加速度、转向响应和跟踪约束限制。此次没有继续升高曾触发保护的横向预算，没有关闭保护、修改模型/PID/比赛时钟或跳过检测线，不能承诺永不保护停车。
