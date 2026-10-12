# 首圈摆动与纠偏急减速：保留 6.5 预算的控制修正

最新两段日志显示 pursuit 模式的第一参考圈为 21.010 秒；后续用户反馈官方单圈约 16 秒。第一段曾出现 desired_steer=-0.48、输出 steer=-0.087、实测速率 1.59 m/s 与指令 0.78 m/s；后续多次因为 outward_error_recovery 压到 1 m/s。这里的 steer 是指令角，不是舵机实际角。附件未含 6.8 停车的 ERROR，不能确定该次故障原因。

本次不再增加 lateral_accel，新增可选择的 `smooth_pursuit`，保留旧 pursuit 作为回退：

* 起步使用较长前视距离，误差小于 2.5 cm、航向误差小于约 2.9°、转向指令已接近目标并持续 0.2 秒后，才解除起步对齐限速。此前允许约 8.6° 航向误差时解除限速。
* 对齐阶段的加速预算不超过 2 m/s²，对齐后恢复传入的 5 m/s²。减少带着未消除的偏差突然加速。
* 纠偏的前视距离和速度边界连续变化，去除旧逻辑一触发纠偏就将速度封顶为 1 m/s 的开关；保留同样的 15 cm 偏差停车保护。
* 用前方曲率变化补偿一部分转向需求，有限速保护，保持原转角上限和转角变化速率。
* 若收到新鲜有效的转向关节反馈，用它检查转向是否跟上；否则回到指令变化量检查，不沿用过期关节数据。必要时提前限速，避免以过高速率进入未完成转向的弯道。激光、地图墙距、定位与里程计时效保护仍保留。
* 增加静止 `prepare` 服务，只预转前轮，所有线速度指令为零，不移动车辆，也不开始控制器跑圈。`start` 沿用已经准备的转角，减少发车时的转向等待。

这项改动针对首圈额外耗时和纠偏顿挫，**尚未实测证明会缩短正式五圈成绩，更没有验证一分钟目标**。后四圈每圈 16 秒，仅消除首圈额外耗时也不足以实现五圈 60 秒。

## 本地更新

停止旧控制器、结束当前比赛轮，更新并构建：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

包路径必须来自 `~/ican_race_ws/src/tianracer`。每个新终端加载上述两行 source。重新启动场景，使车辆回到默认静止起点；不要从故障位置直接重新发车。沿用已跑通的官方场景和裁判配置，控制桥接不要同时启用另一套速度平滑/TEB 控制。

## 启动新模式

控制终端启动以下命令，保持开启，等待 Ready：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit
```

另一终端先加载环境，再调用静止预转向：

```bash
rosservice call /tianracer/continuous_race/prepare "{}"
```

只有 success=True 才继续。确认车辆静止，裁判完成官方环境检测、绑定，然后点击“准备发车”，再执行：

```bash
rosservice call /tianracer/continuous_race/start "{}"
```

prepare 不是必需操作，直接 start 也可运行新模式，但缺少提前转向步骤。不得在车辆行驶、其他控制器发布速度时 prepare。准备过程中反馈掉线会取消准备并发布零速度。控制器启动期间有其他速度发布程序时 prepare/start 均会拒绝。

确认模式：

```bash
rosparam get /tianracer/continuous_race/lateral_control
rosservice call /tianracer/continuous_race/status "{}"
```

应为 smooth_pursuit；状态中查看 measured_steer 是否为有效数值，以及 departure、alignment_ready_seconds、steering_alignment_risk、preview_length、reason。measured_steer=null 表示当前未采用实际关节角，而不是把零转角当作反馈。

五圈裁判锁定后执行 stop。比较首圈、五圈总时及出现纠偏限速的次数；有停车时保留 ERROR 和 status 的 last_fault。需要回退时，只删除启动命令中的 lateral_control:=smooth_pursuit，保留 6.5 预算，其余参数不变。

## 检查范围

开发检查包括真实地图上的五圈自行车模型：无转角反馈的 80/120 ms 假设执行延迟，以及有转角反馈的 200 ms 假设执行延迟；另外验证纠偏阈值附近连续性、未对齐不加速、静止准备保持零线速度、传感器过期取消准备和偏差保护。

200 ms 延迟且没有实际转角反馈的高预算模型仍可能触发偏差停车，不能声称该工况可稳定完赛；延迟较大时，新保护也可能让圈速变慢。上述模型不代表 Gazebo 中的轮胎、侧滑或定位效果。云环境没有 ROS/Gazebo，真实收益需要本地官方五圈验证。
