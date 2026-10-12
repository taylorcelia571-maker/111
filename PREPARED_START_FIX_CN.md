# 发车成功后 7.5 秒停车：预转向确认与起步入弯修正

用户日志确认 prepare/start 服务均 success=True，车辆实际运动在 start 后 0.075 秒发生。约 7.5 秒后仍在 departure，参考偏差达到 0.1540 m，超过原 0.1500 m，触发停车；TF 年龄 0.017 秒，有限投影与全局投影误差一致，没有日志证明的 TF 过期或跨段跳投影。该次路线净空 0.342 m，route_margin=0.34 已生效。

起步约一秒时，目标转角 -0.2270 rad，实际转角 -0.0294 rad，相差约 0.198 rad；实测速度 1.007 m/s，而指令 0.478 m/s。这支持检查转向/速度执行响应，不能仅凭一段日志断言全部由 PID 或轮胎侧滑导致。

## 本次代码改动

1. prepare 成功表示零速转向目标已发出，不再称为实际转向已经完成。status 新增 prepared_steering_error、steering_ready。
2. 使用 prepare 后，start 要求新鲜的实际转向关节反馈，与目标差不超过 0.04 rad；未就绪时返回拒绝原因并保持静止预转向。等关节就绪再发车，等待放在裁判正式计时之前，避免在首圈才转向对齐。未使用 prepare 的原始 start 仍保留原流程，未新增自动运动。
3. departure 原来无论弯道多急，前视距离都至少 0.55 m；现在该起步下限按路线曲率缩短，直道仍保留长前视。前视距离过长可能切弯，缩短前视又可能放大反馈误差，因此保留全部位置、关节、激光与墙距保护，并做离线延迟回归。

本轮没有放宽 0.15 m 偏差、0.23 m 墙距，没有降低 0.25 秒响应预留，也没有增加车辆/轮胎/PID 参数。仅有模型回归支持，尚未证实本机成功完赛、真实首圈提速或 60 秒目标。

## 更新并从默认起点复验

先停车、在裁判结束失败轮，退出旧控制、场景和裁判。更新时保留本地修改：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
```

每个新终端都分别加载 Noetic 和此工作空间。A 启动 `roslaunch tianracer_gazebo planned_race_demo.launch`，B 启动 `roslaunch tianracer_sim_referee referee.launch`，确认官方场景/车辆校验和正式测试绑定有效。C 保留预算，单独验证启动控制改动，等待 Ready：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=10.0 lateral_accel:=6.5 acceleration:=5.5 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit route_margin:=0.34 start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false
```

D 在裁判正式准备之前先执行：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

prepare True 只表示目标发出；**等 status 的 steering_ready 为 true，车体仍静止，再点击裁判“准备发车”**。prepared_steering_error 为 null 表示无法确认关节反馈，False 表示尚未对齐，不要直接发车或盲目提高速度。

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

start 应 True；若返回 still aligning，保持静止，查看 status，待 ready 再试。若实际关节始终不到位，先保存这些公开消息，排查控制桥/关节执行情况：

```bash
rostopic echo -n 1 /tianracer/cmd_vel
rostopic echo -n 1 /tianracer/ackermann_cmd_stamped
rostopic echo -n 1 /tianracer/joint_states
```

预转向期间 cmd_vel.linear.x 和 Ackermann speed 必须零，转角应有目标，实际左右转向关节应逐步响应。这些单条消息只能排查接线；定量时间延迟仍需要同步 bag。不要额外订阅 Gazebo 真值。

如果发车后再停车，保持 C 打开，保存 ERROR 和 status，不要反复 start 覆盖记录；结束失败轮后重建默认起点。若官方五圈有效锁定，则马上调用 stop 并保存本轮报告：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
cat ~/ican_race_logs/control_run_report.json
cat ~/ican_race_logs/planned_course_report.json
```

## 验证证据与局限

适配器回归检查：prepare 发布零速度，缺少/未对齐实际转角时拒绝 start 且 inactive，对齐后可以 start。曲率回归检查：起步弯道不强制 0.55 m，直道保留长前视；仍检查弯道退出对线时真实转角匹配。原真实地图上保留五圈跟踪、墙距及多延迟条件回归。

按同一 0.34 m 地图路线、10/6.5/5.5/3/3.8 预算、80/120/200 ms 假设响应，候选五个模型参考闭环仍约 89.08/91.20/108.46 秒，未出现显著时间下降。模型中初始转向执行是简化的一阶响应，没有复现用户日志全部瞬态，因此不能把模型通过当成本机启动问题已经解决。当前首要实测验收是预转向真实到位、第一弯不再保护停车、三次有效五圈；实际首圈用时收益另行比较。机器可读报告见 PREPARED_START_OFFLINE_RESULTS.json。
