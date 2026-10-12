# 当前 82 秒成绩：提速瓶颈与完整验证流程

**目标尚未达到：你最新提供的官方五圈成绩为 82 秒，而不是 60 秒以内。** 这里提供可重复的测试和瓶颈检查流程，没有一组已经实测证明能稳定小于 60 秒的参数。云端是 Debian 13，没有 ROS Noetic/Gazebo；离线地图、自行车模型和假设执行器延迟测试不能替代你本机的官方五圈结果。

## 先明确哪些修改能影响用时

此前 43.50 m 路线在 max_speed=8、lateral_accel=6.5、acceleration=5、braking=3、yaw_rate=3.8 下，规划峰值只有约 5.99 m/s；最高速度改成 9 没有收益。旧包络模型五圈约 72.04 秒，尚未包含起步、在线响应预留、误差限速和真实动力学。这个模型不是物理极限证明，但说明当前参数没有支持 60 秒的证据。

你此前各圈 20.961/16.050/16.084/16.093/16.037 秒，首圈额外耗时约 4.9 秒。日志中实际开始运动只用了 0.071 秒，随后是低速对线阶段。因此要优化的是对线和执行响应，而不是裁判按钮或 ROS 启动时间；计时前的地图规划耗时不会算进本轮成绩。即使省掉全部首圈额外耗时，85.225 秒仍约为 80.3 秒。

约 43.5 m 路线在 60 秒内完成五圈，平均速度须超过 3.625 m/s，含起步时要求更高。提高 max_speed、只减少起步数秒、或当前逐弯细调的少量模型收益，都不能单独完成这个目标。另一方面，lateral_accel=6.8 已出现过保护停车，不能把更大横向预算当成已验证的安全弯速。

`optimized_race` 当前默认 2.2 m/s，是墙距恢复档；不要把它称作比你的 82 秒配置更快的方案。先保留已完赛的实际控制模式与参数，测量哪些限制占据时间，再测试单一修改。`cmd_vel.angular.z` 在这里表示转角指令，不能当作实际偏航角速度。

本版增加同一参考路线指纹及 `check_race_result.py`，检查计划/运行报告是否一致，列出在线限速占时、参考首圈额外耗时和保护停车。官方圈数和用时由你照裁判原始结果输入；工具不修改结果、不独立核验赛事绑定，也不上传成绩。离线审核新增 `--lateral-control`，避免用另一种控制器的模型解释你的配置。

## 1. 更新并检查包来源

先让车停车、在裁判结束失败轮，再退出旧控制、裁判和仿真。每行单独执行，`setup.bash` 要用 source 加载，不能直接运行，也不能与 roslaunch 拼成一行。

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

两个包必须来自 `~/ican_race_ws/src/tianracer`。若 pull 或构建失败，先处理错误。不要继续启动旧包。保留本地修改，不要使用 reset --hard。每个新终端均执行这两行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

## 2. 从默认起点启动场景、裁判、控制

终端 A：

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

这是官方赛道/车辆、定位及控制桥的入口，不运行 move_base。不要同时运行 waypoint_laps、TEB 导航、遥控或另一套 continuous_race。不要订阅或录制 Gazebo 真值 `/gazebo/model_states`；只让裁判读取它。

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

裁判应识别 `tianracer_racetrack` 和 `tianracer`，官方版本及模型校验有效，正式测试绑定有效。先不要发车。

终端 C：首先沿用你最近稳定完赛的实际命令。若需要对照，下面是此前提供完整完赛日志的 smooth_pursuit 参数；它不是已经验证的亚 60 秒方案，也不声称是最新 82 秒测试的完全相同配置。

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false
```

保持 C 打开，等到 `Ready`。规划只在静止时进行，运行中沿同一个闭合参考线前进；不会每圈重新发送终点停车目标。参考圈数 6 不等于官方圈数 5。

终端 D，在发车前检查参数和传感器：

```bash
rosparam get /tianracer/continuous_race
rosservice call /tianracer/continuous_race/status '{}'
rostopic info /tianracer/cmd_vel
rostopic echo -n 1 /tianracer/odom/twist/twist
```

确认当前控制参数正确、状态 idle、无故障、车辆静止（纵向速度不超过 0.05 m/s）。cmd_vel 只能有这一套车辆控制发布者；裁判不驱动车辆。缺消息时分别运行 `rostopic hz /tianracer/scan`、`rostopic hz /tianracer/odom`，观察后 Ctrl+C；不要并行启动大量检查工具影响计算。

## 3. 每轮独立录包和保存参数

额外终端 E 在发车前创建本轮目录。这里用 run01，每次换成尚不存在的 run02/run03，避免覆盖证据。

```bash
mkdir -p ~/ican_race_logs
mkdir ~/ican_race_logs/run01
cd ~/ican_race_logs/run01
rosparam dump parameters.yaml /tianracer
git -C ~/ican_race_ws/src/tianracer rev-parse HEAD > commit.txt
rosbag record -O telemetry.bag /clock /tf /tf_static /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/odom /tianracer/scan /tianracer/joint_states /tianracer/amcl_pose /rosout_agg
```

保持 E 录制直到停车。写下实际启动命令（含所有参数），保存裁判各圈用时和事件记录。这里只录公开定位/传感器/关节/指令，不能从真值反馈控制车辆。

## 4. 准备、正式发车、五圈后停车

终端 D，在静止起点预转向；smooth_pursuit 和 curvature_feedback 当前支持此服务：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
```

应 success=True，车轮可预转向但车体必须静止。失败时先保存 status 和原因，不要跳过错误或盲目提高限速。若最近完赛基线没有使用 prepare，比较时分别记录是否使用，不能混为同一条件。

在裁判确认环境/绑定有效后点击“准备发车”，裁判应显示等待发车、用时零。随后 D：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

应 success=True。还要持续观察 C：success=True 只表示接受起步，并不代表之后没有保护停车。裁判确认有效运动才开始计时。记录 `Race start`、`Startup`、限速原因及 ERROR。

**裁判显示有效 5/5、结果锁定后立即在 D 停车**，不要等待第六个导航参考闭环：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rostopic echo -n 1 /tianracer/odom/twist/twist
```

确认车体静止，再到 E 按 Ctrl+C 完成 bag。保存本轮报告：

```bash
cp ~/ican_race_logs/planned_course.csv ~/ican_race_logs/run01/
cp ~/ican_race_logs/planned_course_report.json ~/ican_race_logs/run01/
cp ~/ican_race_logs/control_run_report.json ~/ican_race_logs/run01/
```

如果中途停住：先保持 C 开着，用 D 保存 status 与 C 的 ERROR，不要再次 start 覆盖故障。明确停车后在裁判结束本轮，停止 C/A/B，重启官方场景恢复默认静止起点。清除裁判本轮并不能重置 Gazebo 或制动车辆。

```bash
rosservice call /tianracer/continuous_race/status '{}'
```

## 5. 检查实际结果，区分起步和持续限速

将 `--official-seconds` 改成**本轮官方实际秒数**，`--official-laps` 改成实际有效圈数。82 只是你提供的最新总时间示例，不要把它填到另一轮报告里。

```bash
cd ~/ican_race_ws/src/tianracer
python3 tianracer_gazebo/scripts/check_race_result.py --plan ~/ican_race_logs/run01/planned_course_report.json --run ~/ican_race_logs/run01/control_run_report.json --official-seconds 82 --official-laps 5 --output ~/ican_race_logs/run01/acceptance.json
```

退出码 0 表示输入的五圈时间严格小于目标、且报告一致无故障；退出码 1 表示未达标或证据不一致，退出码 2 表示输入/文件错误。60.000 秒本身不满足“60 秒以内”严格小于的目标。该工具的 `independently_verified_official_result` 始终 false，不能冒充官方验收。

重点看 `top_speed_restrictions`、`reference_startup_excess_seconds`、`last_fault`、`max_speed_is_binding`。phase/road/lap 分类相互重叠，工具不会把它们与限速原因混加。参考首圈、模型时间与官方计圈是不同口径。

更高弯速的验证必须从 bag 核对指令转角与左右关节转角、速度指令与里程计实测速度、偏航速率/横向速度、定位偏差和真实碰撞。实际车速大幅高于制动指令时，先解决执行响应，不能把 braking 参数翻倍当成电机已经能更快刹车。响应预留 0.25 秒不是已经测得的时间常数。

## 6. 正确比较候选，不把更慢的恢复档当成提速

每个参数组至少在相同默认起点跑三次，分别记录有效五圈、时间、故障、碰撞/漏线。候选三次均有效且最快值/中位数改善，才称为实测提速。每次仅改一个变量，保留所有安全保护和官方模型。

离线审核必须选择与实车相同的控制模式，以下为高预算 smooth_pursuit 的地图/假设响应审查，不会驱动车辆：

```bash
python3 tianracer_gazebo/scripts/race60_audit.py --max-speed 8 --lateral-accel 6.5 --acceleration 5 --braking 3 --yaw-rate 3.8 --response-delay 0.25 --route-margin 0.26 --lateral-control smooth_pursuit --simulate --lags 0.08 0.12 0.2 --output ~/ican_race_logs/high_budget_model.json
```

模型报告失败先查原因；完成也不证明真实车稳定。更大的 route_margin 要同时重新核对路线和性能，不应与旧 0.26 m 路线混比较。现有数据不足以给出“不翻车不撞墙的极限弯速”。拿到本轮 acceptance.json、两个原始报告和同步 bag 后才能量化响应滞后并决定下一项控制改动；只有一条 82 秒总成绩不能区分首圈对线、转向跟不上还是弯道限制。

本次按上面的命令实际执行离线审核，假设 80/120/200 ms 一阶执行器响应：当前选线五个参考闭环分别为 82.10/84.30/100.54 秒；逐弯细调分别为 79.40/81.16/97.50 秒。细调在前两种假设下的最小车辆中心净空仅约 0.2320/0.2317 m，离原 0.23 m 停车阈值只有约 2 mm，**不能将它作为已验证的高速安全方案**。这说明新增选线模块的模型收益只有数秒，没有支持 60 秒目标；不应为达到目标取消墙距或偏差保护。机器可读证据见 [USER82_OFFLINE_RESULTS.json](USER82_OFFLINE_RESULTS.json)。这些时间不含轮胎、定位和裁判检测线，不是官方成绩。

最终要求：官方环境和绑定有效、顺序通过检测线、有效完成五圈、原始总用时严格小于 60 秒，且无碰撞和保护停车。当前尚未满足该要求。上传正式成绩仍由你在裁判界面操作并确认平台受理。
