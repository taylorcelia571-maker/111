# 启动检查与首圈低速对线修正

> 最新发车后偏差停车修正见 [PREPARED_START_FIX_CN.md](PREPARED_START_FIX_CN.md)。prepare True 只表示目标已发出；需等 status 的 steering_ready=true 再准备裁判并 start。

用户补充的是场景终端 A 日志：模型成功生成，雷达插件、底盘及转向控制器都已加载。早期没有雷达的警告发生在插件生成前；停止/卸载的错误出现在 Ctrl+C 关闭之后。没有终端 C 的 Ready、start 服务返回或运行 ERROR，因此这份日志不能确定新版无法启动的原因，也不能证明控制程序已经运行。

## 已修正的起步逻辑

smooth_pursuit 原低速对线退出要求 `abs(desired_steer)<0.12`，相当于要求方向盘接近直行。若车辆进入曲线，即使位置和航向已经符合条件，也可能继续低速等待直道。这是首圈额外耗时的一个可检查来源，不能将截图全部约六秒额外用时都归因于它。

现在改为比较目标转角与当前路线曲率对应的转角；保留横向误差小于 0.025 m、航向误差小于 0.05 rad、转向指令跟踪差小于 0.04 rad、连续满足至少 0.2 秒。有新鲜关节反馈时，还要求实际转角与目标转角差小于 0.04 rad。弯道可以在对齐后退出起步低速阶段，不需要先把方向盘转直。不改变墙距、激光、最大跟踪误差或正常行驶速度包络，不在每圈冲线重新进入 departure。

start 拒绝原因现在写入 `Race start rejected` 日志和 status 的 `last_start_rejection`；成功发车后清除。失败仍保持静止，不自动越过检查。

此前受限起点连接线在本次 0.34 m 路线的假设模型中反而比普通对线慢约 0.6–0.9 秒。因此本次不推荐开启 start_blend 来宣称起步提速，继续保留 false。测试中的角度及时间不代表真实轮胎、定位与执行器已验证，实际首圈是否缩短须在本机比较。

## 更新与明确启动三部分

先显式停车、在裁判结束失败轮，退出旧场景和控制。保留本地修改，更新或构建失败时先解决错误：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

每个新终端都要执行这两行，不能拼在 roslaunch 前面或直接运行 setup.bash：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

A 启动场景，并保持打开。场景入口不主动让车跑圈：

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

B 启动独立裁判，确认官方环境、车辆与正式绑定有效：

```bash
roslaunch tianracer_sim_referee referee.launch
```

C 启动控制，等待 Ready：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=10.0 lateral_accel:=6.5 acceleration:=5.5 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit route_margin:=0.34 start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false
```

保持原速度预算以单独检验起步逻辑；这些预算不是已认证的安全极限。Ready 的路线中心最低净空至少 0.34 m；静止规划耗时在正式有效起步之前，不等于首圈用时。

D 检查控制服务并预转向，车体必须保持静止：

```bash
rosservice info /tianracer/continuous_race/start
rosservice call /tianracer/continuous_race/status '{}'
rosservice call /tianracer/continuous_race/prepare '{}'
```

若 start 服务不存在，检查 C 的完整启动输出、包来源及是否已退出；A 的插件日志不能替代 C。若存在但 prepare/start success=False，保存返回的 message/status，不要反复重启或忽略错误。传感器过期时检查仿真是否暂停及 scan/odom 是否持续更新。起点离路线太远或航向不符时，重新启动官方场景回到默认静止起点，不用放宽保护阈值强行发车。

prepare 成功后，确认裁判就绪、车辆静止，在裁判点击“准备发车”，再 D：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

成功后观察 C 的 Race start、Startup、departure 退出时间、后续 ERROR。若仍不能启动，请提供这一步实际 success/message 及 C 最后输出；当前还没有这类证据。

官方有效五圈锁定后立即停车：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

保存同一轮的两个 JSON、各圈用时及是否 prepare，用 [完整验证流程](RACE60_CURRENT_VALIDATION_CN.md) 留证。同参数比较至少三轮，检查首圈额外用时是否减少，同时确认后续四圈和完赛率没有恶化。代码回归不能代替这一实际验收。本轮不声称已保证启动成功、消除全部首圈额外用时或五圈小于 60 秒。
