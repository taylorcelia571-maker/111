# quick_start 仍使用旧算法：入口修正与首弯诊断

你最新贴出的命令仍是 quick_start_race.launch。到上一版 420e918 为止，该入口固定使用 smooth_pursuit、legacy 制动包络，启动上限 1.8m/s、启动加速度 2.0m/s²；连接阶段和前 5 米可能保持较低加速预算。因此传入 acceleration=6.6 不代表前 5 米就按 6.6 加速，也没有启用 time_attack 的 race_feedback 和 delayed_braking。这是代码中可直接确认的入口/参数作用问题。

首弯每次明显慢下来的具体在线原因，仍需当前轮的控制记录确定。此前实测日志有 startup_connector、response_preview、steering_slew、outward_error_recovery，不能把它们全部称作同一个故障，更不能仅提高最高速度消除风险限速。

## 本次改动

* quick_start_race 默认改用 race_feedback、delayed_braking、startup_release_early=true。你原有的命令更新后会真正选择新算法，不再需要记住另一个入口名称。
* 保持你传入的加速度、制动及其他预算；未偷偷提高横向预算或降低墙距。新的启动策略在至少行进 0.6m，并持续 0.3 秒满足跟踪、航向及实际转向就绪后，解除启动速度和加速度限制，不强制一直等待前 5m。条件不满足时仍保留启动保护。
* 原来的 quick_start 行为用 legacy_control:=true 回退。旧报告中的 87.546 秒是旧控制方式成绩，不能被当作新方式的实测结果。要复现历史文档里的 quick_start 结果，也需显式加入 legacy_control:=true。
* 启动时新增 ACTIVE CONTROL 日志，显示实际控制器、规划模式、是否提前释放、启动/正常加速度和执行文件路径。借此识别未更新源码、加载错工作空间、旧控制器仍运行等情况。
* 首次发车后的前 6m 增加最多 20Hz、有数量上限的 departure_trace 诊断，写入 run_report；记录实际/指令速度、进度及完整限速状态。它使用控制器已有反馈，不订阅 Gazebo 模型真值，不阻塞控制周期写盘。start 后重置记录，stop 后保留该轮报告。
* 状态里的 startup_release_blockers 明确指出限速为何未释放；acceleration_guard_active 和 active_acceleration 显示是否仍在使用 2.0m/s² 的启动预算。终端每秒提示等待条件，避免只看 acceleration 参数误认为它已完全生效。

0.34m 路线净空、0.23m 靠墙保护、0.15m 偏差保护、雷达、转角变化率及反馈时效检查保留。本次修复没有证明任意预算都能无碰撞完赛或五圈低于一分钟。

## 更新后沿用你的命令

先停止旧控制器和仿真，结束上一轮，回到默认起点。不要把两套控制器同时运行。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

rospack 应指向 ~/ican_race_ws/src/tianracer/tianracer_gazebo。先重新启动你已经跑通的官方场景和裁判，每个新终端加载相同的两个 setup：

```bash
# 终端 A
roslaunch tianracer_gazebo planned_race_demo.launch
# 终端 B
roslaunch tianracer_sim_referee referee.launch
```

终端 C 保持打开，使用你原来的预算，并用新文件名保存这一轮：

```bash
roslaunch tianracer_gazebo quick_start_race.launch steering_feedback_gain:=0.25 acceleration:=6.6 braking:=3.6 refine_line:=true run_report:=$HOME/ican_race_logs/slim_fixed.json
```

等待 Ready。ACTIVE CONTROL 应包含：

```text
controller=race_feedback
speed_planning=delayed_braking
early_start_release=True
startup_acceleration=2.00
normal_acceleration=6.60
```

这些最后两项同时出现是正常的：2.0 用于尚未建立跟踪的启动阶段，6.6 用于启动条件已经满足后的正常加速。如果仍显示 smooth_pursuit 或 legacy，当前运行的入口/工作空间仍不是本次默认新配置，先解决路径和更新问题，不要继续反复修改速度。

终端 D 检查真实生效参数：

```bash
rosparam get /tianracer/continuous_race/lateral_control
rosparam get /tianracer/continuous_race/speed_planning
rosparam get /tianracer/continuous_race/startup_release_early
rosparam get /tianracer/continuous_race/acceleration
```

应分别为 race_feedback、delayed_braking、true、6.6。

## 准备与发车

车辆保持静止：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

prepare=True 只表示发出了零速转向指令；还要等 status 的 steering_ready=True。然后在裁判检测官方环境、有效绑定并“准备发车”，再调用：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

过首弯时观察控制终端。Startup speed guard 日志表示启动条件尚未满足，并列出 blockers；退出该状态后 active_acceleration 应恢复正常预算。如果仍反复降速，先按具体原因判断，不能直接取消保护。

官方有效五圈锁定后：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rostopic echo -n 1 /tianracer/odom/twist/twist
```

## 首弯限速原因的读取

停车或完赛后提取本轮前 6m 记录：

```bash
python3 - <<'PY'
import json, os
path=os.path.expanduser('~/ican_race_logs/slim_fixed.json')
with open(path) as stream:
    report=json.load(stream)
print(json.dumps({'controller':report.get('controller'),
                  'startup':report.get('startup'),
                  'last_fault':report.get('last_fault'),
                  'departure_trace':report.get('departure_trace')},indent=2))
PY
```

| 记录字段/原因 | 含义 |
| --- | --- |
| departure_alignment / start_mode=alignment | 起点连接条件未通过，使用低速对齐；start_mode 提供初始偏差、方向差和连接段曲率以核查原因 |
| minimum_progress_or_connector_not_complete | 还未达到释放所需进度，或旧模式连接段尚未完成 |
| tracking_error / heading_alignment | 当前跟踪或航向尚未满足启动条件 |
| steering_alignment | 目标转向与实际关节角仍有偏差 |
| sustained_readiness_pending | 条件已满足，但仍等待连续 0.3 秒就绪 |
| acceleration_guard_active=true | active_acceleration 仍受启动限制 |
| reason=response_preview | 实测速率与前方制动/速度包络触发提前减速 |
| reason=steering_alignment | 行驶中的转向/航向跟随限速，不等于起步限制 |
| reason=outward_error_recovery | 正在朝参考走廊外偏离，控制器降低速度纠偏 |
| last_fault 非空 | 已保护停车，读取具体报错；不要在故障位置反复 start |

trace 是控制分析证据，不是官方成绩。它能区分“新算法根本未启用”和“启用后仍有首弯跟踪问题”。当前没有新的首弯实测日志，不能声称后一种问题已经被完全消除。

## 回退与验证范围

回退之前已经完成五圈的旧控制方式，必须从全新起点开始：

```bash
roslaunch tianracer_gazebo quick_start_race.launch legacy_control:=true steering_feedback_gain:=0.25 acceleration:=6.6 braking:=3.6 refine_line:=true run_report:=$HOME/ican_race_logs/slim_legacy.json
```

221 项不同的非 ROS 回归通过（全集 218 项，追加 3 项新回归在控制器/适配器集合复验通过）；Python 3.8 语法和 XML 检查通过。机器可读证据见 [FIRST_TURN_VALIDATION_RESULTS.json](FIRST_TURN_VALIDATION_RESULTS.json)。

本次已在真实 ROS Noetic 中使用 roslaunch --dump-params 解析上述现代入口和回退入口，确认实际选择的控制/规划/启动参数及 6.6/3.6 正确传递。它验证启动文件参数，不等于启动 Gazebo 或实际五圈测试。算法和运行适配器回归另外验证：启动未就绪保留限速；就绪后前 5m 可使用请求的加速度；首弯记录写入报告并在下一轮重置；原有停车和传感器保护保留。

新版同预算的历史模型对照见 [TIME_ATTACK_VALIDATION_CN.md](TIME_ATTACK_VALIDATION_CN.md)。官方首圈、首弯最低速度及五圈总用时仍需在 ROS2GO 实测；一分钟目标尚未达到或验证。
