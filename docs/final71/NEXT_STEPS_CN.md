# 本轮结果后的操作顺序

本页取代旧实验矩阵的执行顺序。当前最快仍是用户报告的有效 71 秒，不能把模型圈时、参考圈时或撞墙后的另一轮约 71 秒当成新的精确最好成绩。本次仅新增赛后整理脚本和说明，没有修改控制、规划、launch 默认值及保护阈值。

| 已做实验 | 用户报告总时间 | 处理 |
| --- | ---: | --- |
| B0 | 72.125s | 诊断基线；尚未复现历史 71s |
| V2-R1 新评分 | 72.014s | 比同批 B0 快 0.111s；单次结果不足以确认收益 |
| V1-R1 起步速度 2.1 | 72.654s | 恢复 2.0；暂不继续组合 |
| V1-R2 连接长度 2.4 | 一次撞墙，另一次约 71s | 撞墙轮无效；该连接长度暂不继续使用 |
| V2-R2 反馈 0.22 | 72.472s | 恢复 0.25 |
| V3 组合 | 72.624s | 关闭组合 |

现在不做排列组合。先补一次同条件 B0；如仍有明显波动，最多再补一次，然后根据已有记录选择一个变量，不无限重复基线。

## 1. 停止旧轮，更新独立分支

车辆异常时先执行下面的 stop，再在原驾驶终端 Ctrl+C，等待 bag 正常结束。裁判结束旧轮。关闭旧 Gazebo、裁判和其他驾驶 launch，随后更新。不要同时启动旧的 move_base、waypoint_laps 或第二个速度发布程序。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/stop '{}'
```

控制器未运行时服务不存在是预期情况。更新块遇到错误会停止；若 Git 提示本地改动冲突，不使用 reset --hard 覆盖。

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin main \
    time_attack_speedup_final:refs/remotes/origin/time_attack_speedup_final
  if git show-ref --verify --quiet refs/heads/time_attack_speedup_final; then
    git switch time_attack_speedup_final
  else
    git switch -c time_attack_speedup_final \
      --track origin/time_attack_speedup_final
  fi
  git pull --ff-only origin time_attack_speedup_final
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

最后应输出 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。每个新终端都需要 source ROS 和工作空间。

## 2. 终端 C：只跑基线

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosrun tianracer_gazebo run_final_speedup.sh baseline legacy_control:=true
```

此脚本已经启动场景、定位、裁判和控制器，并开始录制。**不要另开 A/B 再启动一套仿真或裁判。** 等待控制器规划完并显示 Ready。它恢复的是 71 秒参数预算和原评分；当前分支仍含先前的连接终点修复及诊断功能。严格回退旧代码见第 7 步。

固定值：lateral_accel=9.94、acceleration=7.3、braking=3.7、yaw_rate=4.4、startup_speed=2.0、startup_acceleration=2.5、steering_feedback_gain=0.25、steering_preview_time=0.12、heading_feedback_gain=0.8、cross_track_gain=0.8、line_search_level=3、predict_pose=false、speed_recovery_hold=0、startup_connector_length=2.5、route_control_scoring=false。

legacy_control=true 在这个 final_speedup 入口仅强制恢复连接长度和原评分，横向控制仍是 race_feedback。

每轮自动生成唯一目录，记录 bag、C.log、参数、提交号、规划路径、逐周期数据和报告。不覆盖上轮记录。最新目录自动记在 `~/ican_race_logs/final_last_session.txt`。

## 3. 终端 D：准备、检查、发车一次

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
FINAL_RUN_DIR=$(cat ~/ican_race_logs/final_last_session.txt)

rostopic info /tianracer/cmd_vel
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_prestart.txt"
```

cmd_vel 发布者应只有 continuous_race。prepare 必须 success: True；status 必须 success: True，且返回内容含 steering_ready=true，没有 last_fault/sensor_error。若刚下发转向还未就位，可稍后**重新查 status**；不要反复 start。车辆应在官方默认起点静止，不要在失败位置继续。

在裁判界面依次检测环境、绑定、准备发车。随后只执行一次：

```bash
rosservice call /tianracer/continuous_race/start '{}' \
  | tee "$FINAL_RUN_DIR/start_response.txt"
```

必须 success: True。拒绝起步时保留返回信息并检查，不连续发送 start。

## 4. 完赛或失败：先停，再保存

官方有效 5/5 锁定后执行；若碰撞、翻车、保护停车、卡住、明显异常急刹或速度持续大幅超目标，则立即提前执行：

```bash
rosservice call /tianracer/continuous_race/stop '{}' \
  | tee "$FINAL_RUN_DIR/stop_response.txt"
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_final.txt"
```

保存裁判五个单圈和总时间截图，尽可能保留官方原始结果文件。**不要连续 start 挽救失败轮。** 然后回终端 C 按 Ctrl+C，等待脚本退出并结束 bag。先不要启动下一轮，以免“最新目录”指向下一轮。

仍在终端 D：

```bash
rosrun tianracer_gazebo review_final_runs.py \
  --record-latest --analyze-latest --bundle-latest
```

按四项提示录入：

1. 裁判总时间，例如 `1:12.125`；只输入真实界面值。
2. 五个单圈秒数，用空格分开；暂时没有可回车留空。
3. 裁判是否确认有效 5/5：y / n / u（不确定）。
4. 是否发生碰撞、翻车、保护停车、卡住、紧急制动任一项：y / n / u。

分析在赛后运行，读取已有 bag，不连接驾驶话题，也不写 ROS 参数。原始 bag 保留在原目录；`.bag.active` 未正常结束时脚本拒绝继续。程序不会修改官方结果、将 run_report 的 official_result 改成 true，或把人工抄录认证为官方原始成绩。

直接可找的输出：

- `~/ican_race_logs/final71_summary.md`：所有已有自动记录实验的汇总表。
- `~/ican_race_logs/final71_summary.csv`：可用表格软件打开。
- `~/ican_race_logs/final71_summary.json`：首弯、启动、限速原因、周期耗时、关节年龄及缺项。
- 终端打印的 `*_review.zip`：本轮诊断小包，含可用 analysis、规划路径、C 日志、cycles 等；不包含可能很大的原始 bag。

只有显示需要转向/故障话题进一步复核时再使用原始 bag，不要求手动寻找大量零散文件。原始 bag 也没有上传到公开仓库。

已有实验只想先看汇总，可运行：

```bash
rosrun tianracer_gazebo review_final_runs.py
```

旧实验没有人工成绩记录时，总时间列留空；工具不会拿参考圈时间填充。没有本轮文件、bag 或 analysis 时会显示缺项，不能据此确认故障根因。

## 5. 根据数据决定下一步

对齐同一版本、同一官方默认起点、同一 prepare 流程、同样录制方式，再比较：

- 官方首圈与后四圈均值。首圈差异大而后四圈稳定，优先看启动/首弯；五圈都慢，优先看持续过弯和控制限速。
- startup 的 first_measured_motion_seconds、startup_speed_cap_released_seconds，以及 departure_trace 的 startup_release_blockers。需要首圈第一弯进入/最低速度和恢复时间，不把整个首圈损失归因于启动。
- response_preview、steering_slew、outward_error_recovery、acceleration。time_by_state 中 road/phase/lap/reason 有重叠，不能将其时长相加当成总时间；分析器的 final_reason_seconds 单独保留。
- 最大参考偏差、关节反馈年龄、单周期指令降速、command 与 odom 差、last_fault。单周期指令降速不是实测制动能力。
- cycle_wall_timing：控制计算和仿真周期是否发生长尾。录制/存储负载只作为待验证假设，不能直接宣称是变慢原因。

最多补两轮 B0 后仍不能定位时，停下实验，使用自动汇总和诊断小包分析，不再盲调。

## 6. 只有证据支持时才执行下列单变量试验

以下是未实测候选，不是更快结果，也不是同时执行的清单。每一项均从 B0 恢复后单独试；不接着修改上一项。braking、lateral_accel、yaw_rate 暂不增加；不改 max_speed、route_margin、response_delay 和任何保护。

| 条件 | 唯一调整 | 其他预算 | 主要风险 |
| --- | --- | --- | --- |
| 启动 trace 证明速度爬升被 startup_acceleration 限制，且转向/跟踪已就绪 | startup_acceleration 2.5→2.6 | B0 保持 | 连接段过早提速，首弯误差变大 |
| bag 证明持续转向滞后，steering_slew 占时，且不是过补偿或反馈过期 | steering_preview_time 0.12→0.13 | B0 保持 | 过早转向、内侧切弯或出弯纠偏 |
| 正常直线主要受 acceleration 限制，实际速度能跟随，且不是限速恢复 | acceleration 7.3→7.5 | B0 保持 | 出弯提速过早、实际速度超包络 |

分别对应的终端 C 命令：

```bash
# 仅在启动爬升证据成立时执行，不再试已变慢的 startup_speed=2.1。
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=true startup_acceleration:=2.6
```

```bash
# 仅在连续 bag 支持转向滞后时执行。
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=true steering_preview_time:=0.13
```

```bash
# 仅在直线正常加速限制是瓶颈时执行。
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=true acceleration:=7.5
```

每条都完整执行第 3～4 步。有效 5/5、无事故/保护停车/卡住/紧急制动、last_fault=null，才可比较时间；若真实官方时间比精确最佳更短，则保存候选。只比同批 B0 快、但仍慢于 71 秒，不替换最佳。推荐同条件复核以判断波动，不把多次稳定作为比赛强制规则。

若偏差持续逼近 0.15m、出现保护/风险限速、持续明显超目标或车辆异常，立即 stop；这是终止实验的要求，不是修改保护阈值。连接长度 2.4、反馈 0.22、起步速度 2.1 暂时不重试；新评分不与其他变更组合。

## 7. 失败回退

先 stop、保存状态、Ctrl+C 结束旧轮。参数和功能层回退：

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline legacy_control:=true
```

重新生成默认场景，按准备流程开新一轮。需要严格回退旧代码时，先结束所有旧 launch；下面只切分支，不覆盖未提交改动：

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin main
  if ! git show-ref --verify --quiet refs/heads/baseline_71_20261014; then
    git branch baseline_71_20261014 ad9be5bf9b4f23c042fe368d138e9c7b4bd77902
  fi
  git switch baseline_71_20261014
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
```

然后使用原 71 秒完整命令。该入口已经包括场景和裁判，不另启 A/B：

```bash
roslaunch tianracer_gazebo race_tuning_demo.launch \
  startup_release_early:=true line_search_level:=3 \
  lateral_control:=race_feedback lateral_accel:=9.94 \
  acceleration:=7.3 braking:=3.7 yaw_rate:=4.4 \
  startup_speed:=2.0 startup_acceleration:=2.5 \
  steering_feedback_gain:=0.25 steering_preview_time:=0.12 \
  heading_feedback_gain:=0.8 cross_track_gain:=0.8 \
  speed_planning:=delayed_braking speed_recovery_hold:=0.0 \
  predict_pose:=false \
  run_report:=$HOME/ican_race_logs/best_71_rollback.json
```

仍须静止 prepare、status 验证、裁判准备、start 一次。旧分支不包含新汇总脚本；已生成的日志不会因切换代码分支删除。

最终 60 秒验收只认原始官方有效 5/5、official_result=true、original time<60000000 微秒、last_fault=null，以及无碰撞、翻车、保护停车、卡住和紧急制动。当前尚未达到，也不能从本说明的候选参数推断已达到。
