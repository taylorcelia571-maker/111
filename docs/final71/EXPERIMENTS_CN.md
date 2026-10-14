# 71秒基线最后一轮优化与验证

本分支time_attack_speedup_final基于ad9be5b，main及原71秒命令未覆盖。71秒是用户报告的最快有效成绩，尚无该轮原始官方结果和bag。本次没有完成Gazebo官方五圈，不能宣称60秒或保证不碰撞。新功能可关闭；每轮仅手动start一次。

## 实现与版本

| 版本 | 文件与函数 | 行为与回退 |
| --- | --- | --- |
| V1 | predictive_follower.start/step/sample_tracking、connector_active；continuous_race、cycle_telemetry；final_speedup入口 | 新增连接长度参数，默认2.5m；修正小数终点浮点比较可能卡在连接终点的问题；五次偏移权重端点一阶/二阶连续；完整逐周期诊断。关闭实验连接长度即可恢复原值。 |
| V2 | LoopPlanner.estimate/score、controller_score、planned_race.main | 静止选线时计入理想曲率前馈、转向补偿与指令变化率约束。不是实时控制器重写；原运行保护保持。route_control_scoring=false回退原评分。 |
| V3 | final_speedup.launch/demo及组合配置 | 只合并现场有效的V1、V2改动。V3入口默认原连接长度且新评分关闭，不继承高预算；组合必须显式开启并再验证。当前没有官方验证有效的组合。 |

连接终点比较使用直接比较无展开弧长终点，并加入1nm数值容差；该容差不作用于偏差、墙距、雷达、时效或连续就绪门槛。初期2.4m候选曾在模型中卡住，经回归修复后无明显圈时收益，不设为默认提速项。所谓C2是五次偏移权重的端点连续性；离散参考线整体还需几何和跟踪验证。

controller_score假设定位准确、实测转角等于当前参考角、反馈年龄为零；不能代替轮胎/AMCL/雷达/舵机模型。路线几何仍经过原有净空、曲率及航点顺序检查。额外模型速度包络只用于静止评分，绝不提高运行时上限。

硬边界未改：原版车辆/赛道/裁判/校验、路线净空0.34m、中心墙距停车0.23m、后轴参考偏差停车0.15m、response_delay=0.25s、雷达与必需反馈300ms时效检查。启动释放仍至少0.6m、误差<0.025m、航向<0.05rad、目标和实测转角差<0.05rad，持续0.3s。legacy_control在原quick_start入口仍保留；在新入口中表示强制关闭连接长度实验与新路线评分，仍使用原71秒race_feedback。

## 更新到独立分支

先停止上一轮，结束裁判轮次，Ctrl+C停止旧launch。更新命令遇到失败会中止，不重置本地修改。

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin main time_attack_speedup_final:refs/remotes/origin/time_attack_speedup_final
  if git show-ref --verify --quiet refs/heads/time_attack_speedup_final; then
    git switch time_attack_speedup_final
  else
    git switch -c time_attack_speedup_final --track origin/time_attack_speedup_final
  fi
  git pull --ff-only origin time_attack_speedup_final
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

rospack必须指向~/ican_race_ws/src/tianracer/tianracer_gazebo。不要同时保留move_base、waypoint_laps或其他驾驶发布者。

## 最省操作的实验入口

终端C加载ROS和工作空间，然后每次只运行表中一条。脚本自动启动官方场景、裁判和控制器，并记录bag、C.log、Git提交号、完整命令、参数和状态快照。所有文件位于终端显示的同一目录；最新目录记在~/ican_race_logs/final_last_session.txt。脚本不调用start。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash

# B0：先确认分支仍能复现71秒预算；不改变路线算法。
rosrun tianracer_gazebo run_final_speedup.sh baseline

# V2-R1：优先对照新评分，运动预算全保持。
rosrun tianracer_gazebo run_final_speedup.sh v2

# V1-R1：仅当启动trace显示连接限速占时，试起步速度2.1。
rosrun tianracer_gazebo run_final_speedup.sh v1 startup_speed:=2.1

# V1-R2：可选连接长度实验；模型没有圈时收益，低优先级。
rosrun tianracer_gazebo run_final_speedup.sh v1 startup_connector_length:=2.4

# V2-R2：仅在实测显示过补偿后，从已验证V2单改反馈增益。
rosrun tianracer_gazebo run_final_speedup.sh v2 steering_feedback_gain:=0.22

# V3：仅当上述V1速度2.1和V2均在官方实测中有效后才组合。
rosrun tianracer_gazebo run_final_speedup.sh v3 route_control_scoring:=true startup_speed:=2.1
```

各命令须独立运行，上一轮结束并退出旧launch后才执行下一条。V3示例不是已验证更快配置。run脚本拒绝提高lateral_accel到9.94以上、braking到3.7以上，直至真实数据经分析；其他保护参数不能通过该脚本覆盖。

终端D：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
FINAL_RUN_DIR=$(cat ~/ican_race_logs/final_last_session.txt)
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}' | tee "$FINAL_RUN_DIR/status_prestart.txt"
```

prepare必须success:true，status必须steering_ready:true。随后在官方裁判检测环境、绑定、准备发车，再调用一次：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

官方有效5/5锁定后：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rosservice call /tianracer/continuous_race/status '{}' | tee "$FINAL_RUN_DIR/status_final.txt"
```

截图/保存官方各圈、总时间及原始微秒值。Ctrl+C退出终端C，脚本会向记录器发送SIGINT以结束bag，然后分析：

```bash
rosrun tianracer_gazebo analyze_final_race.py \
  --session "$FINAL_RUN_DIR" --bag "$FINAL_RUN_DIR/control.bag"
```

analysis.json自动放在同一目录，给出首弯速度、最大参考偏差、单周期指令降速、reason时长、故障前5秒及制动/有效转向响应拟合。不自动认证官方结果；碰撞和翻车次数无证据则为null。状态服务不能从bag重建历史，保存的status仍应保留。

## 完整roslaunch命令

下列是控制器独立入口，用于下面四终端流程。只改变version和唯一试验参数；log_dir每轮独立。

```bash
roslaunch tianracer_gazebo final_speedup.launch \
  version:=baseline \
  lateral_accel:=9.94 acceleration:=7.3 braking:=3.7 yaw_rate:=4.4 \
  startup_speed:=2.0 startup_acceleration:=2.5 \
  steering_feedback_gain:=0.25 steering_preview_time:=0.12 \
  heading_feedback_gain:=0.8 cross_track_gain:=0.8 \
  line_search_level:=3 predict_pose:=false speed_recovery_hold:=0.0 \
  startup_connector_length:=2.5 route_control_scoring:=false \
  require_prepared_start:=true log_dir:=$HOME/ican_race_logs/final71/manual_B0
```

V1：上式version:=v1，单独将startup_speed改成2.1，或单独将startup_connector_length改成2.4，报告目录对应更名。

V2：上式version:=v2、route_control_scoring:=true，其余全部保持。

V3：上式version:=v3，只有现场证明有效后才将route_control_scoring:=true及对应V1参数合并。没有V1收益时，V3保持2.0/2.5/2.5的启动设置，仅重复V2。

## 四终端流程（与自动入口二选一）

每个终端均source /opt/ros/noetic/setup.bash与~/ican_race_ws/devel/setup.bash。

A：
```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

B：
```bash
roslaunch tianracer_sim_referee referee.launch
```

C：先创建唯一目录，然后使用上述完整final_speedup.launch命令，把log_dir指向该目录，并保存日志。
```bash
FINAL_RUN_DIR=$HOME/ican_race_logs/final71/$(date +%Y%m%d_%H%M%S)_manual
mkdir -p "$FINAL_RUN_DIR"
printf '%s\n' "$FINAL_RUN_DIR" > ~/ican_race_logs/final_last_session.txt
# 把完整roslaunch命令中的log_dir改为"$FINAL_RUN_DIR"，末尾追加：
# 2>&1 | tee "$FINAL_RUN_DIR/C.log"
```

D：在prepare前记录bag，随后按上文prepare/status/裁判准备/start顺序。
```bash
FINAL_RUN_DIR=$(cat ~/ican_race_logs/final_last_session.txt)
rosbag record --buffsize=256 -O "$FINAL_RUN_DIR/control.bag" \
  /clock /tf /tf_static /tianracer/cmd_vel \
  /tianracer/ackermann_cmd_stamped /tianracer/odom \
  /tianracer/joint_states /tianracer/scan /tianracer/amcl_pose \
  /tianracer/imu /rosout_agg > "$FINAL_RUN_DIR/record.log" 2>&1 &
FINAL_BAG_PID=$!
# 发车和完赛后先stop，再结束录制：
# kill -INT "$FINAL_BAG_PID"
# wait "$FINAL_BAG_PID"
```

没有订阅/gazebo/model_states。原官方里程计适配器的输入链路保持原样，未新增全局状态驾驶决策。

## 参数矩阵与安全范围

| 参数 | 71秒基线 | V1第一轮 | V2第一轮 | V3初始/组合 | 安全范围 | 每轮只动 |
| --- | ---: | ---: | ---: | --- | --- | --- |
| lateral_accel | 9.94 | 9.94 | 9.94 | 9.94 | 本组合无更高预算安全证据 | 否 |
| acceleration | 7.3 | 7.3 | 7.3 | 7.3 | 不先提高；需证明加速瓶颈 | 是 |
| braking | 3.7 | 3.7 | 3.7 | 3.7 | 未有真实制动验证，保持 | 否 |
| yaw_rate | 4.4 | 4.4 | 4.4 | 4.4 | 先保持；不与横向/制动同时提高 | 是 |
| startup_speed | 2.0 | 2.0；单项候选2.1 | 2.0 | 2.0；有效后才合并2.1 | 2.1为模型候选，非认证安全范围 | 是 |
| startup_acceleration | 2.5 | 2.5 | 2.5 | 2.5 | 暂不改，启动释放仍严格 | 是 |
| steering_feedback_gain | 0.25 | 0.25 | 0.25；过补偿才试0.22 | 0.25或现场赢家 | 0.22待实测，不默认更安全 | 是 |
| steering_preview_time | 0.12 | 0.12 | 0.12 | 0.12 | 暂不扩大，先估有效响应 | 是 |
| heading_feedback_gain | 0.8 | 0.8 | 0.8 | 0.8 | 暂不改 | 是 |
| cross_track_gain | 0.8 | 0.8 | 0.8 | 0.8 | 暂不改 | 是 |
| line_search_level | 3 | 3 | 3 | 3 | 不改变拓扑或航点顺序 | 是 |
| predict_pose | false | false | false | false | 没有同步延迟证据，保持 | 是 |
| startup_connector_length | 2.5 | 2.5；单项可选2.4 | 2.5 | 2.5或已验证值 | 代码可接受2.0～3.0，不代表安全认证 | 是 |
| route_control_scoring | false | false | true | 默认false；V2实测有效后开启 | 模型评分开关，不解除任何保护 | 是 |

## 成功、失败与回退

提升记录：官方有效5/5、无碰撞/翻车/保护停车/卡住/紧急制动、last_fault=null，且实际时间<当前最快有效成绩；最大偏差及摆动不劣于同流程基线。建议同组2～3轮有效，至少一次打破71秒；排名成绩仍以最快有效单次为准，不把模型或失败轮算成绩。

亚60秒：官方结果official_result=true且original time<60000000微秒，另满足上述全部安全条件。控制run_report的official_result始终false，不得手改成true。

出现意外风险限速、明显超目标、卡顿、接近13cm偏差、碰撞/翻车/保护停车时，立即：
```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

不连续start。保存现场，结束失败轮，Ctrl+C退出旧launch，重新生成静止起点。更慢也回退。

功能级回退（完整71秒预算，新评分关闭，连接长度恢复）：
```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline legacy_control:=true
```

严格代码回退：先stop和退出旧launch，再切到冻结基线分支，不覆盖本地修改。
```bash
cd ~/ican_race_ws/src/tianracer
if ! git show-ref --verify --quiet refs/heads/baseline_71_20261014; then
  git branch baseline_71_20261014 ad9be5b
fi
git switch baseline_71_20261014
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
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

## 离线证据与限制

见MODEL_RESULTS.json及VALIDATION_CN.md。模型排除轮胎、翻车、AMCL、激光和裁判，只检验假设自行车跟踪。V1连接2.4无明显圈时收益；startup_speed2.1只改善约0～0.02s，不作为大幅提速保证。V2同预算新路线在部分响应模型改善0.48～1.36s，但某些场景最大指令降速或偏差略增；250ms执行器/40ms位姿/40ms关节延迟模型所有版本均失败，不能声称稳健覆盖所有响应条件。

逐周期日志每次入队不等待磁盘。若队列丢样或写入错误，analysis会标记数据不完整；不能依据缺失日志推断无异常。制动拟合仅输出连续实测窗口，不自动批准提高braking；有效转向拟合包含消息传输与时间戳影响，不等于单独测得舵机物理延迟。

未验证项：71秒原始官方结果、同轮bag、真实制动能力、实际反馈与执行器响应、当前云代码的官方Gazebo五圈、零碰撞/零翻车/零紧急制动、V3实测收益及一分钟目标。正式结果由你本机官方裁判验证。
