# 启动、首弯与速度验证

用户报告的约72秒仍是当前最佳，新增配置没有官方实测成绩。目标是官方有效5/5、无翻车/碰撞/保护停车，original time严格小于60000000微秒。控制报告中的official_result=false表示它不是官方成绩文件，不能自行改成true。

## 本次改动

race_tuning_demo.launch和race_tuning.launch新增可选predict_pose参数，默认false。原先此入口将已有的短时定位补偿固定关闭。打开后，仅在TF年龄0～80ms且odom与TF时间戳相差不超过20ms时，按公开里程计补偿位姿；不满足条件时使用原始位置。它不提高舵机速度。

新增对补偿前位置的地图墙距和15cm参考偏差检查。补偿后仍经过原来的控制与保护。0.34m路线净空、0.23m靠墙停车、0.15m偏差停车、response_delay=0.25，以及雷达/反馈/TF新鲜度检查均保留。

yaw_rate是转弯角速度预算，不是舵机动作速度。启动限速释放仍需要至少0.6m、跟踪误差<2.5cm、航向误差<0.05rad、转向误差<0.05rad并持续0.3s。增大启动加速度不能让这些条件提前满足。定位延迟导致晚纠偏，是待现场验证的假设，尚未确认是本轮停顿根因。

假设定位延迟40ms、执行器一阶响应150ms的自行车模型：补偿关闭/打开的五圈83.18/74.54s，首圈17.66/15.88s。另检查80ms、250ms执行器响应和60ms定位延迟，详见STARTUP_DELAY_MODEL_RESULTS.json。模型缺少完整轮胎、侧滑、翻车、AMCL、雷达和裁判，不能证明Gazebo成绩。开发检查84项通过；未完成用户机器上的官方Gazebo验证。试验的启动加速度平滑没有模型提速收益，未发布该参数。

72s→60s需要时间减少16.7%、或相同距离下平均速度增加20%；仅改起步不能保证达标。

## 1. 更新，避免detached HEAD拉取失败后编译旧版本

先stop旧控制器、结束失败轮、Ctrl+C关闭旧导航与仿真。更新失败会中止，不继续编译旧版本，不强制覆盖本地改动。

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin
  if git show-ref --verify --quiet refs/heads/main; then
    git switch main
  else
    git switch -c main --track origin/main
  fi
  git pull --ff-only origin main
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
```

若Git报告本地改动冲突或分支分叉，停止更新，不执行reset --hard。每个新终端source ROS和工作空间。rospack find tianracer_gazebo应指向~/ican_race_ws/src/tianracer/tianracer_gazebo。

## 2. 第一轮：恢复72秒预算，只开定位补偿

终端A如下。此入口已经启动官方场景、裁判和控制器，不另外启动referee或其他导航。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
mkdir -p ~/ican_race_logs
roslaunch tianracer_gazebo race_tuning_demo.launch \
  startup_release_early:=true line_search_level:=2 \
  lateral_control:=race_feedback lateral_accel:=9.9 \
  acceleration:=6.6 braking:=3.6 yaw_rate:=3.8 \
  startup_speed:=1.8 startup_acceleration:=2.0 \
  steering_feedback_gain:=0.25 steering_preview_time:=0.10 \
  heading_feedback_gain:=0.8 cross_track_gain:=0.8 \
  speed_planning:=legacy speed_recovery_hold:=0.0 \
  predict_pose:=true \
  run_report:=$HOME/ican_race_logs/delay_A.json
```

等待规划Ready。终端B加载环境，检查参数并在零速度下预置转向：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosparam get /tianracer/continuous_race/predict_pose
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

predict_pose应为true。prepare必须success: True；status中steering_ready必须true，否则保持静止，查看服务原因，不发start。随后在裁判中检测环境、绑定、准备发车，再执行一次：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

start必须success: True。运动时在status的feedback中检查prediction_applied和tf_age_seconds；总是false时不要仅根据参数true认定生效，也不要继续升预算。状态服务可直接查看，不要求你寻找bag。

卡顿、风险限速、大幅超目标速度、控制停车、偏差达到13cm预警、翻车或碰撞，立即stop，不连续start：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

## 3. 一次只改一个变量

每次结束后stop、关闭旧launch、重新启动场景回到默认静止起点，重新准备裁判。以下新数值都未官方实测。保留有效且更快的一次；失败或变慢就回退当前最佳，不要求连续三次。

| 轮次 | 相对上一组保留配置只改这一项 | 条件 |
|---|---|---|
| A | predict_pose=false→true | 其他保持72秒配置，先验证首弯 |
| B | lateral_accel=9.9→10.2 | A有效、更快、首弯顺畅 |
| C | lateral_accel=10.2→10.5 | B有效且更快 |
| D（可选） | steering_preview_time=0.10→0.12 | 有效完赛但仍存在转向跟随迟滞时单独试验，可能增加切弯偏差 |

B、C复用A完整命令，仅替换lateral_accel和报告名为delay_B.json/delay_C.json。D复用当前最佳完整命令，仅替换steering_preview_time和报告名。不要追加重复同名参数，不提高yaw_rate或startup_acceleration，不同时改路线、制动或反馈增益。

B/C无收益时不继续盲目提高；max_speed=10不作为主要提速手段。level2→3追加线路搜索保护起点前方6m，不能指望它消除首弯卡顿。

成功判据：官方有效5/5，无翻车、碰撞、保护停车，last_fault=null，总时间严格小于当前最佳，才保留。最终达标必须官方original time<60000000微秒。没有官方结果时不能宣布60秒达标。

控制报告自动保存到命令指定的~/ican_race_logs/delay_*.json；官方锁定后stop保存最终报告，不需要手动寻找路径。参数在节点初始化时读取，运行中rosparam set不会自动更新这些值，更改后必须重启控制器。

## 4. 回退到用户报告的72秒配置

先stop、关闭旧launch、重新生成静止起点，再执行：

```bash
roslaunch tianracer_gazebo race_tuning_demo.launch \
  startup_release_early:=true line_search_level:=2 \
  lateral_control:=race_feedback lateral_accel:=9.9 \
  acceleration:=6.6 braking:=3.6 yaw_rate:=3.8 \
  startup_speed:=1.8 startup_acceleration:=2.0 \
  steering_feedback_gain:=0.25 steering_preview_time:=0.10 \
  heading_feedback_gain:=0.8 cross_track_gain:=0.8 \
  speed_planning:=legacy speed_recovery_hold:=0.0 \
  predict_pose:=false \
  run_report:=$HOME/ican_race_logs/rollback_72.json
```
