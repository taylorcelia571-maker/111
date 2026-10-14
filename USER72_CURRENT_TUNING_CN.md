# 当前72秒基线：起步、首弯与提速

本页以用户2026-10-14最新提供的有效五圈72秒配置为基线：横向预算10.5、巡航加速度7.0、制动3.6、yaw3.8、起步速度上限2.0、起步加速度2.5、转向反馈0.25、转向预瞄0.12、heading/cross gain各0.8、delayed_braking、predict_pose=false、路线等级2、提前释放启动限速。不要用旧文档中的9.9/6.6/legacy组替换这套最快基线。

## 本次代码改动

新增require_prepared_start，默认false，下面的验证命令显式打开。开启后，没有先做零速度prepare并取得新鲜、就绪的实测转向，start会被拒绝；车辆不会边起步边等待转向对准。prepare不启动比赛，不发正向速度，仍需用户在官方裁判中准备发车并手动调用一次start。若原先已经使用prepare并确认steering_ready，此开关只防止漏步骤，不能声称它一定缩短成绩。

新增startup_speed_cap_released_seconds与一次性终端提示。原有alignment_released_seconds只表示departure对齐阶段结束，连接线起步时它可能很早出现，却仍处于launch_limited；不能把它当作启动速度限制解除时间。本次没有降低释放门槛：至少0.6m、跟踪误差<2.5cm、航向误差<0.05rad、目标与实测转向误差<0.05rad并持续0.3s。

保留0.34m路线净空、0.23m墙距停车、0.15m参考偏差停车、response_delay=0.25、激光与反馈新鲜度保护。车辆、官方地图与裁判没有修改。默认参数不变，旧命令仍可回退。

## 更新

先stop、结束失败轮、Ctrl+C关闭旧launch。下面的更新遇到错误会中止，避免拉取失败却继续编译旧代码，也适用于之前的detached HEAD：

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

Git若报本地改动冲突或分叉，不要reset --hard。新终端均加载ROS与工作空间；包路径应指向~/ican_race_ws/src/tianracer/tianracer_gazebo。

## A：保留全部72秒速度参数，先验证静止准备

终端A：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
mkdir -p ~/ican_race_logs
roslaunch tianracer_gazebo race_tuning_demo.launch \
  startup_release_early:=true line_search_level:=2 \
  lateral_control:=race_feedback lateral_accel:=10.5 \
  acceleration:=7.0 braking:=3.6 yaw_rate:=3.8 \
  startup_speed:=2.0 startup_acceleration:=2.5 \
  steering_feedback_gain:=0.25 steering_preview_time:=0.12 \
  heading_feedback_gain:=0.8 cross_track_gain:=0.8 \
  speed_planning:=delayed_braking speed_recovery_hold:=0.0 \
  predict_pose:=false require_prepared_start:=true \
  run_report:=$HOME/ican_race_logs/user72_prepare.json
```

此入口已经包含官方场景和裁判，不再运行另一个referee、move_base或其他cmd_vel发布程序。等待规划Ready、车在默认起点静止。终端B：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

prepare必须success: True；status的steering_ready必须true。否则保持静止，可再次查看status，不要先start再等待转向。随后官方裁判检测环境、绑定、准备发车，再调用一次：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

start必须success: True。新增开关不会自动start；未准备的start被拒绝是预期行为。运行终端会显示“Startup speed cap released after ...”，它表示控制器启动限速解除，仍不是官方起步/圈时。普通启动限速、planned_envelope和response_preview不是故障；意外停顿、风险限速、大幅超目标速度、保护停车、翻车或碰撞则立即stop。

## B：首弯同预算对照，单独打开定位补偿

A有效完赛、起步转向流程正常后，复制A完整命令，仅把predict_pose改为true，报告名改为user72_predict.json。其他速度预算、startup与steering_preview_time保持不变。

补偿只对0～80ms且与里程计时间戳匹配的TF年龄有效。运动中status的feedback.prediction_applied才说明实际应用了补偿；参数true并不保证每个周期都生效。它用于检验“滞后位置导致晚纠偏”的假设，尚未确认是本轮现场根因。变慢或卡顿就改回false，不强行保留。

## C：首弯顺畅之后，小步提高整体弯道速度

从当前最快有效配置出发，每轮只改一项：

| 轮次 | 唯一控制变量变化 | 报告名 |
|---|---|---|
| C1 | lateral_accel=10.5→10.8 | user72_lat108.json |
| C2 | C1有效且更快后，10.8→11.0 | user72_lat110.json |

完整启动命令就是A命令，predict_pose采用A/B中已验证更快的值，其余保持相同；只替换表中参数和报告名，不追加重复同名参数。保持require_prepared_start=true，并每轮重新prepare。

巡航加速度7.0、起步速度2.0、起步加速度2.5、yaw3.8、预瞄0.12、制动3.6本轮不再提高。max_speed=10不作为提速手段。横向预算升高并不保证更快：yaw、制动包络、纠偏或反馈可能仍在限速，C1无收益就不继续追高。

不要求连续三次：单次官方有效5/5、无翻车/碰撞/保护停车、last_fault=null且严格快于当前最佳，才记录新最佳；72秒在被更快有效成绩替代前一直保留。没有官方成绩，不把模型圈数当成有效完赛。

## 每轮重置、失败与回退

立即停止：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

遇到风险限速、明显超目标、卡顿、保护停车、13cm偏差预警、翻车/碰撞时，不连续start。结束当前轮，关闭旧launch，重新生成默认静止起点。失败或更慢就回退当前最佳。

回退到用户原始72秒配置：A完整命令中的lateral_accel恢复10.5、predict_pose=false、require_prepared_start=false，报告名改为user72_rollback.json；其余参数原样保留。其他各阶段若已经有更快官方有效结果，则回退那个结果，而不是固定回到72秒。不要在移动中rosparam set这些初始化参数；需重启控制器才生效。

每轮报告自动保存到命令指定的~/ican_race_logs/user72_*.json；官方锁定后stop保存最终报告。无需另外寻找或上传bag。控制报告不是官方成绩文件，official_result=false不能自行改成true。

## 验证证据与限制

66项开发检查通过，包括未准备拒绝发车、实测转向未就绪拒绝发车、prepare零速度、stop后必须重新准备、保护检查、启动限速解除指标及launch转发。没有接入用户ROS会话，未完成官方Gazebo五圈实测。

本次对照使用当前新基线，而不是沿用旧预算的模型。假设执行器响应150ms、位姿延迟20ms，补偿关闭/打开模型五圈73.32/71.30s；延迟40ms为78.14/70.96s。但80ms执行器响应、20ms位姿延迟时，69.48→72.78s反而变慢。不能据此测出真实执行器延迟或宣布补偿必然更快。预瞄提高到0.14、yaw提高到3.9与降低启动加速度均没有普遍收益，不作为推荐档位。另试验的转向预瞄补偿在慢响应模型中接近偏差保护，已撤销，未上传。

模型不包含完整轮胎、翻车、AMCL、雷达和官方裁判，全部标记official_result=false，见USER72_CURRENT_MODEL_RESULTS.json。当前最快官方有效结果仍按用户报告记为72秒。最终目标必须由官方有效5/5且original time<60000000微秒证明，本版没有证明一分钟目标。
