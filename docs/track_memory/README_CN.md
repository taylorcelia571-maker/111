# 赛道记忆、固定预算路线优化：执行说明

分支：`time_attack_track_memory`。基于 `time_attack_speedup_final` 的 `aaea1a5`，不覆盖原分支、main 或原 71 秒参数。所有新功能默认关闭；关闭后按原路线和控制律运行。

## 先回答“第一圈记路，后四圈省计算”

当前程序已经在发车前完成整圈 A*、平滑、Level-3 搜索和速度包络，并在每圈复用。无需牺牲第一圈录制。后四圈依然必须根据实时定位确认所在弧长，再做转向反馈和雷达/墙距/新鲜度检查。按时间回放第一圈指令无法应对执行器响应、侧滑、AMCL 修正和车辆速度变化，不能用来替代闭环。

新增缓存把固定线段向量、投影分母、循环速度最小值和曲率最大值提前准备好。它从第一圈之前即可使用；不保存车辆位姿或控制指令作为后续圈的真值。查询跨圈连续，参数/参考被更换时拒绝使用旧缓存。起步准备的 deepcopy 路径已测试。

缓存只减少微小计算开销。本次离线局部投影中位时间约从 0.03835ms 到 0.03584ms；这不是完整控制周期或圈时收益。五圈模型输出与不缓存逐项一致。不能把控制计算耗时按周期相加，从比赛成绩里直接扣掉。

## 真正优先验证的改动：受约束路线平滑

`joint_line_polish.py::polish_line` 在静止规划阶段，基于原 Level-3 路线建立周期高斯基函数的法向偏移，用带线性约束的二次目标减少曲率及其变化。它不是在线 MPC，也不改动车辆动力学。

求解器只提出候选。每个候选仍经过原 `LoopPlanner.feasible`：完整路线净空、转向几何、航点距离和顺序全部保留，然后以原完整速度包络评分。没有可行收益、求解失败或超过迭代上限，保留原路线。起点后约 6m、圈缝前约 1m 的输入路线受保护；最终仍需原起步连接段检查。周期重采样不构成严格 C2 数学证明。

保持不变：原车辆/地图/裁判/校验，route_margin=0.34m，墙距停车=0.23m，参考偏差停车=0.15m，response_delay=0.25s，雷达/关节/TF/里程计新鲜度、启动持续就绪、转角及转角变化率、全部风险限速/保护。未新增 `/gazebo/model_states` 订阅。

## 模型结果，绝不是官方成绩

路线、运动预算固定；表中的时间均来自自行车加假设一阶执行器响应，未包含轮胎、翻车、AMCL 跳变、真实雷达和裁判。不能从这些结果推断有效 5/5 或没有紧急制动。

| 执行器/位姿/关节年龄假设(ms) | 原路线模型五圈 | 路线强度0.5 | 路线强度1.0 |
| --- | ---: | ---: | ---: |
| 80/20/0 | 70.28s | 68.12s | 66.42s |
| 150/20/0 | 73.72s | 71.18s | 69.04s |
| 150/40/40 | 77.12s | 73.78s | 71.08s |
| 250/40/40 | 跟踪失败 | 跟踪失败 | 跟踪失败 |
| 150/20/120 | 88.06s | 82.48s | 77.64s |

不能只看模型用时。例如 150/20/0 时强度0.5的最大单周期指令降速为1.377m/s，原路线为0.718m/s；强度1.0为0.630m/s。150/40/40时强度1.0约1.038m/s，原路线约0.877m/s。强度与指令平顺性并非单调关系。这些不是实测制动减速度，但构成必须监控的风险，**不能宣称“不急刹”“稳定性完全不降”**。

强度0.5、1.0实际最大提出法向位移约5.03cm、10.05cm；每条候选均仍受0.34m净空检查。Python与Noetic的这两条选线哈希一致，快速模型用时一致。原始结果见 `ROUTE_MODEL.json`、`NOETIC_ROUTE_MODEL.json`。

转向自适应预瞄也做了试验，但没有普遍收益：0.12s设置在80/20/0模型中由70.28变成73.14s，在150/40/40模型中由77.12变成74.66s。因此默认关闭，**不与路线实验一起启用**。见 `PREVIEW_MODEL.json`。

当前官方最快仍是用户报告的71秒。本次没有官方55秒或60秒结果。

## 1. 更新新分支

先 stop，在旧驾驶终端 Ctrl+C 等待 bag 正常结束，裁判结束旧轮。不要同时保留旧仿真、裁判或速度发布者。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/stop '{}'
```

服务未运行时不存在可以忽略。更新块遇错停止，不覆盖未提交改动：

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin \
    time_attack_track_memory:refs/remotes/origin/time_attack_track_memory
  if git show-ref --verify --quiet refs/heads/time_attack_track_memory; then
    git switch time_attack_track_memory
  else
    git switch -c time_attack_track_memory \
      --track origin/time_attack_track_memory
  fi
  git pull --ff-only origin time_attack_track_memory
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

rospack 应指向 `~/ican_race_ws/src/tianracer/tianracer_gazebo`。

## 2. 终端 C：每轮只选一条

新开关通过 `run_final_speedup.sh` / `final_speedup.launch` / `final_speedup_demo.launch` / `planned_race.launch` 传递；不要直接追加到未新增这些参数的旧 `race_tuning_demo.launch`。

| 新参数 | 默认 | 用途 |
| --- | --- | --- |
| route_joint_polish | false | 开启静止路线优化；推荐先独立验证 |
| route_polish_strength | 0.5 | 优化候选缩放，数值校验范围0.1–1.0，并非认证安全范围 |
| runtime_path_cache | false | 固定几何/速度查询缓存；与控制回放无关 |
| steering_preview_mode | legacy | curvature_adaptive为未完成现场验证的预瞄对照 |
| legacy_control | false | true强制恢复原路线评分/连接长度并关闭全部新功能 |

运动预算保持9.94/7.3/3.7/4.4，起步2.0/2.5，转向反馈0.25、预瞄时间0.12、航向/横向增益0.8/0.8、Level-3、predict_pose=false、speed_recovery_hold=0。新开关不会把这些预算自动提高。

终端先加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

B0：当前分支的71秒预算对照，全部新功能关闭。已有同条件基线时不要求无限重跑。

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline legacy_control:=true
```

R1：优先只改路线。起步速度、转向预瞄方式、反馈增益、运动预算全部保持。先用0.5强度试验；并不保证其指令降速更小，发生异常立即终止。

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=false route_joint_polish:=true route_polish_strength:=0.5
```

**新路线实验必须 legacy_control=false。true 会强制关掉新路线、缓存和自适应预瞄。** 查看规划日志中的 `Joint route polish (model only)`：只有 `accepted=true` 才实际换线；否则按原路线执行，不算新路线的性能结果。

R2：仅当R1官方有效、无风险且改善后，单改强度0.5→0.6。0.6是未完成官方验证的候选，不保证比0.5更快；候选检查可能选择较小幅度。

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=false route_joint_polish:=true route_polish_strength:=0.6
```

之后只有每步有效且不恶化稳定性，才考虑0.7、0.8等单变量试验；没有现场证据时，不直接跳到1.0，不叠加加速度/yaw/横向预算。

缓存独立检查入口（不预期明显圈时收益，不必优先花比赛轮次）：

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=false runtime_path_cache:=true
```

自适应预瞄仅在连续bag支持转向响应问题后单独对照：

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline \
  legacy_control:=false steering_preview_mode:=curvature_adaptive
```

此条不是推荐的首轮，也不搭配route_joint_polish。不要照抄Monza的3–15m前瞻；本算法以本车尺度设置目标距离上限0.60m及曲率限制，并限制目标过渡速度。起步/连接/启动限速期间仍保持原预瞄。

脚本已启动仿真、定位、裁判、控制器和录制，不再另开A/B启动第二套。不会自动prepare或start；每次生成唯一记录目录。

## 3. 终端 D：准备、发车一次

等待C完成规划显示Ready。保持官方默认起点静止：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
FINAL_RUN_DIR=$(cat ~/ican_race_logs/final_last_session.txt)
rostopic info /tianracer/cmd_vel
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_prestart.txt"
```

仅一个continuous_race速度发布者，prepare/status成功，steering_ready=true，无sensor_error/last_fault。等待转向就绪只重查status，不连续start。

裁判检测环境、绑定并准备发车后：

```bash
rosservice call /tianracer/continuous_race/start '{}' \
  | tee "$FINAL_RUN_DIR/start_response.txt"
```

只发一次，必须success: True。

## 4. 完赛或失败保存

官方有效5/5锁定后stop；保护停车、风险限速、卡住、翻车、碰撞、明显异常急刹、持续大幅速度超调或参考偏差持续逼近15cm时提前stop，不继续start：

```bash
rosservice call /tianracer/continuous_race/stop '{}' \
  | tee "$FINAL_RUN_DIR/stop_response.txt"
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_final.txt"
```

截图保存裁判各圈/总时间，保留可用原始官方结果。回C按Ctrl+C，等bag结束。不要先启动下一轮。D执行：

```bash
rosrun tianracer_gazebo review_final_runs.py \
  --record-latest --analyze-latest --bundle-latest
```

录入真实界面总时间、五个单圈（无则留空）、是否有效5/5，以及是否有任一事故/保护/卡住/急刹。人工抄录不认证官方原始微秒数。输出固定在 `~/ican_race_logs/final71_summary.{md,csv,json}`，小包位置自动打印，原始bag留在本轮目录。

主要对比官方总时间/各圈、首圈与后四圈均值、参考最大/p95误差、单周期指令降速、tracking_lateral/steering_alignment/outward_error_recovery/acceleration/response_preview原因、last_fault和实际反馈时效。reason/road/phase/lap有重叠，不相加。最大单周期指令降速不等于实际制动减速度。

同一官方环境、默认起点、prepare流程和录制负载比较。有效且更快、同时无稳定性退化才保留；比本批B0快但未打破历史最佳，不替换71秒最佳。观察到任何风险或更慢，保存后回退。

## 5. 回退

功能与参数回退：结束旧轮后重新启动：

```bash
rosrun tianracer_gazebo run_final_speedup.sh baseline legacy_control:=true
```

回到本次修改之前的代码，先结束所有旧launch：

```bash
(
  set -e
  cd ~/ican_race_ws/src/tianracer
  git switch time_attack_speedup_final
  cd ~/ican_race_ws
  source /opt/ros/noetic/setup.bash
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
```

再运行原71秒命令或原分支的baseline入口。该旧分支在本次发布时仍为aaea1a5，没有被覆盖。严格回到更早71秒代码ad9be5b的步骤见 `../final71/NEXT_STEPS_CN.md` 第7步。

最终60秒只认原始官方有效5/5、official_result=true、original time<60000000微秒、last_fault=null，以及无事故/保护停车/卡住/紧急制动。本次没有满足该标准的新增实测成绩。
