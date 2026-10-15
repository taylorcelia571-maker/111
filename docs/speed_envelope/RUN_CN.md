# 70秒后的速度包络优化：逐轮执行

本分支 `time_attack_speed_envelope` 基于 `time_attack_track_memory` 的 aa32de9，不覆盖旧分支。目标继续争取官方有效五圈小于60秒；本次没有该成绩，也没有远程驾驶用户电脑的小车。

本版新增：静止多起点路线搜索及整圈速度包络优化；直线/过渡/弯道模型速度分区报告；从本机已保存的最佳轮继承参数的单变量试验入口。新算法默认关闭，原71/70秒参数默认值不变。

## 0. 固定你真正的70秒配置

若你尚未开启下一轮、最新记录就是有效70秒轮：先stop，在驾驶终端Ctrl+C，等bag正常结束，再保存指针。不要复制整个大bag，也不要删除旧记录。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/stop '{}'
# 回旧驾驶终端 Ctrl+C，等程序退出。随后：
cat ~/ican_race_logs/final_last_session.txt \
  > ~/ican_race_logs/best_race_session.txt
```

若已经有best70_session.txt，程序也兼容；若已录入多轮成绩而没有指针，会挑人工记录有效5/5、无事故且last_fault=null的最短轮。人工记录不等于已核验官方原始微秒结果。不能把指针指向失败轮。

## 1. 更新独立分支

结束旧裁判轮，退出旧仿真/驾驶launch。更新块遇错停止，不覆盖本地改动：

```bash
(
  set -e
  source /opt/ros/noetic/setup.bash
  cd ~/ican_race_ws/src/tianracer
  git fetch origin \
    time_attack_speed_envelope:refs/remotes/origin/time_attack_speed_envelope
  if git show-ref --verify --quiet refs/heads/time_attack_speed_envelope; then
    git switch time_attack_speed_envelope
  else
    git switch -c time_attack_speed_envelope \
      --track origin/time_attack_speed_envelope
  fi
  git pull --ff-only origin time_attack_speed_envelope
  cd ~/ican_race_ws
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

应指向 `~/ican_race_ws/src/tianracer/tianracer_gazebo`。

## 2. 先打印继承结果，不启动车辆

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosrun tianracer_gazebo speed_trial.py --base best --profile baseline
```

默认仅dry-run，不发布控制、不启动ROS程序。检查source_session确实是你的最佳轮，参数保持实际记录的路线强度、预瞄、起步、横向/加速/制动/yaw。程序不会假定你的70秒一定用了强度0.5。

源目录存在正在录制的bag、非null/缺失last_fault、多次start、事故/无效人工记录、参数与报告冲突或路径校验和不匹配，会拒绝使用。源边界必须是0.34m路线净空、0.25s响应预留、0.15m偏差、max_speed10、steering_rate2以及race_feedback/delayed_braking。

若没有保存最佳指针，但确定最新结束的轮就是70秒，可用 `--base latest` 打印，然后按第0步保存指针。不要凭参考圈时间选择最佳。

## 3. 终端 C：第一轮只改算法

```bash
rosrun tianracer_gazebo speed_trial.py \
  --base best --profile algorithm --run
```

只改变 `route_speed_refine=false→true`，所有速度预算继承最佳轮。算法内部从原Level-3路线做两次受约束平滑提案，再做有峰值质量门控的速度模型搜索；不是在线MPC。规划较久时等待Ready，不要在规划期间发车。

该入口已经启动场景、定位、裁判、控制器和录制，不另启A/B，不同时保留旧驾驶程序。它不调用prepare/start。实际命令、完整参数、代码版本、源轮和唯一变化项都保存到本轮目录；`trial_recipe.json`记录继承关系。

如果算法已在源轮启用，algorithm会提示没有变化；要复测现有最佳，用 `--profile baseline --run`。

## 4. 终端 D：准备、发车一次

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
FINAL_RUN_DIR=$(cat ~/ican_race_logs/final_last_session.txt)
rostopic info /tianracer/cmd_vel
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_prestart.txt"
```

仅continuous_race发布速度；prepare/status成功、steering_ready=true、无故障/传感器错误，默认起点静止。若转向未就绪，只重查status。

裁判检测环境、绑定、准备发车后，只执行一次：

```bash
rosservice call /tianracer/continuous_race/start '{}' \
  | tee "$FINAL_RUN_DIR/start_response.txt"
```

返回success: True才算起步成功。不连续start。

## 5. 完赛或失败停止、整理

官方有效5/5锁定即stop；翻车/碰撞/保护停车/卡住/异常急刹、风险限速、持续明显超目标、误差持续逼近15cm时提前stop，不从故障点重启：

```bash
rosservice call /tianracer/continuous_race/stop '{}' \
  | tee "$FINAL_RUN_DIR/stop_response.txt"
rosservice call /tianracer/continuous_race/status '{}' \
  | tee "$FINAL_RUN_DIR/status_final.txt"
```

保存官方各圈/总时间及可用原始结果。回C按Ctrl+C等录制结束，不要先开下一轮。D执行：

```bash
rosrun tianracer_gazebo review_final_runs.py \
  --record-latest --analyze-latest --bundle-latest
```

只有官方有效、无事故/保护/卡住/急刹、last_fault=null、稳定性不变差且总时间更快，才将最新轮设为新最佳：

```bash
cat ~/ican_race_logs/final_last_session.txt \
  > ~/ican_race_logs/best_race_session.txt
```

更慢或失败不更新指针，下一轮仍从当前最佳继承。比赛看单次最快，不强制连续三次；同条件复核有助于区分波动。

## 6. 提高速度预算：仍一次只改一个

| profile | 唯一变化 | 何时执行 |
| --- | --- | --- |
| lateral_101 | lateral_accel升至10.1 | 当前最佳稳定后，第一档弯速预算试验 |
| lateral_102 | lateral_accel升至10.2 | 10.1有效且更快并已设为最佳后 |
| accel_75 | acceleration升至7.5 | 日志证明正常直线加速限制主导时；模型收益很小，低优先级 |

```bash
# 从当前最佳继承其余全部设置，仅提升横向预算。
rosrun tianracer_gazebo speed_trial.py \
  --base best --profile lateral_101 --run
```

若10.1有效且更快，按第5步更新最佳后：

```bash
rosrun tianracer_gazebo speed_trial.py \
  --base best --profile lateral_102 --run
```

程序禁止从9.94直接跳到10.2，也禁止一次增量超过0.2。**10.2只是本轮数值试验上限，不是实测安全范围。** 当前算法未获益时，不更新best，后面的预算试验会自动回到原70秒配置，不保留失败改动。

直线加速度的可选独立对照，不与上面同一轮叠加：

```bash
rosrun tianracer_gazebo speed_trial.py \
  --base best --profile accel_75 --run
```

max_speed仍为10，因为它不约束当前模型；braking不超过3.7，没有真实制动证据不增加。yaw和启动参数保持源最佳值。所有参数在新节点启动时生效，不在行驶中rosparam set后假定已更新。

按用户本次提高速度预算的请求，底层runner新增显式 `budget_trial:=true` 入口，才允许lateral_accel高于原9.94、最高10.2；默认仍限制9.94。speed_trial仅在需要时显式开启并记录。此门槛扩展不代表物理安全认证；墙距、偏差、雷达、新鲜度、response_delay及route_margin未减弱。

## 7. 失败回退

先stop、保存、Ctrl+C结束旧轮，然后复用已保存的最佳参数：

```bash
rosrun tianracer_gazebo speed_trial.py \
  --base best --profile baseline --run
```

这比legacy_control=true更适合回退到70秒的具体配置：它保留那轮实际的路线优化设置。legacy_control=true会连旧路线优化也关闭，不能称为70秒的精确参数回退。

若要严格退出新代码，结束所有旧launch，再切回未覆盖的旧分支：

```bash
(
  set -e
  cd ~/ican_race_ws/src/tianracer
  git switch time_attack_track_memory
  cd ~/ican_race_ws
  source /opt/ros/noetic/setup.bash
  catkin_make
) && source ~/ican_race_ws/devel/setup.bash
```

之后按旧分支原成功命令启动，记录目录使用新目录。切分支不删除已有日志。

官方小于60秒仍只认真实有效5/5、official_result=true、original time<60000000微秒、last_fault=null，以及无事故/保护停车/卡住/紧急制动。模型和人工抄录不自动认证这一条件。
