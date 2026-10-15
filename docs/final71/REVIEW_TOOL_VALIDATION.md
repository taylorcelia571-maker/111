# 赛后汇总工具检查

本次变更仅涉及 `review_final_runs.py`、对应测试和操作说明；不修改控制器、规划器、launch、车辆、赛道、裁判或保护阈值。

- Python 3.12：13 项测试通过。
- ROS Noetic 环境的 Python 3.8.10：同样 13 项测试通过；是同一组测试，不能合计成 26 项不同检查。
- 包含命令行实际生成 Markdown/CSV/JSON/ZIP 的端到端检查。
- 包含 `.bag.active` 阻止赛后处理、缺失 last_fault 不当作 null、碰撞/保护停车覆盖人工有效标记、未知事故状态保留未知、NaN/溢出输入拒绝、已有人工记录不覆盖、原始 bag 保留且不加入小包等检查。
- 人工抄录即使小于 60 秒，仍不认证 `official_result` 或原始微秒用时；参考圈时间也不会填入官方总时间列。

复现命令：

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover \
  -s tianracer_gazebo/test -p test_run_review.py -v
```

没有在本次工作中远程控制用户小车，没有新增 Gazebo 五圈或官方成绩实测。71 秒及本批实验总时间来自用户反馈；无法仅凭这些总时间确定启动、转向、选线或制动根因。
