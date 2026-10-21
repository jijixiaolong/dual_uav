# 双四旋翼对接 / 分离仿真

项目已从早期 synthetic 原型发展到 MuJoCo 六自由度物理与 C++17 控制核心。当前范围为 P0/P1 验收。

请阅读 [当前说明与一键运行命令](README.md)、[阶段一审计](reports/stage1_experiment_audit.md) 和 [阶段二技术报告](reports/STAGE2_REPORT_CN.md)。

`src/event_surrogate.py` 和 `results/synthetic_trials.csv` 保留为明确标注的简化模型示例；它们不代表真实碰撞或论文实验结果。旧 MuJoCo 学习结果也保留供审计，不能作为修复后性能依据。后续配对统计与 ILC/ECLR 尚待下一阶段完成。
