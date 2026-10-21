# 捕获审计证据包

总报告：[CAPTURE_PHYSICS_AUDIT_CN.md](../CAPTURE_PHYSICS_AUDIT_CN.md)。源码实现提交 `cda30f2`；完整环境与源码哈希见 manifest.json。

- `data/capture_traces.csv.gz`：71 个工况、86,600 行、原 500 Hz；关键字段的无损列投影，不是降采样。含全部 54 个随机、7 个专项和 10 个确定性重复工况，失败未删除。
- `data/events.json.gz`：全部 528 个事件的真实时刻、前后原生状态与模式转换。
- `trial_design.json`、`data/catalog.json`：预先分类、所有随机输入、种子，以及完整原始 CSV 的本机路径和 SHA256。完整所有列 CSV 保存在新的 results/capture_audit_20261021/ 下。
- `summary.json`、`figures/trial_statistics.csv`、`locked_geometry.csv`：统计明确区分捕获成功、误锁、未接触、约束误差与 site 配合间隙；不可观察值为 null。
- `python_test_status.json`、`cpp_test_status.json`、`p0_p1_summary.json`：逐项验证状态；没有编译产物或大型临时构建日志。
- `data/*_replay.csv.gz`：实际 qpos 回放输入；videos/ 是真实记录回放，before_after 左基线右修复，高速失败为 4 倍慢放。
- `cad_audit.json`：只读原生参数审计；不提供 BREP 有效性、体积或真实质量认证。
- `verify_records.py`：从压缩 CSV 与 JSON 独立重建模式、审核严格驻留和接合条件，检查事件状态连续性。运行 `python3 reports/capture_audit_20261021/verify_records.py`。

机械能定义、有限预充能和接触积分的适用范围见总报告。压缩数据是本轮要求的实验交付，不包含 CAD 二进制、CMake 缓存、动态库或训练模型。
