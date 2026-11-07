# MotionLaserSync V19

当前版本说明见 **[README_V19.md](README_V19.md)**。

本版本包含 V18 同步报告修复、用户实机 BASIC 归档和上位机软实时 I/O 调度器。
`README_V18_P0.md`、`P0_BASELINE_MANIFEST.json` 和旧 P0 文档仅保留作历史记录，不代表 V19 的变更或验收结果。
校验 V19 请用 `python tools/verify_v19.py`，不要使用旧的 `verify_v18_p0.py` 校验新版本。

---

## V20 单球 14 维联合标定扩展

本副本增加了面向 19.05 mm 单球和 LLT2950-50/BL 的 14 维可观测联合标定模块、
D-optimal 姿态规划器和离线 CLI。详见：

- `README_V20_SingleSphere14.md`
- `docs/V20_SingleSphere14_Calibration.md`

注意：在没有三个独立平移约束时，代码不会把结果错误地表述为可唯一分解的
`11 PIGE + 6DoF hand-eye` 物理 17 参数。


## V20 GUI integrated single-sphere calibration

The V19 `CalibrationWidget` now contains a `单球五轴14维标定` tab. It reuses
the original profile-preview path and the original five-axis joint-motion path,
adds fresh static profile capture, sphere fitting, D-optimal pose planning,
observability checks, solve/save/load, and session CSV import/export.

See `docs/V20_GUI_Integrated_Workflow.md`.

### V20.1 实机采集加固

单球 GUI 标定现在固定使用 `SoftwareInternal` 软件/内部触发，并在启动标定预览时临时恢复
2950 的 `Large / ROI1_PRESET=0` 全量程。界面 X/Z ROI 仅作为 RANSAC 球弧候选搜索窗口，
不会修改硬件测量量程；只有 RANSAC 圆弧内点会进入 14 维联合标定器。
