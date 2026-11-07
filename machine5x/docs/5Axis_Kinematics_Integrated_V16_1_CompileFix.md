# V16.1 五轴运动学集成版 - VS2022 编译兼容修正

本版本仅修正 V16 集成版在 Windows/VS2022/Qt 工程中的编译兼容问题，不改变五轴运动学模型、B/C 零点、方向或现有运动控制逻辑。

## 1. 根因

V16 的 `MachineKinematics5Axis.h` 使用了 `std::max(...)`（包括 initializer_list 形式）。在 Windows/部分第三方头文件环境中，`max` 可能已被定义为宏，导致 MSVC 出现 `C4002: 类函数宏调用 max 参数过多`，并进一步造成 `m_cfg`、`matrixFinite`、`rotationIsOrthonormal` 等大量级联解析错误。

错误列表里 Qt 头文件的 E2422/E26495 等也可能被 IntelliSense 级联放大；真正需要先消除的是项目头文件的宏/语法解析错误。

## 2. 修正内容

- `MachineKinematics5Axis.h` 主动清除 `min/max` 宏影响。
- 不再使用任何 `std::max(...)`，改为内部 `updateMaximum()`。
- `Config` 改为显式 C++11 构造函数，避免嵌套结构默认成员初始化在不同 VS/Qt 配置中的 IntelliSense 兼容问题。
- `MachineKinematics5Axis()` 改为显式构造 `m_cfg`。
- `QString::arg()` 对限位值使用完整格式参数，减少旧 Qt/MSVC 重载解析歧义。
- MainWindow 的 FK→IK 残差计算取消 `std::max({ ... })`。
- 原三轴路径表 `PathPoint` 创建改为逐字段赋值，避免增加 C/B 字段后旧语言标准下的 aggregate 初始化兼容问题。

## 3. 运动学未改变

仍为：

`T_ME = Trans(X,Y,Z) * Rz(-C) * Ry(-B)`

B/C 最新机械零位仍为：

- C: `ZERO_COUNTS_C = 8163418`
- B: `ZERO_COUNTS_B = 9200405`

## 4. VS2022 建议操作

替换文件后建议：

1. 关闭正在运行的旧程序。
2. VS2022 中执行 CMake “删除缓存并重新配置”（或删除 `out/build/x64-Release` / `x64-Debug` 对应构建目录）。
3. 重新生成 x64-Release。
4. 先看“生成输出”里的第一条真实编译错误，不要优先处理 C26495 这类 Qt 静态分析警告。

如果仍失败，请提供“生成”窗口中第一条 C/C++ compiler error，而不是仅提供 IntelliSense 错误列表。
