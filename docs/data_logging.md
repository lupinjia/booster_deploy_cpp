# 策略频率数据日志（DataTamer / MCAP）

本文档介绍 `booster_deploy_cpp` 的**策略频率数据日志**功能：开启后，控制器在每个策略步
（policy step，50 Hz）通过 **DataTamer** 库记录一次机器人状态、PD 目标、观测与动作，
保存为 **MCAP** 文件，可用 PlotJuggler 等工具回放分析。

字段布局与 `booster_deploy/docs/data_logging.md`（Python 版）保持一致，方便仿真/实机逐字段对比。

## 1. 使用的库：DataTamer

[DataTamer](https://github.com/PickNikRobotics/data_tamer)（原 facontidavide/data_tamer）
是一个**低开销、线程安全**的 C++ 数据采集库，专为机器人周期循环（periodic loop）设计：

- **零拷贝注册**：`LogChannel::registerValue(name, &variable)` 注册指向稳定内存的指针，
  每次 `takeSnapshot()` 只做一次 memcpy + 异步入队，50 Hz / 数百字段下开销可忽略。
- **异步写入**：`MCAPSink` 内部用并发队列 + 后台写线程落盘，`takeSnapshot()` 不阻塞在磁盘 I/O。
- **跨线程安全**：注册值由别的线程（如 500 Hz DDS 回调）修改时，用
  `LogChannel::writeMutex()` 保护；本项目改为在控制线程内先填样本再快照，天然无竞争。
- **无 ROS 依赖**：`DATA_TAMER_BUILD_ROS=OFF` 可纯 standalone 构建（本项目即如此）。

**版本**：DataTamer v1.0.0（2026-07，PickNikRobotics/data_tamer，commit e761f4f）。

> 注意：网上早期资料中的 `tamer::tamer_manager` / `register_scalar` 等 API 是 pre-1.0
> 旧接口，已废弃；当前 API 为 `DataTamer::ChannelsRegistry` / `LogChannel` /
> `registerValue` / `takeSnapshot`。

## 2. 依赖与链接

### 2.1 依赖

| 依赖 | 用途 | 状态 |
|------|------|------|
| DataTamer | 数据采集 + MCAP 落盘 | vendored：`third_party/data_tamer/` |
| mcap（随 DataTamer 内嵌） | MCAP 文件格式 | vendored：`3rdparty/mcap` |
| **zstd**（系统包） | MCAP 压缩 | `/usr/include/zstd.h`（已装） |
| **lz4**（系统包） | MCAP 压缩 | 见下方构建说明 |

### 2.2 构建 DataTamer（一次性，已执行并安装到 `third_party/data_tamer`）

```bash
git clone https://github.com/PickNikRobotics/data_tamer.git
cd data_tamer/data_tamer_cpp

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DDATA_TAMER_BUILD_ROS=OFF \
      -DDATA_TAMER_BUILD_TESTS=OFF \
      -DDATA_TAMER_BUILD_EXAMPLES=OFF \
      -DDATA_TAMER_BUILD_BENCHMARKS=OFF \
      -DCMAKE_CXX_FLAGS="-Wno-error=conversion -Wno-error=sign-conversion"
cmake --build build --parallel
cmake --install build --prefix <repo>/third_party/data_tamer
```

**构建要点**：

- `DATA_TAMER_BUILD_ROS=OFF`：纯 standalone，不依赖 ament_cmake / ROS2。
- vendored mcap 在 GCC 9.x 下会触发 `-Wconversion` 且被库的 `-Werror` 拦截，
  故放宽 `-Wno-error=conversion -Wno-error=sign-conversion`。
- mcap 需要 **lz4** 开发包。若系统缺 `liblz4-dev`（本机无 sudo）：
  - 从 lz4 官方仓库下载 `lz4.h / lz4frame.h / lz4hc.h` 到临时 include 目录；
  - 用 `ln -s /usr/lib/x86_64-linux-gnu/liblz4.so.1.9.4 <tmp>/liblz4.so` 提供链接库；
  - cmake 时加 `-DCMAKE_CXX_FLAGS="-I<tmp_inc>" -DCMAKE_LIBRARY_PATH=<tmp_lib>`。
  - 若后续 `apt install liblz4-dev`，可去掉这些变通。

安装产物：`third_party/data_tamer/include/`（头文件）、`lib/<arch>/libdata_tamer.a` +
`lib/<arch>/libmcap_lib.a`（架构特定静态库）、`lib/cmake/data_tamer/`（CMake package config）。

### 2.3 项目内链接（CMakeLists.txt）

按 `CMAKE_SYSTEM_PROCESSOR` 自动选择架构（与 onnxruntime 同模式），直接链接静态库：

```cmake
set(DATA_TAMER_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/data_tamer" CACHE PATH "Path to the DataTamer install prefix")
# DATA_TAMER_ARCH: x86_64 / arm64（由 CMAKE_SYSTEM_PROCESSOR 推导）
set(DATA_TAMER_LIBRARY "${DATA_TAMER_DIR}/lib/${DATA_TAMER_ARCH}/libdata_tamer.a")
...
target_link_libraries(booster_deploy_cpp ...
  ${DATA_TAMER_LIBRARY}      # 静态库（含 mcap 引用）
  ${DATA_TAMER_MCAP_LIBRARY} # vendored mcap
  zstd lz4                   # mcap 压缩依赖（系统包）
  ...)
include_directories(...# ${DATA_TAMER_DIR}/include)
```

链接顺序：`libdata_tamer.a` → `libmcap_lib.a` → `zstd`/`lz4`（静态库依赖后置）。
运行时 `libzstd.so.1` / `liblz4.so.1` 由系统提供。

## 3. 启用方法

`params/k1_config.yaml`：

```yaml
# 空字符串 = 关闭（零开销，不创建 logger）
# 非空     = 启用，保存到 <path>.mcap（自动追加 .mcap 后缀，勿写扩展名）
data_log_path: "logs/k1_policy"
```

`main.cpp` 读取该字段传给 `RobotController`，并在 `SIGINT`/`SIGTERM` 时调用
`StopLogging()` 冲刷落盘（对应 Python 版 "Ctrl+C 任意位置都能保存"）。

## 4. 记录内容

采样率 = 策略频率（`dt = 0.02` → 50 Hz）。关节维度为**实机关节序**（`k1_joint_names`，22 关节）。

| 字段 | 形状 | 说明 |
|------|------|------|
| `t` | (N,) | 模拟时间（秒），`step_count * dt`，按名义策略周期递增 |
| `inference_time` | (N,) | 本步控制耗时（墙钟秒，含 obs 计算 + 推理），可评估 50 Hz → 20 ms 预算 |
| `state` | (N,) | 状态机状态（`STATES` 枚举值：`DAMPING=0`、`STAND=1`、`CTRL=2`） |
| `joint_pos` | (N, 22) | 关节位置，实机序（`robot.jpos`） |
| `joint_vel` | (N, 22) | 关节速度（`robot.jvel`） |
| `feedback_torque` | (N, 22) | 执行器力矩反馈（`robot.tau`，SDK `tau_est`） |
| `root_pos_w` | (N, 3) | 机体质心位置（世界系）。**K1 实机无里程计，恒为 0** |
| `root_quat_w` | (N, 4) | 机体四元数 `[w, x, y, z]`，由 IMU rpy 经 `euler_to_quat` 转换 |
| `root_lin_vel_b` | (N, 3) | 机体线速度。**K1 实机无里程计，恒为 0** |
| `root_ang_vel_b` | (N, 3) | 机体角速度（`robot.gyro`） |
| `dof_targets` | (N, 22) | 本步发送给 PD 的位置目标（`robot.jpos_des`），即实际执行动作 |
| `obs` | (N, obs_dim) | 策略输入观测（`last_obs()`，term-major 含历史堆叠；CTRL 前为空） |
| `action` | (N, 22) | 缩放前原始策略输出（`last_action()`）；`dof_targets = default_joint_pos + action * action_scale` |

13 个数组全部记录；`obs` / `action` 在首次 `Calculate()` 前为空（未进入 CTRL 时不产生）。

## 5. 实现原理

### 5.1 代码结构

```
include/data_logger.hpp     DataLogger 类：DataTamer 封装（注册 + 快照 + 停止）
include/robot_controller.hpp  ControlStep() 末尾 LogDataStep() 填充样本并 log()
src/main.cpp                  读取 data_log_path、SIGINT/SIGTERM 处理
params/k1_config.yaml         data_log_path 配置
```

### 5.2 生命周期

```
RobotController 构造    保存 data_log_path
StartControl()          非空时创建 DataLogger(mcap 路径, 22, obs_dim)，注册 12 字段
ControlStep() 每步       LogDataStep()：填 DataLogSample -> data_logger_.log() -> takeSnapshot()
StopLogging() / Ctrl+C    MCAPSink::stopRecording() 冲刷并关闭 MCAP
```

### 5.3 关键设计

- **注册稳定缓冲**：`registerValue` 指向 `DataLogger` 内部 `buf_`（`DataLogSample`），
  不是直接指向 robot 成员——控制线程每步先 `buf_ = sample` 再快照，
  与 500 Hz DDS 回调线程（写 `robot.jpos` 等）天然隔离，无数据竞争。
- **异步落盘**：`takeSnapshot()` 只是 memcpy + 入队，写文件由 DataTamer 后台线程完成，
  不阻塞 50 Hz 控制周期。
- **float → double**：机器人状态为 `std::vector<float>`，样本统一转 `std::vector<double>`
  以满足 DataTamer schema 稳定性（注册后缓冲尺寸不可变，维度在创建时固定）。

## 6. 查看与分析

- **PlotJuggler（推荐）**：DataTamer 官方配套工具，直接打开 `.mcap` 逐曲线查看/导出。
- **DataTamer 解析器**：`third_party/data_tamer/include/data_tamer_parser/` 提供单头文件
  `data_tamer_parser.hpp`（零依赖），可写小工具读取 MCAP 做程序化分析。
- 与 Python 版 npz 日志字段同名，可与 `booster_deploy` 的仿真/实机日志逐字段对比。

## 7. 注意事项

- 重复运行会**覆盖**同一 `.mcap`（不追加），保留多次实验请改 `data_log_path` 或及时拷贝。
- `t` 是模拟时间（名义周期递增），`inference_time` 是墙钟时间，两者不同。
- 硬杀进程（SIGKILL）可能丢尾部缓冲；正常退出 / Ctrl+C / SIGTERM 都会冲刷保存。
- 更换模型后若 `actor_obs_dim` 变化，`obs` 列数随 `obs_dim()` 自动适配。
