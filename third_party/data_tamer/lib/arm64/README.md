# arm64 (aarch64) DataTamer 静态库

本目录存放 **arm64/aarch64** 架构的 `libdata_tamer.a`。当前为空——请在一台
arm64 Linux 机器（或 arm64 交叉编译环境）上构建后放入本目录。

## 为什么需要单独构建

DataTamer 是纯 C++ 库（无架构特定汇编，源码完全可移植），但预编译的
`libdata_tamer.a` 是 **x86-64 ELF 目标文件**，无法在 arm64 上链接。
因此 arm64 机器必须使用 arm64 构建的库。

## 在 arm64 机器上构建（推荐：直接在目标机/arm64 容器内）

```bash
git clone https://github.com/PickNikRobotics/data_tamer.git
cd data_tamer/data_tamer_cpp

# arm64 需要 lz4 开发包
sudo apt install -y liblz4-dev libzstd-dev

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DDATA_TAMER_BUILD_ROS=OFF \
      -DDATA_TAMER_BUILD_TESTS=OFF \
      -DDATA_TAMER_BUILD_EXAMPLES=OFF \
      -DDATA_TAMER_BUILD_BENCHMARKS=OFF
cmake --build build --parallel
cmake --install build --prefix <repo>/third_party/data_tamer
```

安装后把 `lib/` 下的库文件移动到本目录：

```bash
# 在 <repo>/third_party/data_tamer 下执行
mkdir -p lib/arm64
mv lib/libdata_tamer.a lib/libmcap_lib.a lib/arm64/
# 验证架构
file lib/arm64/libdata_tamer.a
# 期望: ... ELF 64-bit LSB relocatable, ARM aarch64, ...
```

## 交叉编译（可选）

在 x86_64 主机上用 aarch64 工具链交叉编译：

```bash
sudo apt install -y g++-aarch64-linux-gnu liblz4-dev:arm64 libzstd-dev:arm64
cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE=<aarch64 toolchain file> \
      -DDATA_TAMER_BUILD_ROS=OFF \
      -DDATA_TAMER_BUILD_TESTS=OFF \
      -DDATA_TAMER_BUILD_EXAMPLES=OFF \
      -DDATA_TAMER_BUILD_BENCHMARKS=OFF
```

需要额外的 lz4/zstd arm64 交叉版本头文件与库（apt 源需启用
`dpkg --add-architecture arm64`）。

## CMake 自动选择

`CMakeLists.txt` 按 `CMAKE_SYSTEM_PROCESSOR` 自动选择：

- x86_64/amd64 → `lib/x86_64/`
- arm64/aarch64 → `lib/arm64/`（本目录）
