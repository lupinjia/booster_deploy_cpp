cmake_minimum_required(VERSION 3.16)
project(booster_deploy_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

# ---------------------------------------------------------------
# booster_robotics_sdk (override with -DBOOSTER_SDK_DIR=...)
# ---------------------------------------------------------------
set(BOOSTER_SDK_DIR "" CACHE PATH "Path to the booster_robotics_sdk root")
if(NOT BOOSTER_SDK_DIR)
  get_filename_component(_booster_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
  set(BOOSTER_SDK_DIR "${_booster_root}/booster_robotics_sdk")
endif()
message(STATUS "Using booster_robotics_sdk at ${BOOSTER_SDK_DIR}")

set(BOOSTER_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT BOOSTER_SDK_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE BOOSTER_SDK_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(BOOSTER_SDK_ARCH MATCHES "^(AMD64|amd64)$")
    set(BOOSTER_SDK_ARCH x86_64)
endif()

set(BOOSTER_SDK_LIBRARY "${BOOSTER_SDK_DIR}/lib/${BOOSTER_SDK_ARCH}/libbooster_robotics_sdk.a")
if(NOT EXISTS "${BOOSTER_SDK_LIBRARY}")
    message(FATAL_ERROR "Booster Robotics SDK library not found: ${BOOSTER_SDK_LIBRARY}")
endif()

# ---------------------------------------------------------------
# onnxruntime (vendored C++ SDK in third_party/onnxruntime)
#   lib/<arch>/ : arch-specific prebuilt libonnxruntime.so (x86_64 or arm64)
#   include/    : arch-independent headers
# ---------------------------------------------------------------
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/onnxruntime")

set(ONNXRUNTIME_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT ONNXRUNTIME_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ONNXRUNTIME_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(ONNXRUNTIME_ARCH MATCHES "^(AMD64|amd64|x86_64)$")
    set(ONNXRUNTIME_ARCH x86_64)
elseif(ONNXRUNTIME_ARCH MATCHES "^(aarch64|arm64)$")
    set(ONNXRUNTIME_ARCH arm64)
else()
    message(FATAL_ERROR "Unsupported onnxruntime architecture: ${ONNXRUNTIME_ARCH}")
endif()
message(STATUS "onnxruntime architecture: ${ONNXRUNTIME_ARCH}")

set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
    message(FATAL_ERROR "onnxruntime library not found: ${ONNXRUNTIME_LIBRARY}")
endif()

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# ---------------------------------------------------------------
# LibTorch (runtime-selectable inference backend; path overridable
# with -DLIBTORCH_DIR=...)
# ---------------------------------------------------------------
set(LIBTORCH_DIR "$ENV{HOME}/libtorch" CACHE PATH "Path to the LibTorch root")
list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_DIR}")
find_package(Torch REQUIRED)

# ---------------------------------------------------------------
# OpenVINO (runtime-selectable inference backend; path overridable
# with -DOPENVINO_DIR=..., default /opt/intel/openvino_2026)
# ---------------------------------------------------------------
set(OPENVINO_DIR "/opt/intel/openvino_2026" CACHE PATH "Path to the OpenVINO root")
list(APPEND CMAKE_PREFIX_PATH "${OPENVINO_DIR}/runtime")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# ---------------------------------------------------------------
# DataTamer (policy-rate MCAP data log; vendored in third_party/data_tamer)
# Arch-specific static libs: lib/<arch>/, selected by CMAKE_SYSTEM_PROCESSOR
# (same scheme as onnxruntime above).
# ---------------------------------------------------------------
set(DATA_TAMER_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/data_tamer" CACHE PATH "Path to the DataTamer install prefix")

set(DATA_TAMER_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT DATA_TAMER_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE DATA_TAMER_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(DATA_TAMER_ARCH MATCHES "^(AMD64|amd64|x86_64)$")
    set(DATA_TAMER_ARCH x86_64)
elseif(DATA_TAMER_ARCH MATCHES "^(aarch64|arm64)$")
    set(DATA_TAMER_ARCH arm64)
else()
    message(FATAL_ERROR "Unsupported DataTamer architecture: ${DATA_TAMER_ARCH}")
endif()

set(DATA_TAMER_LIBRARY "${DATA_TAMER_DIR}/lib/${DATA_TAMER_ARCH}/libdata_tamer.a")
set(DATA_TAMER_MCAP_LIBRARY "${DATA_TAMER_DIR}/lib/${DATA_TAMER_ARCH}/libmcap_lib.a")
if(NOT EXISTS "${DATA_TAMER_LIBRARY}")
    message(FATAL_ERROR "DataTamer library not found: ${DATA_TAMER_LIBRARY} (build it for ${DATA_TAMER_ARCH}; see third_party/data_tamer/lib/${DATA_TAMER_ARCH}/README.md)")
endif()
message(STATUS "DataTamer architecture: ${DATA_TAMER_ARCH}")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${BOOSTER_SDK_DIR}/include
  ${ONNXRUNTIME_DIR}/include/core/session
  ${TORCH_INCLUDE_DIRS}
  ${DATA_TAMER_DIR}/include
)

add_executable(booster_deploy_cpp
  src/main.cpp
)

target_link_libraries(booster_deploy_cpp
  pthread
  ${BOOSTER_SDK_LIBRARY}
  ${ONNXRUNTIME_LIBRARY}
  yaml-cpp
  spdlog::spdlog
  ${TORCH_LIBRARIES}
  openvino::runtime
  ${DATA_TAMER_LIBRARY}
  ${DATA_TAMER_MCAP_LIBRARY}
  zstd
  lz4
  ${CMAKE_DL_LIBS}
  rt
)

# make libonnxruntime.so / libtorch_*.so / libopenvino*.so findable at runtime
set_target_properties(booster_deploy_cpp PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH};${LIBTORCH_DIR}/lib;${OPENVINO_DIR}/runtime/lib/intel64"
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH};${LIBTORCH_DIR}/lib;${OPENVINO_DIR}/runtime/lib/intel64"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

add_executable(virtual_remote tools/virtual_remote.cpp)
target_link_libraries(virtual_remote
  pthread
  ${BOOSTER_SDK_LIBRARY}
  ${CMAKE_DL_LIBS}
  rt
)
