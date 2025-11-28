cmake_minimum_required(VERSION 3.10) # 建议添加最低版本要求
project(RobotCtrlLibrary)            # 建议添加项目名称

# 1. 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 查找依赖包 (必须在链接之前查找)
# -----------------------------------------------------------
# 查找 Torch
set(Torch_DIR "/home/ray/software/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# 查找 Iceoryx
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

# 注意: LCM 和 ONNX 这里的变量 (${onnx}) 未定义
# 假设你需要手动查找或定义它们，如果是在系统路径中，通常需要 find_package(LCM)
# find_package(LCM REQUIRED)

# 3. 设置源文件
# -----------------------------------------------------------
# GLOB_RECURSE 用于自动查找 cpp 文件
file(GLOB_RECURSE robot_ctrl_sources
        "*.cpp"
        "FSM/*.cpp"
)

# 4. 定义库目标
# -----------------------------------------------------------
add_library(robot_ctrl_library SHARED
        ${robot_ctrl_sources}
        ../utilities/src/easylogging++.cc
)

# 5. 包含头文件路径 (使用 target_include_directories 代替全局 include_directories)
# -----------------------------------------------------------
# 定义 ONNX 路径变量，避免硬编码
set(ONNX_ROOT "../onnxruntime-linux-x64-1-16-1")

target_include_directories(robot_ctrl_library PRIVATE
        /usr/local/include
        ../utilities/types
        ../utilities/inc
        FSM/
        ${ONNX_ROOT}/include
        ${TORCH_INCLUDE_DIRS} # 确保包含 Torch 头文件
)

# 6. 链接库文件
# -----------------------------------------------------------
# 注意: link_directories 通常不推荐使用，建议使用 find_library 或 absolute path
link_directories(/usr/local/lib)

target_link_libraries(robot_ctrl_library
        PUBLIC
        iceoryx_posh::iceoryx_posh
        ${TORCH_LIBRARIES}

        PRIVATE
        lcm           # 确保系统能找到 liblcm.so
        ${onnx}       # 确保此变量在某处已被定义 (例如: /path/to/libonnxruntime.so)
)