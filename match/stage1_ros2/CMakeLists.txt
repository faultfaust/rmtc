  cmake_minimum_required(VERSION 3.8)
project(rmtc_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 包含你的算法头文件
include_directories(src)

# 核心算法库
add_library(rmtc_core
    src/world.cpp
    src/planner.cpp
)

# 可执行文件
add_executable(rmtc_bot_node
    src/main.cpp
)

# 链接依赖
target_link_libraries(rmtc_bot_node
    rmtc_core
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    nlohmann_json::nlohmann_json
)

install(TARGETS
    rmtc_bot_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
