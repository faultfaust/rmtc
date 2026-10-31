cmake_minimum_required(VERSION 3.8)
project(rmtc_stage2_tm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
enable_testing()

# ---- 核心库源码（无 ROS 依赖，离线测试与节点共用同一份）----
set(RMTC_CORE_SRC
  src/agent.cpp
  src/combat.cpp
  src/planner.cpp
  src/world.cpp
  src/rmtc/protocol.cpp
  src/rmtc/json.cpp
  src/rmtc/radar.cpp
  src/rmtc/event.cpp
  src/rmtc/event_prompt.cpp
  src/rmtc/llm.cpp
  src/rmtc/http_transport.cpp
)

# 节点
add_executable(rmtc_bot src/main.cpp ${RMTC_CORE_SRC})
target_include_directories(rmtc_bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(rmtc_bot PRIVATE -Wall -Wextra)
target_link_libraries(rmtc_bot Threads::Threads)
ament_target_dependencies(rmtc_bot rclcpp std_msgs)

install(TARGETS rmtc_bot
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/mock_gateway.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# ---- 离线评测器：4 车 match4（不依赖 ROS）----
add_executable(rmtc_match4 src/match4.cpp ${RMTC_CORE_SRC})
target_include_directories(rmtc_match4 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(rmtc_match4 PRIVATE -Wall -Wextra)
target_link_libraries(rmtc_match4 Threads::Threads)
install(TARGETS rmtc_match4
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 离线回归（默认构建；不需要 ROS 图）----
option(RMTC_BUILD_TESTS "Build offline regression binaries" ON)
if(RMTC_BUILD_TESTS)
  add_executable(v12_check test/v12_check.cpp ${RMTC_CORE_SRC})
  target_include_directories(v12_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(v12_check Threads::Threads)
  add_test(NAME v12_check COMMAND v12_check)

  add_executable(damage_mission_check test/damage_mission_check.cpp ${RMTC_CORE_SRC})
  target_include_directories(damage_mission_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(damage_mission_check Threads::Threads)
  add_test(NAME damage_mission_check COMMAND damage_mission_check)

  add_executable(stage2_probe test/stage2_probe.cpp ${RMTC_CORE_SRC})
  target_include_directories(stage2_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stage2_probe Threads::Threads)
  add_test(NAME stage2_probe COMMAND stage2_probe)

  add_executable(stage2_sim test/stage2_sim.cpp ${RMTC_CORE_SRC})
  target_include_directories(stage2_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stage2_sim Threads::Threads)
  add_test(NAME stage2_sim COMMAND stage2_sim)

  add_executable(stage2_llm_test test/stage2_llm_test.cpp ${RMTC_CORE_SRC})
  target_include_directories(stage2_llm_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stage2_llm_test Threads::Threads)
  add_test(NAME stage2_llm_test COMMAND stage2_llm_test)

  add_executable(stage2_llm_e2e test/stage2_llm_e2e.cpp ${RMTC_CORE_SRC})
  target_include_directories(stage2_llm_e2e PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stage2_llm_e2e Threads::Threads)
  add_test(NAME stage2_llm_e2e COMMAND stage2_llm_e2e)

  add_executable(stage2_ai_qualify test/stage2_ai_qualify_test.cpp ${RMTC_CORE_SRC})
  target_include_directories(stage2_ai_qualify PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stage2_ai_qualify Threads::Threads)
  add_test(NAME stage2_ai_qualify COMMAND stage2_ai_qualify)

  # 问题(1)(2) 回归：LLM 全失败时 reach 仍要完成；任务公告坐标兜底；
  # /referee/event_text 注入；事件默认超时 3000ms。
  add_executable(reach_event_check test/reach_event_check.cpp ${RMTC_CORE_SRC})
  target_include_directories(reach_event_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(reach_event_check Threads::Threads)
  add_test(NAME reach_event_check COMMAND reach_event_check)

  # DNS 剩余风险回归：坏域名不能把请求拖到总截止之外。
  add_executable(dns_check test/dns_check.cpp src/rmtc/http_transport.cpp)
  target_include_directories(dns_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(dns_check Threads::Threads)
  add_test(NAME dns_check COMMAND dns_check)
  add_executable(stage2_gateway_check test/stage2_gateway_check.cpp
    src/rmtc/http_transport.cpp
    src/rmtc/json.cpp
    src/rmtc/event.cpp
    src/rmtc/event_prompt.cpp
  )
  target_include_directories(stage2_gateway_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(stage2_gateway_check Threads::Threads)
  add_test(NAME stage2_gateway_check_help COMMAND stage2_gateway_check --help)
endif()

ament_package()
