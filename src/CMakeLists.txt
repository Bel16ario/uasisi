add_executable(test_orchestrator
    core/orchestrator.cpp
    io/config.cpp
    testmain.cpp
)

target_link_libraries(test_orchestrator PRIVATE yaml-cpp)
