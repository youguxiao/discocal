cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(discocal)
# 强制指定为 Debug 模式
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build" FORCE)

# 打印一下，确保配置生效
message(STATUS "🔥🔥 调试确认：当前构建类型确实是 -> ${CMAKE_BUILD_TYPE}")
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
add_subdirectory(src/lib_calib)
add_subdirectory(src/lib_detect)
# add_subdirectory(src/lib_utils)

add_executable(mono.out src/main_mono.cpp)
target_link_libraries(mono.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    yaml-cpp
)
set_property(TARGET mono.out PROPERTY CXX_STANDARD 20)

add_executable(stereo.out src/main_stereo.cpp)
target_link_libraries(stereo.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    stereoCalibrator
    yaml-cpp
)
set_property(TARGET stereo.out PROPERTY CXX_STANDARD 20)

add_subdirectory(pybind11)
pybind11_add_module(pydiscocal src/main_bind.cpp)
target_link_libraries(pydiscocal PRIVATE
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    stereoCalibrator
    yaml-cpp
)
