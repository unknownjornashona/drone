cmake_minimum_required(VERSION 3.10)

# 项目名称
project(DronePathOptimizer)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 指定头文件目录
include_directories(include)

# 定义源文件
set(SOURCES
    src/main.cpp
    src/DronePathOptimizer.cpp
    src/DynamicPathPlanner.cpp
    src/HeuristicImprovement.cpp
    src/ParallelComputation.cpp
    src/PathSmoothing.cpp
    src/WeatherModeling.cpp
    src/MultiUAVCoordination.cpp
    src/UserInterface.cpp
    src/MachineLearningIntegration.cpp
    src/SensorIntegration.cpp
    src/SystemIntegration.cpp
)

# 创建可执行文件
add_executable(DronePathOptimizer ${SOURCES})

# 链接 OpenCV
target_link_libraries(DronePathOptimizer ${OpenCV_LIBS})
