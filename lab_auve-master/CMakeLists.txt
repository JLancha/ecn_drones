cmake_minimum_required(VERSION 3.8)
project(lab_auve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(PythonLibs REQUIRED)

add_executable(uavcontrol src/uavcontrol.cpp src/uavsim.cpp include/lab_auve/uavsim.h include/matplotlibcpp.h)
target_include_directories(uavcontrol PRIVATE include ${PYTHON_INCLUDE_DIRS})
target_link_libraries(uavcontrol Eigen3::Eigen ignition-transport11::core ${PYTHON_LIBRARIES})

install(DIRECTORY
  launch gazebo
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
