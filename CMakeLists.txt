cmake_minimum_required(VERSION 3.16)
project(traffic_monitor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# OpenCV
find_package(OpenCV REQUIRED)

# Include dirs
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Collect sources (필요하면 파일 추가/제거)
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# MVP: TensorRT YOLO skeleton not needed yet -> exclude from build
list(FILTER SRC_FILES EXCLUDE REGEX ".*/src/detect/trt_yolo_detector\\.cpp$")

# If you have tools separately, you can exclude them like this:
# list(FILTER SRC_FILES EXCLUDE REGEX ".*/tools/.*")

add_executable(traffic_monitor ${SRC_FILES})

target_link_libraries(traffic_monitor
  ${OpenCV_LIBS}
  pthread
)

# Optional: warnings
target_compile_options(traffic_monitor PRIVATE
  -Wall -Wextra -Wpedantic
)