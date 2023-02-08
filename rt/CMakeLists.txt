include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../lcm-types/cpp")

add_library(rt src/rt_spi.cpp)

message("rt_spi current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")

target_include_directories(rt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(rt PUBLIC "../lcm-types/cpp")