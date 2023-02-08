cmake_minimum_required(VERSION 3.5)

project(spiTest)

# include_directories("../lcm-types/cpp")       # this worked, but instead added a target_link_libraries to rt
add_subdirectory(rt)
add_subdirectory(motorApp)
