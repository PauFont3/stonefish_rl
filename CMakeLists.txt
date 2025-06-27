cmake_minimum_required(VERSION 3.10)
project(STONEFISH_RL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Stonefish REQUIRED 1.5.0)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZMQ_INCLUDE_DIRS}
)

add_executable(StonefishRLTest 
   src/main.cpp
   src/StonefishRL.cpp
)

# Link with Stonefish library
target_link_libraries(StonefishRLTest
    Stonefish::Stonefish
    ${ZMQ_LIBRARIES}
)
