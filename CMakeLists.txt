cmake_minimum_required(VERSION 3.10)
project(STONEFISH_RL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Stonefish REQUIRED 1.5.0)


include_directories(
    #${CMAKE_SOURCE_DIR}/../stonefish/Library/include
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(StonefishRLTest 
   src/main.cpp
   src/StonefishRL.cpp
)

file(COPY ${CMAKE_SOURCE_DIR}/scenes/minimal_scene.xml DESTINATION ${CMAKE_BINARY_DIR}/scenes)
file(COPY ${CMAKE_SOURCE_DIR}/scenes/minimal_learning.xml DESTINATION ${CMAKE_BINARY_DIR}/scenes)

# Link with Stonefish library
target_link_libraries(StonefishRLTest
    Stonefish::Stonefish
)
