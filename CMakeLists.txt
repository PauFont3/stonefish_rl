cmake_minimum_required(VERSION 3.10)
project(LearningTestRL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Stonefish REQUIRED 1.5.0)

include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(LearningTestRL
    src/main.cpp
    src/LearningTestManager.cpp
)

target_link_libraries(LearningTestRL
    Stonefish::Stonefish
)
