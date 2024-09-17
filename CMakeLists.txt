cmake_minimum_required(VERSION 3.5.0)

set(PROJECT Kinematic)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_library(OpenBLAS_LIB libopenblas NAMES libopenblas openblas HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(Armadillo_LIB libarmadillo NAMES libarmadillo armadillo HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

set(KINEMATIC_CLASS ${CMAKE_CURRENT_SOURCE_DIR}/kinematic_class)

add_executable(${PROJECT}
    main.cpp
    ${KINEMATIC_CLASS}/kinematic.hpp    ${KINEMATIC_CLASS}/kinematic.cpp
)

target_link_libraries(${PROJECT} 
    ${OpenBLAS_LIB}
    ${Lapack_LIB}
    ${Armadillo_LIB}
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)