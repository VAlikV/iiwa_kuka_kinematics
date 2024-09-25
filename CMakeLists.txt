cmake_minimum_required(VERSION 3.5.0)

set(PROJECT Kinematic)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_library(OpenBLAS_LIB libopenblas NAMES libopenblas openblas HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(Armadillo_LIB libarmadillo NAMES libarmadillo armadillo HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(FORTRUN libgfortran.so.5 NAMES libgfortran5 gfortran gfortran5  gfortran REQUIRED)
message(${FORTRUN})

set(KINEMATIC_CLASS ${CMAKE_CURRENT_SOURCE_DIR}/kinematic_class)
set(USABLE_FUNCTIONS ${CMAKE_CURRENT_SOURCE_DIR}/usable_functions)

add_executable(${PROJECT}
    main.cpp
    ${KINEMATIC_CLASS}/kinematic.hpp            ${KINEMATIC_CLASS}/kinematic.cpp
    ${USABLE_FUNCTIONS}/usable_functions.hpp    ${USABLE_FUNCTIONS}/usable_functions.cpp
)

target_link_libraries(${PROJECT} 
    ${OpenBLAS_LIB}
    ${Lapack_LIB}
    ${Armadillo_LIB}
    ${FORTRUN}
    Threads::Threads
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(for_tests)