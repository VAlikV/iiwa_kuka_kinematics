cmake_minimum_required(VERSION 3.5.0)

set(PROJECT Tests)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# find_package(Threads REQUIRED)
# find_library(OpenBLAS_LIB libopenblas NAMES libopenblas openblas HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(Armadillo_LIB libarmadillo NAMES libarmadillo armadillo HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(FORTRUN libgfortran.so.5 NAMES libgfortran5 gfortran gfortran5  gfortran REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_library(FORTRUN libgfortran.so.5 NAMES libgfortran5 gfortran gfortran5 gfortran REQUIRED)
# find_library(KDL liborocos-kdl.so.1.5.1 NAMES liborocos-kdl liborocos kdl HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

add_executable(${PROJECT}
    test.cpp
)

target_link_libraries(${PROJECT} 
    # ${OpenBLAS_LIB}
    # ${Lapack_LIB}
    # ${Armadillo_LIB}
    # ${FORTRUN}
    Eigen3::Eigen
    # ${KDL}
    # Threads::Threads
)
