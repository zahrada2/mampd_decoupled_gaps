cmake_minimum_required(VERSION 2.8.3)
project(cvrp)

#####################
## Compile options ##
#####################
add_compile_options(-std=c++14)
add_compile_options(-Wall -ggdb3 -g)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##################
## Optimization ##
##################
## Good and precise optimization.
add_compile_options(-O3)
## Super fast optimization, but can cause inaccurate results in computing with floats.
# add_compile_options(-Ofast)

## System dependencies are found with CMake's conventions.
find_package(Threads)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.58 REQUIRED COMPONENTS program_options system filesystem log_setup log)

include_directories(
        libs
        src
        ${Boost_INCLUDE_DIRS}
)

add_subdirectory(libs)
add_subdirectory(src)

add_executable(${PROJECT_NAME} main.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE
        -DDEFAULT_DIR_IN="${PROJECT_SOURCE_DIR}/data/mapf_vrp"
        -DDEFAULT_MAPF_DIR_IN="${PROJECT_SOURCE_DIR}/data/mapf_vrp"
        -DDEFAULT_RANDOM_INT_FILE="${PROJECT_SOURCE_DIR}/data/random-integers-2016-09-11.txt"
        )

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_LIB
        ${BOOST_LIBRARIES}
        vrplib
        )

