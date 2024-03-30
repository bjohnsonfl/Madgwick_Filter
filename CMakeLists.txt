cmake_minimum_required(VERSION 3.11)

project(Madgwick_Filter)

# Should be in a subdirectory 
add_library(madgwick SHARED madgwickFilter.c)

target_include_directories(madgwick PUBLIC ${PROJECT_SOURCE_DIR}/)

add_executable( ${PROJECT_NAME} main.c)

target_link_libraries( ${PROJECT_NAME} 
        madgwick
        m)