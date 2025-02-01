cmake_minimum_required(VERSION 3.22)

project(OpenSTM_LIB)

add_library(${PROJECT_NAME} STATIC "Math/StmMath.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(DESKTOP)
  add_subdirectory("Test")
endif()
