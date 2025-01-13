cmake_minimum_required(VERSION 3.22)

project(OpenSTM_BSP)

add_library(${PROJECT_NAME} STATIC "Nucleo64-F303RE/App.cpp")

target_link_libraries(${PROJECT_NAME} OpenSTM_HAL)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
