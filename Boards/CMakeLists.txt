cmake_minimum_required(VERSION 3.22)

project(OpenSTM_BSP)


if("${BOARD_NAME}" STREQUAL "F303RE")
    add_library(${PROJECT_NAME} STATIC "Nucleo64-F303RE/App.cpp")
elseif("${BOARD_NAME}" STREQUAL "F072DISCO")
    add_library(${PROJECT_NAME} STATIC "Nucleo-F072-Discovery/App.cpp")
endif()

target_link_libraries(${PROJECT_NAME} OpenSTM_HAL)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
