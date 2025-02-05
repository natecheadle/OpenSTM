cmake_minimum_required(VERSION 3.22)
project(OpenSTM_Lib_Test)

include(GoogleTest)

find_package(GTest CONFIG REQUIRED)

add_executable(${PROJECT_NAME} "main.cpp" "EventTests.cpp")

target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest OpenSTM_LIB)
gtest_add_tests(TARGET ${PROJECT_NAME})
