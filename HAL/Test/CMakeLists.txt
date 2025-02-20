cmake_minimum_required(VERSION 3.22)

project(OpenSTM_HAL_Test)

include(GoogleTest)

find_package(GTest CONFIG REQUIRED)

add_executable(
  ${PROJECT_NAME} "main.cpp" "DigitalInTests.cpp" "DigitalOutTests.cpp"
                  "SystemTimerTests.cpp" "USARTTests.cpp")

target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest OpenSTM_HAL)
gtest_add_tests(TARGET ${PROJECT_NAME})
