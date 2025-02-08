cmake_minimum_required(VERSION 3.22)

project(OpenSTM_HAL)

add_library(
  ${PROJECT_NAME} STATIC
  "App.cpp" "Vendor/STMicro/F3/IO/DigitalOut.cpp"
  "Vendor/STMicro/F3/IO/DigitalIn.cpp" "Vendor/STMicro/F3/Serial/USART.cpp"
  "Vendor/STMicro/F3/Timer/SystemTimer.cpp")

target_link_libraries(${PROJECT_NAME} stm32cubemx OpenSTM_LIB)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
