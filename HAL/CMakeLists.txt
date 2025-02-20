cmake_minimum_required(VERSION 3.22)

project(OpenSTM_HAL)

if(EMBEDDED)
  set(SOURCES
      "Vendor/STMicro/F3/IO/DigitalOut.cpp"
      "Vendor/STMicro/F3/IO/DigitalIn.cpp" "Vendor/STMicro/F3/Serial/USART.cpp"
      "Vendor/STMicro/F3/Timer/SystemTimer.cpp")
elseif(DESKTOP)
 
  find_package(Boost REQUIRED COMPONENTS asio)

  set(SOURCES
      "Vendor/Desktop/IO/DigitalOut.cpp" "Vendor/Desktop/IO/DigitalIn.cpp"
      "Vendor/Desktop/Serial/USART.cpp" "Vendor/Desktop/Timer/SystemTimer.cpp")

  add_subdirectory(Test)

endif()

add_library(${PROJECT_NAME} STATIC "App.cpp" "Serial/USART_Base.cpp" ${SOURCES} )

if(EMBEDDED)
target_link_libraries(${PROJECT_NAME} stm32cubemx OpenSTM_LIB)
elseif(DESKTOP)
target_link_libraries(${PROJECT_NAME} OpenSTM_LIB Boost::asio)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
