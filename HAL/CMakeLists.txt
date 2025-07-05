cmake_minimum_required(VERSION 3.22)

project(OpenSTM_HAL)

if(EMBEDDED)
  set(SOURCES
      "Vendor/${PROCESSOR_VENDOR}/${PROCESSOR_SERIES}/IO/DigitalOut.cpp"
      "Vendor/${PROCESSOR_VENDOR}/${PROCESSOR_SERIES}/IO/DigitalIn.cpp" "Vendor/${PROCESSOR_VENDOR}/${PROCESSOR_SERIES}/Serial/USART.cpp"
      "Vendor/${PROCESSOR_VENDOR}/${PROCESSOR_SERIES}/Timer/SystemTimer.cpp")
elseif(DESKTOP)
  find_package(boost_asio REQUIRED CONFIG)
  set(SOURCES
      "Vendor/Desktop/IO/DigitalOut.cpp" "Vendor/Desktop/IO/DigitalIn.cpp"
      "Vendor/Desktop/Serial/USART.cpp" "Vendor/Desktop/Timer/SystemTimer.cpp")

  add_subdirectory(Test)

endif()

add_library(${PROJECT_NAME} STATIC "App.cpp" "Serial/USART_Base.cpp" ${SOURCES} )

if(EMBEDDED)
target_link_libraries(${PROJECT_NAME} STM32_Drivers OpenSTM_LIB)
elseif(DESKTOP)
target_link_libraries(${PROJECT_NAME} OpenSTM_LIB Boost::asio)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
