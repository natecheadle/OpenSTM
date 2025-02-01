cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

project(OpenSTM)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

add_subdirectory("Lib")

if(EMBEDDED)
  add_compile_definitions(EMBEDDED)

  add_subdirectory("HAL")
  add_subdirectory("Boards")
  add_subdirectory(${PROCESSOR_VENDOR})

  add_executable(${PROJECT_NAME} "main.cpp")
  target_link_libraries(${PROJECT_NAME} OpenSTM_BSP)

elseif(DESKTOP)
  add_compile_definitions(DESKTOP)
endif()
