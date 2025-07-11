cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_FULL_LL_DRIVER 
	HSE_VALUE=8000000 
	HSE_STARTUP_TIMEOUT=100 
	LSE_STARTUP_TIMEOUT=5000 
	LSE_VALUE=32768 
	EXTERNAL_CLOCK_VALUE=8000000 
	HSI_VALUE=8000000 
	LSI_VALUE=40000 
	VDD_VALUE=3300 
	PREFETCH_ENABLE=1 
	STM32F303xE
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_LIST_DIR}/../../Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/CMSIS/Device/ST/STM32F3xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_SYS_Src
    ${CMAKE_CURRENT_LIST_DIR}/../../Src/sysmem.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Src/syscalls.c
    CACHE INTERNAL ""
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_LIST_DIR}/../../Src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Src/stm32f3xx_it.c
    ${MX_SYS_Src}
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_LIST_DIR}/../../Src/system_stm32f3xx.c
    ${CMAKE_CURRENT_LIST_DIR}/../../startup_stm32f303xe.s
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_utils.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_exti.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_gpio.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dma.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usart.c
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rcc.c
)

# Drivers Midllewares



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
