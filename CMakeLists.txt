# CMake configuration for STM32 projects
cmake_minimum_required(VERSION 3.12)

# Path to the toolchain file
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/CMake/gcc-arm-none-eabi.cmake)

# Project name
project(MyApp)

#generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set MCU configuration
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F446xx)

# Executable name
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

# Enable languages
enable_language(C CXX ASM)

# Set standards for C and C++
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add directories for includes
set(INCLUDE_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/src/application
    ${CMAKE_SOURCE_DIR}/src/application/tasks
    ${CMAKE_SOURCE_DIR}/src/application/algorithms
    ${CMAKE_SOURCE_DIR}/src/mcu
    ${CMAKE_SOURCE_DIR}/src/hal/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/src/hal/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/src/hal/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/src/hal/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/src/config
    ${CMAKE_SOURCE_DIR}/src/application/lib
    ${CMAKE_SOURCE_DIR}/src/external/freeRTOS/org/Source/include
    ${CMAKE_SOURCE_DIR}/src/external/freeRTOS/org/Source/portable/GCC/ARM_CM4F
    
)

# Add source files
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/src/**/*.c
    ${CMAKE_SOURCE_DIR}/src/**/*.cpp
    ${CMAKE_SOURCE_DIR}/src/**/*.s
    ${CMAKE_SOURCE_DIR}/src/main.c
)

# Create the executable
add_executable(${EXECUTABLE} ${SOURCES})

# Compiler definitions
target_compile_definitions(${EXECUTABLE} PRIVATE
    ${MCU_MODEL}
    USE_HAL_DRIVER
)

# Include directories
target_include_directories(${EXECUTABLE} PRIVATE ${INCLUDE_DIRECTORIES})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    -O0  # Change to -O2 or -Os for release builds
    -g3
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${CMAKE_SOURCE_DIR}/src/config/STM32F446RETx_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${EXECUTABLE}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -u _printf_float
)

# Post-build steps for generating .hex and .bin files
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
