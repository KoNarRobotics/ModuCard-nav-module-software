cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ModuCard_nav_module)



# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})




# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})




set(STMEPIC_FDCAN ON)
# set(STMEPIC_GENERATE_DOCS ON)

add_subdirectory(src/StmEpic)

message(INFO "Linker flags: " ${CMAKE_C_LINK_FLAGS})


# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)


add_subdirectory(src/can_constants)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
  # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  # Add user sources here
  src/main_prog.cpp
  src/main_callbacks.cpp
  # src/quadspi/Dev_Inf.c
  # src/quadspi/Loader_Src.c
  # src/quadspi/W25Q128.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  # Add user defined include paths
  Core/Inc
  src
  src/quadspi
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
  stm32cubemx
  stmepic
  can_constants
)


############################################################################################################
# Configure nice task to make life easy  [4/4]
 
find_program(CCACHE_PROGRAM ccache)
if(NOT CCACHE_PROGRAM)
    message(STATUS "ccache not found")
else()
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()


add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf firmware.hex
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_PROJECT_NAME}.elf firmware.elf
  # Show ccache statistics
 )
 
if(CCACHE_PROGRAM)
  add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CCACHE_PROGRAM} -s
  )
endif()