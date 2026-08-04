# ---------------------------------------------------------------------------
# Cross-compilation toolchain for bare-metal ARM Cortex-M (arm-none-eabi-gcc).
#
# Shared by both MCUs in this project; the CPU-specific flags (-mcpu, device
# define, linker script) are applied per target in the top-level CMakeLists.txt,
# because the STM32F446 is Cortex-M4 and the STM32F103 is Cortex-M3.
#
# Usage:
#   cmake -B build -G Ninja --toolchain cmake/arm-none-eabi.cmake
# ---------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# There is no OS and no startup code during CMake's compiler probe, so a full
# link would fail. Building a static library instead is the standard fix.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi)

find_program(ARM_CC      ${TOOLCHAIN_PREFIX}-gcc     REQUIRED)
find_program(ARM_CXX     ${TOOLCHAIN_PREFIX}-g++)
find_program(ARM_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy REQUIRED)
find_program(ARM_SIZE    ${TOOLCHAIN_PREFIX}-size    REQUIRED)
find_program(ARM_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
find_program(ARM_NM      ${TOOLCHAIN_PREFIX}-nm      REQUIRED)

set(CMAKE_C_COMPILER   "${ARM_CC}")
set(CMAKE_ASM_COMPILER "${ARM_CC}")   # gcc drives the assembler for .s files
if(ARM_CXX)
    set(CMAKE_CXX_COMPILER "${ARM_CXX}")
endif()

set(CMAKE_OBJCOPY "${ARM_OBJCOPY}" CACHE FILEPATH "objcopy"  FORCE)
set(CMAKE_SIZE    "${ARM_SIZE}"    CACHE FILEPATH "size"     FORCE)
set(CMAKE_OBJDUMP "${ARM_OBJDUMP}" CACHE FILEPATH "objdump"  FORCE)
set(CMAKE_NM      "${ARM_NM}"      CACHE FILEPATH "nm"       FORCE)

# Look for programs on the host, but headers/libraries only in the target sysroot.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
