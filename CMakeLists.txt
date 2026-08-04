# ---------------------------------------------------------------------------
# Project Phase 2 — firmware build for both MCUs.
#
#   station : STM32F446RE (Cortex-M4)  laptop UART  -> nRF24
#   car     : STM32F103C8 (Cortex-M3)  nRF24        -> motors / LEDs
#
# Both link the same portable modules (contracts/rf_protocol, shared/nrf24),
# each with its own driver layer.
#
# Configure and build:
#   cmake -B build -G Ninja --toolchain cmake/arm-none-eabi.cmake
#   cmake --build build                 # both
#   cmake --build build --target car    # just one
#
# Flash (needs OpenOCD; see OPENOCD_BIN below):
#   cmake --build build --target flash-car
#   cmake --build build --target flash-station
#
# The C# control app is NOT part of this build — it is a .NET project:
#   cd station/control-app && dotnet run
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

project(project-phase-2 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# Strict -std=c11 rather than -std=gnu11: with gnu11 selected, -Wpedantic stops
# flagging GNU extensions, silently weakening the warnings the code is held to.
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Flags common to both MCUs
# ---------------------------------------------------------------------------
set(COMMON_C_FLAGS
    -Wall -Wextra -Wpedantic
    -ffunction-sections -fdata-sections   # let the linker drop unused code
    -fno-common
)

set(COMMON_LINK_FLAGS
    -specs=nano.specs                     # newlib-nano: small printf/malloc
    -specs=nosys.specs                    # stub out syscalls (bare metal)
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# ---------------------------------------------------------------------------
# Portable sources — identical on both targets
# ---------------------------------------------------------------------------
set(PORTABLE_SOURCES
    ${CMAKE_SOURCE_DIR}/contracts/rf_protocol.c
    ${CMAKE_SOURCE_DIR}/shared/nrf24/nrf24.c
)
set(PORTABLE_INCLUDES
    ${CMAKE_SOURCE_DIR}/contracts
    ${CMAKE_SOURCE_DIR}/shared/nrf24
    ${CMAKE_SOURCE_DIR}/platform/cmsis
)

# ---------------------------------------------------------------------------
# add_firmware(<name> CPU <flags> DEVICE <define> LINKER <script>
#              SOURCES <...> INCLUDES <...>)
#
# Builds <name>.elf, then emits .bin and .hex and prints the size summary.
# ---------------------------------------------------------------------------
function(add_firmware NAME)
    cmake_parse_arguments(FW "" "CPU;DEVICE;LINKER" "SOURCES;INCLUDES" ${ARGN})

    add_executable(${NAME} ${FW_SOURCES} ${PORTABLE_SOURCES})

    set_target_properties(${NAME} PROPERTIES
        OUTPUT_NAME "${NAME}"
        SUFFIX ".elf"
    )

    target_include_directories(${NAME} PRIVATE ${FW_INCLUDES} ${PORTABLE_INCLUDES})
    target_compile_definitions(${NAME} PRIVATE ${FW_DEVICE})

    separate_arguments(CPU_FLAGS UNIX_COMMAND "${FW_CPU}")
    target_compile_options(${NAME} PRIVATE ${CPU_FLAGS} ${COMMON_C_FLAGS})

    # The CPU flags must also reach the linker so it picks the right libc variant.
    target_link_options(${NAME} PRIVATE
        ${CPU_FLAGS}
        ${COMMON_LINK_FLAGS}
        -T${FW_LINKER}
        -Wl,-Map=$<TARGET_FILE_DIR:${NAME}>/${NAME}.map
    )

    # Relink whenever the linker script changes.
    set_target_properties(${NAME} PROPERTIES LINK_DEPENDS ${FW_LINKER})

    add_custom_command(TARGET ${NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${NAME}>
                $<TARGET_FILE_DIR:${NAME}>/${NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${NAME}>
                $<TARGET_FILE_DIR:${NAME}>/${NAME}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${NAME}>
        COMMENT "Generating ${NAME}.bin / ${NAME}.hex"
        VERBATIM
    )
endfunction()

# ---------------------------------------------------------------------------
# station — STM32F446RE, Cortex-M4 (soft float: no FPU instructions emitted)
# ---------------------------------------------------------------------------
file(GLOB STATION_DRIVERS CONFIGURE_DEPENDS station/drivers/*.c)

add_firmware(station
    CPU     "-mcpu=cortex-m4 -mthumb"
    DEVICE  STM32F446xx
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f446/STM32F446RETX_FLASH.ld
    SOURCES
        ${STATION_DRIVERS}
        station/app/main.c
        platform/f446/system_stm32f4xx.c
        platform/f446/startup_stm32f446retx.s
    INCLUDES
        ${CMAKE_SOURCE_DIR}/station/drivers
        ${CMAKE_SOURCE_DIR}/station/app
        ${CMAKE_SOURCE_DIR}/platform/f446
)

# ---------------------------------------------------------------------------
# car — STM32F103C8, Cortex-M3
# ---------------------------------------------------------------------------
file(GLOB CAR_DRIVERS CONFIGURE_DEPENDS car/drivers/*.c)
file(GLOB CAR_DEVICES CONFIGURE_DEPENDS car/devices/tb6612/*.c)
file(GLOB CAR_MIXER   CONFIGURE_DEPENDS car/lib/mixer/*.c)

add_firmware(car
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        ${CAR_DEVICES}
        ${CAR_MIXER}
        car/nodes/gateway/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CMAKE_SOURCE_DIR}/car/drivers
        ${CMAKE_SOURCE_DIR}/car/devices/tb6612
        ${CMAKE_SOURCE_DIR}/car/lib/mixer
        ${CMAKE_SOURCE_DIR}/car/nodes/gateway
        ${CMAKE_SOURCE_DIR}/platform/f103
)

# ---------------------------------------------------------------------------
# Flashing via OpenOCD
#
# Override the binary if yours lives elsewhere:
#   cmake -B build ... -DOPENOCD_BIN=/path/to/openocd
# ---------------------------------------------------------------------------
find_program(OPENOCD_BIN openocd
    HINTS "C:/OpenOCD/xpack-openocd-0.12.0-7/bin"
    DOC   "OpenOCD executable used by the flash-* targets"
)

if(OPENOCD_BIN)
    # station: on-board ST-Link V2-1. car: external ST-Link V2 on the SWD header.
    add_custom_target(flash-station
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg -f target/stm32f4x.cfg
                -c "program $<TARGET_FILE:station> verify reset exit"
        DEPENDS station
        COMMENT "Flashing station (STM32F446RE) via OpenOCD"
        VERBATIM
    )

    add_custom_target(flash-car
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:car> verify reset exit"
        DEPENDS car
        COMMENT "Flashing car (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    # diag-car: read the car's s_diag snapshot over SWD and decode it (the car
    # has no UART). Needs nm to locate the struct in the ELF.
    find_program(ARM_NM NAMES arm-none-eabi-nm)
    if(ARM_NM)
        add_custom_target(diag-car
            COMMAND ${CMAKE_COMMAND}
                    -DNM=${ARM_NM}
                    -DELF=$<TARGET_FILE:car>
                    -DOPENOCD=${OPENOCD_BIN}
                    -P ${CMAKE_SOURCE_DIR}/cmake/read_diag.cmake
            DEPENDS car
            USES_TERMINAL
            COMMENT "Reading car diagnostics (s_diag) over SWD"
            VERBATIM
        )
    else()
        message(STATUS "arm-none-eabi-nm not found - diag-car target disabled")
    endif()

else()
    message(STATUS "OpenOCD not found - flash-* targets disabled "
                   "(set -DOPENOCD_BIN=... to enable)")
endif()
