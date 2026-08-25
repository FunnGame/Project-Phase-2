# ---------------------------------------------------------------------------
# Project Phase 2 — firmware build for both MCUs.
#
#   station         : STM32F446RE (M4)  laptop UART -> nRF24
#   gateway         : STM32F103C8 (M3)  nRF24       -> CAN
#   vehicle_control : STM32F103C8 (M3)  CAN         -> TB6612 motors / encoders
#   sensor          : STM32F103C8 (M3)  VL53L0X     -> CAN
#
# Both link the same portable modules (contracts/rf_protocol, shared/nrf24),
# each with its own driver layer.
#
# Configure and build:
#   cmake -B build -G Ninja --toolchain cmake/arm-none-eabi.cmake
#   cmake --build build                     # everything
#   cmake --build build --target gateway    # just one
#
# Flash (needs OpenOCD; see OPENOCD_BIN below):
#   cmake --build build --target flash-gateway
#   cmake --build build --target flash-vehicle-control
#   cmake --build build --target flash-sensor
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

# Emit build/compile_commands.json so the editor sees each file's REAL include
# paths and defines. Without it a mixed-toolchain project like this one has to
# be described twice — once here, once in .vscode — and the two drift.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
file(GLOB STATION_DEVICES CONFIGURE_DEPENDS station/devices/ili9341/*.c)

add_firmware(station
    CPU     "-mcpu=cortex-m4 -mthumb"
    DEVICE  STM32F446xx
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f446/STM32F446RETX_FLASH.ld
    SOURCES
        ${STATION_DRIVERS}
        ${STATION_DEVICES}
        station/app/main.c
        platform/f446/system_stm32f4xx.c
        platform/f446/startup_stm32f446retx.s
    INCLUDES
        ${CMAKE_SOURCE_DIR}/station/drivers
        ${CMAKE_SOURCE_DIR}/station/devices/ili9341
        ${CMAKE_SOURCE_DIR}/station/app
        ${CMAKE_SOURCE_DIR}/platform/f446
)

# ---------------------------------------------------------------------------
# The car is THREE STM32F103C8 boards on one CAN bus. They share car/drivers,
# the generated CAN packers and the E2E layer; each adds only the peripherals
# it owns, and each keeps its OWN <node>_config.h beside its main.c.
#
#   gateway         nRF24 radio -> CAN                 (no actuators)
#   vehicle_control CAN -> TB6612 motors, encoders     (the only node that moves the car)
#   sensor          VL53L0X -> CAN
# ---------------------------------------------------------------------------
file(GLOB CAR_DRIVERS CONFIGURE_DEPENDS car/drivers/*.c)
file(GLOB CAR_DEVICES CONFIGURE_DEPENDS car/devices/tb6612/*.c)
file(GLOB CAR_IMU     CONFIGURE_DEPENDS car/devices/mpu6050/*.c)
file(GLOB CAR_MIXER   CONFIGURE_DEPENDS car/lib/mixer/*.c)
file(GLOB CAR_AEB     CONFIGURE_DEPENDS car/lib/aeb/*.c)

# Shared by every car node.
set(CAR_COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/contracts/gen/adas.c
    ${CMAKE_SOURCE_DIR}/shared/e2e/e2e.c
)
set(CAR_COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}/car/drivers
    ${CMAKE_SOURCE_DIR}/contracts/gen
    ${CMAKE_SOURCE_DIR}/shared/e2e
    ${CMAKE_SOURCE_DIR}/platform/f103
)

# ---------------------------------------------------------------------------
# gateway — radio to CAN. Was the `car` target before the three-way split;
# it no longer links the mixer or the TB6612, because it drives nothing.
# ---------------------------------------------------------------------------
add_firmware(gateway
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        ${CAR_COMMON_SOURCES}
        car/nodes/gateway/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CAR_COMMON_INCLUDES}
        ${CMAKE_SOURCE_DIR}/car/nodes/gateway
)

# ---------------------------------------------------------------------------
# vehicle_control — CAN to motors. Owns everything that can move the car.
# ---------------------------------------------------------------------------
add_firmware(vehicle_control
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        ${CAR_DEVICES}
        ${CAR_IMU}
        ${CAR_MIXER}
        ${CAR_AEB}
        ${CAR_COMMON_SOURCES}
        car/nodes/vehicle_control/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CAR_COMMON_INCLUDES}
        ${CMAKE_SOURCE_DIR}/car/devices/tb6612
        ${CMAKE_SOURCE_DIR}/car/devices/mpu6050
        ${CMAKE_SOURCE_DIR}/car/lib/mixer
        ${CMAKE_SOURCE_DIR}/car/lib/aeb
        ${CMAKE_SOURCE_DIR}/car/nodes/vehicle_control
)

# ---------------------------------------------------------------------------
# sensor — STM32F103C8, Cortex-M3. VL53L0X ranging node.
#
# Publishes object lists on CAN per contracts/adas.dbc (the debug UART it used
# during bring-up is gone). Pulls in the ST VL53L0X API (a vendor blob) + our
# platform port + the HAL, on top of the shared car/drivers, plus the generated
# CAN packers and the shared E2E layer. The vendor Middleware is compiled with
# warnings suppressed — it is not held to the project's -Wall -Wextra bar.
# ---------------------------------------------------------------------------
set(VL53_DIR ${CMAKE_SOURCE_DIR}/car/devices/vl53l0x)
set(VL53_MW  ${VL53_DIR}/Middleware/VL53L0X_1.0.4)
file(GLOB VL53_API CONFIGURE_DEPENDS ${VL53_MW}/Api/core/src/*.c)
set(VL53_MW_SRC ${VL53_API} ${VL53_MW}/Api/platform/src/vl53l0x_platform.c)

add_firmware(sensor
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        ${VL53_MW_SRC}
        ${VL53_DIR}/HAL/src/hal_vl53.c
        ${CMAKE_SOURCE_DIR}/contracts/gen/adas.c
        ${CMAKE_SOURCE_DIR}/shared/e2e/e2e.c
        car/nodes/sensor/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CMAKE_SOURCE_DIR}/car/drivers
        ${VL53_DIR}/HAL/inc
        ${VL53_DIR}/inc
        ${VL53_MW}/Api/core/inc
        ${VL53_MW}/Api/platform/inc
        ${CMAKE_SOURCE_DIR}/car/nodes/sensor
        ${CMAKE_SOURCE_DIR}/contracts/gen
        ${CMAKE_SOURCE_DIR}/shared/e2e
        ${CMAKE_SOURCE_DIR}/platform/f103
)

# The vendor API predates the project's warning flags; don't hold it to them.
set_source_files_properties(${VL53_MW_SRC} TARGET_DIRECTORY sensor
    PROPERTIES COMPILE_OPTIONS "-w")

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

# ---------------------------------------------------------------------------
# imu-probe - standalone MPU6050 bench test for a SPARE STM32F103C8.
#
# Not part of the vehicle and not on any bus: no CAN, no radio, no contract.
# It reuses car/drivers and car/devices/mpu6050 so a pass here is a statement
# about the same driver the vehicle node runs, and reads the SAME CAR_MPU_*
# settings out of vehicle_config.h so bus, address and ranges cannot drift
# from the real thing.
#
#   cmake --build build --target imu-probe
#   cmake --build build --target flash-imu-probe
# ---------------------------------------------------------------------------
add_firmware(imu_probe
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        ${CAR_IMU}
        tests/imu_probe/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CAR_COMMON_INCLUDES}
        ${CMAKE_SOURCE_DIR}/car/devices/mpu6050
        ${CMAKE_SOURCE_DIR}/car/nodes/vehicle_control
)

# ---------------------------------------------------------------------------
# nrf-probe - nRF24L01+ bench test for the GATEWAY board, read out over SWD.
#
# Blink code 3 says "the radio did not respond" and stops there. This says why:
# it dumps the config registers into a global that OpenOCD reads out of RAM, so
# no UART or USB-serial adapter is needed - the ST-Link already attached for
# flashing is the whole instrument.
#
#   cmake --build build --target flash-nrf-probe
#   cmake --build build --target read-nrf-probe
# ---------------------------------------------------------------------------
add_firmware(nrf_probe
    CPU     "-mcpu=cortex-m3 -mthumb"
    DEVICE  STM32F103xB
    LINKER  ${CMAKE_SOURCE_DIR}/platform/f103/STM32F103C8TX_FLASH.ld
    SOURCES
        ${CAR_DRIVERS}
        tests/nrf_probe/main.c
        platform/f103/system_stm32f1xx.c
        platform/f103/startup_stm32f103c8tx.s
    INCLUDES
        ${CAR_COMMON_INCLUDES}
        ${CMAKE_SOURCE_DIR}/car/nodes/gateway
)

if(OPENOCD_BIN)
    # station: on-board ST-Link V2-1. car: external ST-Link V2 on the SWD header.
    #
    # Both adapters are usually plugged in at once, and interface/stlink.cfg
    # matches BOTH - OpenOCD then picks one arbitrarily and dies with a bare
    # "Error: open failed" before it ever reaches the target. Pinning the USB
    # product ID per target makes each flash- goal address its own probe:
    #
    #   0x3748  ST-Link V2   (standalone dongle -> the three F103 car boards)
    #   0x374b  ST-Link V2-1 (on-board on the Nucleo -> the F446 station)
    #
    # The command is "st-link vid_pid" (hyphenated) - it comes from the adapter
    # driver name set by interface/stlink.cfg, and "stlink" is not a command.
    # Override on the command line if you use a clone with a different PID:
    #   cmake -B build ... -DSTLINK_PID_CAR=0x374b
    set(STLINK_PID_CAR     "0x3748" CACHE STRING "USB PID of the car's ST-Link")
    set(STLINK_PID_STATION "0x374b" CACHE STRING "USB PID of the station's ST-Link")

    add_custom_target(flash-station
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_STATION}"
                -f target/stm32f4x.cfg
                -c "program $<TARGET_FILE:station> verify reset exit"
        DEPENDS station
        COMMENT "Flashing station (STM32F446RE) via OpenOCD"
        VERBATIM
    )

    add_custom_target(flash-gateway
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_CAR}"
                -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:gateway> verify reset exit"
        DEPENDS gateway
        COMMENT "Flashing gateway node (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    add_custom_target(flash-vehicle-control
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_CAR}"
                -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:vehicle_control> verify reset exit"
        DEPENDS vehicle_control
        COMMENT "Flashing vehicle-control node (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    add_custom_target(flash-nrf-probe
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_CAR}"
                -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:nrf_probe> verify reset exit"
        DEPENDS nrf_probe
        COMMENT "Flashing nRF24 probe (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    # Reads the result struct back over SWD and decodes it. Safe to re-run.
    #
    # Driven by cmake -P rather than a shell script: on Windows a bare "bash"
    # in a custom target picks up whichever bash is first on PATH, and the WSL
    # one cannot open C:/... paths - it fails with a misleading "No such file
    # or directory" pointing at a script that plainly exists.
    add_custom_target(read-nrf-probe
        COMMAND ${CMAKE_COMMAND}
                -DOPENOCD=${OPENOCD_BIN}
                -DELF=$<TARGET_FILE:nrf_probe>
                -DPID=${STLINK_PID_CAR}
                -P ${CMAKE_SOURCE_DIR}/tests/nrf_probe/read_probe.cmake
        DEPENDS nrf_probe
        COMMENT "Reading nRF24 probe result over SWD"
        VERBATIM
    )

    # Live view of the RUNNING gateway - no reflash, no reset. See gw_debug.h.
    add_custom_target(read-gateway
        COMMAND ${CMAKE_COMMAND}
                -DOPENOCD=${OPENOCD_BIN}
                -DELF=$<TARGET_FILE:gateway>
                -DPID=${STLINK_PID_CAR}
                -P ${CMAKE_SOURCE_DIR}/tests/nrf_probe/read_gateway.cmake
        DEPENDS gateway
        COMMENT "Reading live gateway state over SWD"
        VERBATIM
    )

    # Why did the AEB not fire? Peak-held approach data. See vc_debug.h.
    add_custom_target(read-vehicle
        COMMAND ${CMAKE_COMMAND}
                -DOPENOCD=${OPENOCD_BIN}
                -DELF=$<TARGET_FILE:vehicle_control>
                -DPID=${STLINK_PID_CAR}
                -P ${CMAKE_SOURCE_DIR}/tests/nrf_probe/read_vehicle.cmake
        DEPENDS vehicle_control
        COMMENT "Reading live + peak-held vehicle-control AEB state over SWD"
        VERBATIM
    )

    add_custom_target(flash-imu-probe
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_CAR}"
                -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:imu_probe> verify reset exit"
        DEPENDS imu_probe
        COMMENT "Flashing IMU bench probe (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    add_custom_target(flash-sensor
        COMMAND ${OPENOCD_BIN}
                -f interface/stlink.cfg
                -c "st-link vid_pid 0x0483 ${STLINK_PID_CAR}"
                -f target/stm32f1x.cfg
                -c "program $<TARGET_FILE:sensor> verify reset exit"
        DEPENDS sensor
        COMMENT "Flashing sensor node (STM32F103C8) via OpenOCD"
        VERBATIM
    )

    # diag-car is gone. It read an s_diag snapshot out of the old combined `car`
    # firmware over SWD, because that board had no way to report anything. The
    # nodes now publish their own health on CAN — heartbeats carry uptime, node
    # state and the real CAN error counters — so a USB-CAN adapter shows more,
    # live, without a debugger attached. cmake/read_diag.cmake is unused.

else()
    message(STATUS "OpenOCD not found - flash-* targets disabled "
                   "(set -DOPENOCD_BIN=... to enable)")
endif()
