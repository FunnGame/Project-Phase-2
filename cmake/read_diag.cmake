# ---------------------------------------------------------------------------
# read_diag.cmake — read the car's s_diag snapshot over SWD and decode it.
#
# The F103 car has no UART, so the firmware fills a RAM struct (car_diag_t
# s_diag in car/nodes/gateway/main.c) at boot and this script reads it back
# through the debugger:
#   1. locate s_diag in the ELF with nm,
#   2. have OpenOCD halt the target and dump the struct with `mdw`,
#   3. decode each 32-bit field and compare it to the expected value.
#
# Run via the diag-car target, not directly:
#   cmake --build build --target diag-car
#
# Invoked as: cmake -DNM=.. -DELF=.. -DOPENOCD=.. -P cmake/read_diag.cmake
#
# car_diag_t layout (13 x uint32, one mdw word each, in order):
#   0 magic  1 sysclk  2 pclk1  3 timer  4 clock_ok  5 radio_ok
#   6 CONFIG 7 EN_AA   8 SETUP_RETR  9 RF_CH  10 RF_SETUP  11 STATUS  12 RX_PW_P0
# ---------------------------------------------------------------------------
if(NOT NM OR NOT ELF OR NOT OPENOCD)
    message(FATAL_ERROR "read_diag: need -DNM= -DELF= -DOPENOCD=")
endif()

set(DIAG_WORDS 13)
set(DIAG_MAGIC "d1a6c0de")

# --- 1. find the address of s_diag ----------------------------------------
execute_process(
    COMMAND "${NM}" "${ELF}"
    OUTPUT_VARIABLE nm_out
    RESULT_VARIABLE nm_rc
)
if(NOT nm_rc EQUAL 0)
    message(FATAL_ERROR "nm failed on ${ELF}")
endif()

# A line reads like "20000abc b s_diag" (b/B = .bss, d/D = .data).
if(NOT nm_out MATCHES "([0-9a-fA-F]+) [bBdD] s_diag")
    message(FATAL_ERROR
        "s_diag not found in ${ELF}.\n"
        "Build with CAR_DIAG=1 (the default) so the snapshot is compiled in.")
endif()
set(DIAG_ADDR "0x${CMAKE_MATCH_1}")
message(STATUS "s_diag @ ${DIAG_ADDR}")

# --- 2. dump the struct over SWD ------------------------------------------
execute_process(
    COMMAND "${OPENOCD}"
            -f interface/stlink.cfg -f target/stm32f1x.cfg
            -c "init" -c "halt"
            -c "mdw ${DIAG_ADDR} ${DIAG_WORDS}"
            -c "resume" -c "exit"
    OUTPUT_VARIABLE ocd_out
    ERROR_VARIABLE  ocd_err
    RESULT_VARIABLE ocd_rc
)
set(dump "${ocd_out}\n${ocd_err}")      # OpenOCD prints mdw output on stderr
string(REGEX REPLACE "\r" "" dump "${dump}")   # drop Windows CRs before parsing
if(NOT ocd_rc EQUAL 0)
    message(FATAL_ERROR "OpenOCD failed:\n${dump}")
endif()

message(STATUS "---- raw OpenOCD output ----")
message(STATUS "${dump}")
message(STATUS "----------------------------")

# --- 3. collect the hex words ---------------------------------------------
# An mdw line: "0x20000abc: 11223344 55667788 ...". Anchor on the address+colon
# (OpenOCD's banner also contains hex), and split the remainder on whitespace —
# CMake's regex engine has no {n} repetition, so length-matching each word does
# not work; splitting sidesteps that entirely.
string(REPLACE "\n" ";" _lines "${dump}")
set(words "")
foreach(line IN LISTS _lines)
    if(line MATCHES "^0x[0-9a-fA-F]+:(.*)")
        string(STRIP "${CMAKE_MATCH_1}" _rest)
        string(REGEX REPLACE "[ \t]+" ";" _toks "${_rest}")
        foreach(t IN LISTS _toks)
            if(t MATCHES "^[0-9a-fA-F]+$")
                list(APPEND words "${t}")
            endif()
        endforeach()
    endif()
endforeach()

list(LENGTH words nwords)
message(STATUS "parsed ${nwords} words: ${words}")
if(nwords LESS DIAG_WORDS)
    message(FATAL_ERROR
        "expected ${DIAG_WORDS} words, parsed ${nwords} — see raw output above.")
endif()

foreach(i RANGE 0 12)
    list(GET words ${i} w${i})
    math(EXPR v${i} "0x${w${i}}")       # decimal value of the word
endforeach()

# --- 4. decode -------------------------------------------------------------
# math(EXPR) is integer arithmetic only (no ==, >, AND) — all comparisons go
# through if(), which does support EQUAL/GREATER/LESS/AND.
set(fail 0)

# report(<name> <value> <ok 0|1> <expected-text>)
function(report name value ok expected)
    if(ok)
        set(tag "  ok")
    else()
        set(tag "FAIL")
        set(fail 1 PARENT_SCOPE)
    endif()
    message(STATUS "  [${tag}] ${name} = ${value}   (expect ${expected})")
endfunction()

message(STATUS "")
message(STATUS "==== car diagnostic ====")

if(w0 STREQUAL DIAG_MAGIC)
    message(STATUS "  magic ok (0x${w0})")
else()
    message(STATUS "")
    message(STATUS "  ################################################################")
    message(STATUS "  #  MAGIC MISMATCH: read 0x${w0}, expected 0x${DIAG_MAGIC}")
    message(STATUS "  #  The RAM at s_diag does NOT hold this build's struct, so every")
    message(STATUS "  #  value below is meaningless. Almost always a stale flash:")
    message(STATUS "  #    cmake --build build --target flash-car")
    message(STATUS "  #  then re-run diag-car. (Also check the car actually reset and")
    message(STATUS "  #  reached board_init.)")
    message(STATUS "  ################################################################")
    message(STATUS "")
endif()

# Clocks (allow ~1% slack).
set(sys_ok 0)
if(v1 GREATER 71000000 AND v1 LESS 73000000)
    set(sys_ok 1)
endif()
set(pclk_ok 0)
if(v2 GREATER 35000000 AND v2 LESS 37000000)
    set(pclk_ok 1)
endif()
report("SYSCLK Hz" "${v1}" "${sys_ok}"  "~72000000")
report("PCLK1  Hz" "${v2}" "${pclk_ok}" "~36000000  (feeds SPI2)")
message(STATUS "         TIM3 Hz = ${v3}")
report("clock_ok"  "${v4}" "${v4}" "1")
report("radio_ok"  "${v5}" "${v5}" "1")

# nRF24 registers, on the low byte (& is supported by math(EXPR)).
math(EXPR c  "${v6}  & 0xFF")
math(EXPR a  "${v7}  & 0xFF")
math(EXPR sr "${v8}  & 0xFF")
math(EXPR ch "${v9}  & 0xFF")
math(EXPR rs "${v10} & 0xFF")
math(EXPR st "${v11} & 0xFF")
math(EXPR pw "${v12} & 0xFF")

# reg_ok(<var> <value> <expected-decimal>) -> sets <var> to 0/1
macro(reg_ok out val exp)
    if(${val} EQUAL ${exp})
        set(${out} 1)
    else()
        set(${out} 0)
    endif()
endmacro()
reg_ok(c_ok  "${c}"  15)   # 0x0F
reg_ok(a_ok  "${a}"   1)
reg_ok(sr_ok "${sr}" 31)   # 0x1F
reg_ok(ch_ok "${ch}" 76)   # 0x4C
reg_ok(rs_ok "${rs}"  6)
reg_ok(pw_ok "${pw}"  7)

message(STATUS "  ---- nRF24 registers ----")
report("CONFIG"     "${c}"  "${c_ok}"  "15 = 0x0F (RX)")
report("EN_AA"      "${a}"  "${a_ok}"  "1")
report("SETUP_RETR" "${sr}" "${sr_ok}" "31 = 0x1F")
report("RF_CH"      "${ch}" "${ch_ok}" "76 = 0x4C")
report("RF_SETUP"   "${rs}" "${rs_ok}" "6")
message(STATUS "         STATUS = ${st}  (0x0E idle; RX_DR/TX_DS/MAX_RT bits vary)")
report("RX_PW_P0"   "${pw}" "${pw_ok}" "7")

# Whole-bus verdict from the register set.
set(all_zero 0)
if(c EQUAL 0 AND a EQUAL 0 AND ch EQUAL 0 AND rs EQUAL 0 AND pw EQUAL 0)
    set(all_zero 1)
endif()
set(all_ff 0)
if(c EQUAL 255 AND a EQUAL 255 AND ch EQUAL 255 AND pw EQUAL 255)
    set(all_ff 1)
endif()
message(STATUS "")
if(all_zero)
    message(STATUS "  >> every register 0x00: nRF24 not driving MISO —")
    message(STATUS "     module unpowered, MISO unwired, or CSN never asserting.")
elseif(all_ff)
    message(STATUS "  >> every register 0xFF: MISO floating high —")
    message(STATUS "     MISO disconnected, or MOSI/SCK not reaching the module.")
elseif(fail)
    message(STATUS "  >> some fields off — see FAIL rows above.")
else()
    message(STATUS "  >> clocks and radio registers all nominal.")
endif()
message(STATUS "========================")
