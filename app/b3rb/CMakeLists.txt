#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

if (${BOARD} STREQUAL "mr_canhubk3")
    message(STATUS "enabling mr_canhubk3_adap shield")
    set(SHIELD mr_canhubk3_adap)
endif ()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(. LANGUAGES C)

set(SOURCE_FILES src/main.c)

list(APPEND SOURCE_FILES src/fsm.c)
list(APPEND SOURCE_FILES src/mixing.c)
list(APPEND SOURCE_FILES src/manual.c)
list(APPEND SOURCE_FILES src/auto.c)
list(APPEND SOURCE_FILES src/movement.c)
list(APPEND SOURCE_FILES src/lighting.c)

list(APPEND SOURCE_FILES
        src/casadi/gen/b3rb.c)

set(flags
        -std=c11
        -Wall
        -Wextra
        -Werror
        -Wstrict-prototypes
        -Waggregate-return
        -Wbad-function-cast
        -Wcast-align
        -Wcast-qual
        -Wfloat-equal
        -Wformat-security
        -Wlogical-op
        -Wmissing-declarations
        # ubxlib includes geographiclib which dir doesn't exist
        #-Wmissing-include-dirs
        -Wmissing-prototypes
        -Wnested-externs
        -Wpointer-arith
        -Wredundant-decls
        -Wsequence-point
        -Wshadow
        -Wstrict-prototypes
        -Wswitch
        -Wundef
        -Wunreachable-code
        -Wunused-but-set-parameter
        -Wwrite-strings
)
string(JOIN " " flags ${flags})

set_source_files_properties(
        ${SOURCE_FILES}
        PROPERTIES COMPILE_FLAGS
        "${flags}"
)

set_source_files_properties(
        src/casadi/gen/b3rb.c
        PROPERTIES COMPILE_FLAGS
        "${flags}\
  -Wno-unused-parameter\
  -Wno-missing-prototypes\
  -Wno-missing-declarations\
  -Wno-float-equal")

target_sources(app PRIVATE ${SOURCE_FILES})

target_include_directories(app SYSTEM BEFORE PRIVATE ${ZEPHYR_BASE}/include)

# vi: ts=2 sw=2 et
