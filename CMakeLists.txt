cmake_minimum_required(VERSION 3.7)

###############################################################################
### toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CROSS_NAME arm-none-eabi)
set(CMAKE_FORCE_C_COMPILER ${CROSS_NAME}-gcc)
set(CMAKE_FORCE_CXX_COMPILER ${CROSS_NAME}-g++)
SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER ${CROSS_NAME}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_NAME}-g++)

#set(TOOLCHAIN_PATH /opt/gcc-arm-none-eabi-5_4-2016q3)
#set(CROSS_NAME arm-atollic-eabi)
#set(TOOLCHAIN_PATH /opt/Atollic_TrueSTUDIO_for_ARM_7.1.0/ARMTools)

#set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-gcc)
#set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-g++)
#set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH}/${CROSS_NAME})

#set(CMAKE_FIND_ROOT_PATH /usr/lib/gcc/arm-none-eabi/5.4.1)#/usr/lib/${CROSS_NAME})
#set(CMAKE_MAKE_PROGRAM make)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


###############################################################################
### options
# target flag
set(COMMON_FLAGS "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16" )
add_definitions(-DSTM32F407xx)

# required flags
add_definitions(-DUSE_HAL_DRIVER -D_IO=volatile ${COMMON_FLAGS} )

# compile flags
add_definitions(-ffunction-sections -fdata-sections -fstack-usage -Wall -Os)
set(C_COVERAGE_FLAGS "-std=gnu11")
set(CXX_COVERAGE_FLAGS "-std=c++14 -fno-threadsafe-statics")
set(LINK_COVERAGE_FLAGS "${COMMON_FLAGS} -Wl,--gc-sections -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -specs=nano.specs -specs=nosys.specs -static -Wl,-u,Reset_Handler")

# routine
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_COVERAGE_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_COVERAGE_FLAGS}")

# debug
#set(CMAKE_VERBOSE_MAKEFILE ON)

###############################################################################
### files
## project files
project(boiler C CXX ASM)
include_directories(
        ${CMAKE_SOURCE_DIR}/Inc
        ${CMAKE_SOURCE_DIR}/Othres)
file(GLOB EXE_FILES
        ${CMAKE_SOURCE_DIR}/Src/*
        ${CMAKE_SOURCE_DIR}/Others/*.c
        ${CMAKE_SOURCE_DIR}/Others/*.cpp)

## middlewares
set(MIDDLEWARES_PATH ${CMAKE_SOURCE_DIR}/Middlewares)
include_directories(
	${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Core/Inc
        ${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Class/CDC/Inc
        ${MIDDLEWARES_PATH}/Third_Party/u8glib
        ${MIDDLEWARES_PATH}/Third_Party/u8glib/src
        ${MIDDLEWARES_PATH}/Third_Party/OneWire
        ${MIDDLEWARES_PATH}/Third_Party/MAX31855)
file(GLOB MIDDLEWARES_EXE_FILES
        ${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Core/Src/*
        ${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Class/CDC/Src/*
        ${MIDDLEWARES_PATH}/Third_Party/u8glib/*.c
        ${MIDDLEWARES_PATH}/Third_Party/u8glib/src/*.c
        ${MIDDLEWARES_PATH}/Third_Party/OneWire/*.c
        ${MIDDLEWARES_PATH}/Third_Party/OneWire/*.s
        ${MIDDLEWARES_PATH}/Third_Party/MAX31855/*.c)

## freeRTOS
set(FREERTOS_PATH ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS)
include_directories(
        ${FREERTOS_PATH}/Source/CMSIS_RTOS
        ${FREERTOS_PATH}/Source/include
        ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM4F)
file(GLOB FREERTOS_EXE_FILES
        ${FREERTOS_PATH}/Source/*
        ${FREERTOS_PATH}/Source/CMSIS_RTOS/*
        ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM4F/*
        ${FREERTOS_PATH}/Source/portable/MemMang/*)

## LwIP
set(LWIP_PATH ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/LwIP)
include_directories(
        ${LWIP_PATH}/src/include
        ${LWIP_PATH}/system
        ${LWIP_PATH}/src/apps/httpd)
file(GLOB LWIP_EXE_FILES
        ${LWIP_PATH}/system/OS/*.c
        ${LWIP_PATH}/src/netif/ppp/*.c
        ${LWIP_PATH}/src/netif/*.c
        ${LWIP_PATH}/src/core/*.c
        ${LWIP_PATH}/src/core/ipv4/*.c
        ${LWIP_PATH}/src/core/ipv6/*.c
        ${LWIP_PATH}/src/api/*.c
        ${LWIP_PATH}/src/apps/httpd/*.c)

## drivers files
set(DRIVERS_PATH ${CMAKE_SOURCE_DIR}/Drivers)
include_directories(
        ${DRIVERS_PATH}/STM32F4xx_HAL_Driver/Inc
        ${DRIVERS_PATH}/STM32F4xx_HAL_Driver/Inc/Legacy
        ${DRIVERS_PATH}/CMSIS/Device/ST/STM32F4xx/Include
        ${DRIVERS_PATH}/CMSIS/Include )
file(GLOB DRIVERS_EXE_FILES
        ${DRIVERS_PATH}/STM32F4xx_HAL_Driver/Src/*
        ${DRIVERS_PATH}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        ${CMAKE_SOURCE_DIR}/startup/startup_stm32f407xx.s)

## main logic files
include_directories(${CMAKE_SOURCE_DIR}/logic)
file(GLOB LOGIC_EXE_FILES
        ${CMAKE_SOURCE_DIR}/logic/*.c
        ${CMAKE_SOURCE_DIR}/logic/*.cpp)

## http filesystem file
include_directories(${CMAKE_SOURCE_DIR}/httpfs)

## linker script file
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VG_FLASH.ld)

###############################################################################
### target
## executable
add_executable(${PROJECT_NAME}
    ${EXE_FILES}
    ${DRIVERS_EXE_FILES}
    ${MIDDLEWARES_EXE_FILES}
    ${LWIP_EXE_FILES}
    ${FREERTOS_EXE_FILES}
    ${LOGIC_EXE_FILES})

## build http filesystem
file(GLOB_RECURSE fsdata_files RELATIVE ${CMAKE_SOURCE_DIR}/httpfs  #/src
    ${CMAKE_SOURCE_DIR}/httpfs/*) #src/
list(REMOVE_ITEM fsdata_files "fsdata.c")
add_custom_target( fsdata
    COMMAND ${CMAKE_SOURCE_DIR}/makefsdata #${CMAKE_SOURCE_DIR}/httpfs/src/
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/httpfs/
    COMMENT "Make HTTP filesystem from: ${fsdata_files}"
    BYPRODUCTS ${CMAKE_SOURCE_DIR}/httpfs/fsdata.c )
    #DEPENDS ${CMAKE_SOURCE_DIR}/httpfs/src)
add_dependencies(${PROJECT_NAME} fsdata)

##linking
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT} ${LINK_COVERAGE_FLAGS}")

## objcopy
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
    ${CROSS_NAME}-objcopy -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
    ${CROSS_NAME}-size ${PROJECT_NAME})
## program
#add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
#openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "init" -c "reset init" -c "flash write_image erase %{CurrentProject:Name}.hex" -c "reset" -c "shutdown" )


