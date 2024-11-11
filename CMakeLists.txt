CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m7)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_C_COMPILER "C:/Work_Space/environment/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_ASM_COMPILER "C:/Work_Space/environment/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_C_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -std=c99")
SET(CMAKE_ASM_FLAGS " -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_COMPILER "C:/Work_Space/environment/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-g++.exe")
SET(CMAKE_CXX_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_OBJCOPY "C:/Work_Space/environment/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-objcopy.exe")
SET(CMAKE_SIZE "C:/Work_Space/environment/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-size.exe")

SET(CMAKE_EXE_LINKER_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,Reset_Handler -T ${CMAKE_SOURCE_DIR}/board/linker_scripts/link.lds -Wl,--gc-sections,--print-memory-usage")

SET(CMAKE_C_STANDARD 99)
SET(CMAKE_CXX_STANDARD 17)

PROJECT(rtthread C CXX ASM)

INCLUDE_DIRECTORIES(
	applications
	../../../components/libc/compilers/common/include
	../../../components/libc/compilers/newlib
	../../../components/libc/cplusplus
	../../../components/libc/cplusplus/cpp11/gcc
	../../../components/libc/cplusplus/os
	../../../components/drivers/include
	board
	board/CubeMX_Config/Inc
	../libraries/HAL_Drivers/drivers
	../libraries/HAL_Drivers/drivers/config
	../libraries/HAL_Drivers
	../libraries/HAL_Drivers/CMSIS/Include
	../../../components/dfs/dfs_v1/include
	../../../components/dfs/dfs_v1/filesystems/devfs
	../../../components/finsh
	.
	../../../include
	../../../components/drivers/ktime
	../../../components/drivers/ktime/inc
	../../../libcpu/arm/common
	../../../libcpu/arm/cortex-m7
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Inc
	../libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Include
	../../../components/libc/posix/delay
	../../../components/libc/posix/io/epoll
	../../../components/libc/posix/io/eventfd
	../../../components/libc/posix/io/poll
	../../../components/libc/posix/io/stdio
	../../../components/libc/posix/ipc
	../../../components/libc/posix/pthreads
	../../../components/libc/posix/tls
	../../../components/legacy/usb/usbdevice
	../../../components/drivers/smp
	../../../components/utilities/ulog
)

ADD_DEFINITIONS(
	-DRT_USING_LIBC
	-DRT_USING_NEWLIBC
	-DSTM32H723xx
	-DUSE_HAL_DRIVER
	-D_POSIX_C_SOURCE=1
	-D__RTTHREAD__
)

# Library source files
SET(RT_APPLICATIONS_SOURCES
	applications/main.c
)

SET(RT_COMPILER_SOURCES
	../../../components/libc/compilers/common/cctype.c
	../../../components/libc/compilers/common/cstdlib.c
	../../../components/libc/compilers/common/cstring.c
	../../../components/libc/compilers/common/ctime.c
	../../../components/libc/compilers/common/cunistd.c
	../../../components/libc/compilers/common/cwchar.c
	../../../components/libc/compilers/newlib/syscalls.c
)

SET(RT_CPP_SOURCES
	../../../components/libc/cplusplus/cpp11/atomic_8.c
	../../../components/libc/cplusplus/cpp11/gcc/condition_variable.cpp
	../../../components/libc/cplusplus/cpp11/gcc/mutex.cpp
	../../../components/libc/cplusplus/cpp11/gcc/thread.cpp
	../../../components/libc/cplusplus/cpp11/gcc/utils.cpp
	../../../components/libc/cplusplus/cpp11/thread_local_impl.cpp
	../../../components/libc/cplusplus/cxx_crt.cpp
	../../../components/libc/cplusplus/cxx_crt_init.c
	../../../components/libc/cplusplus/os/cxx_Mutex.cpp
	../../../components/libc/cplusplus/os/cxx_Semaphore.cpp
	../../../components/libc/cplusplus/os/cxx_Thread.cpp
)

SET(RT_DEVICEDRIVERS_SOURCES
	../../../components/drivers/core/device.c
	../../../components/drivers/ipc/completion_comm.c
	../../../components/drivers/ipc/completion_up.c
	../../../components/drivers/ipc/condvar.c
	../../../components/drivers/ipc/dataqueue.c
	../../../components/drivers/ipc/pipe.c
	../../../components/drivers/ipc/ringblk_buf.c
	../../../components/drivers/ipc/ringbuffer.c
	../../../components/drivers/ipc/waitqueue.c
	../../../components/drivers/ipc/workqueue.c
	../../../components/drivers/pin/dev_pin.c
	../../../components/drivers/rtc/dev_rtc.c
	../../../components/drivers/serial/dev_serial.c
)

SET(RT_DRIVERS_SOURCES
	../libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s
	board/CubeMX_Config/Src/stm32h7xx_hal_msp.c
	board/CubeMX_Config/Src/usbd_conf.c
	board/board.c
	../libraries/HAL_Drivers/drivers/drv_gpio.c
	../libraries/HAL_Drivers/drivers/drv_usart.c
	../libraries/HAL_Drivers/drv_common.c
)

SET(RT_FILESYSTEM_SOURCES
	../../../components/dfs/dfs_v1/filesystems/devfs/devfs.c
	../../../components/dfs/dfs_v1/src/dfs.c
	../../../components/dfs/dfs_v1/src/dfs_file.c
	../../../components/dfs/dfs_v1/src/dfs_fs.c
	../../../components/dfs/dfs_v1/src/dfs_posix.c
)

SET(RT_FINSH_SOURCES
	../../../components/finsh/cmd.c
	../../../components/finsh/msh.c
	../../../components/finsh/msh_file.c
	../../../components/finsh/shell.c
	../../../components/finsh/msh_parse.c
)

SET(RT_KERNEL_SOURCES
	../../../src/clock.c
	../../../src/components.c
	../../../src/cpu_up.c
	../../../src/defunct.c
	../../../src/idle.c
	../../../src/ipc.c
	../../../src/irq.c
	../../../src/klibc/kerrno.c
	../../../src/klibc/kstdio.c
	../../../src/klibc/kstring.c
	../../../src/kservice.c
	../../../src/mem.c
	../../../src/mempool.c
	../../../src/object.c
	../../../src/scheduler_comm.c
	../../../src/scheduler_up.c
	../../../src/thread.c
	../../../src/timer.c
)

SET(RT_KTIME_SOURCES
	../../../components/drivers/ktime/src/hrtimer.c
	../../../components/drivers/ktime/src/cputimer.c
	../../../components/drivers/ktime/src/boottime.c
)

SET(RT_LIBCPU_SOURCES
	../../../libcpu/arm/common/div0.c
	../../../libcpu/arm/common/showmem.c
	../../../libcpu/arm/cortex-m7/context_gcc.S
	../../../libcpu/arm/cortex-m7/cpu_cache.c
	../../../libcpu/arm/cortex-m7/cpuport.c
)

SET(RT_LIBRARIES_SOURCES
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hcd.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sram.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
	../libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_comp.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cec.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
	../libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c
)

SET(RT_POSIX_SOURCES
	../../../components/libc/posix/delay/delay.c
	../../../components/libc/posix/io/stdio/stdio.c
	../../../components/libc/posix/pthreads/pthread.c
	../../../components/libc/posix/pthreads/pthread_attr.c
	../../../components/libc/posix/pthreads/pthread_barrier.c
	../../../components/libc/posix/pthreads/pthread_cond.c
	../../../components/libc/posix/pthreads/pthread_mutex.c
	../../../components/libc/posix/pthreads/pthread_rwlock.c
	../../../components/libc/posix/pthreads/pthread_spin.c
	../../../components/libc/posix/pthreads/pthread_tls.c
	../../../components/libc/posix/pthreads/sched.c
	../../../components/libc/posix/tls/emutls.c
)

SET(RT_RT_USBD_SOURCES
	../../../components/legacy/usb/usbdevice/class/cdc_vcom.c
	../../../components/legacy/usb/usbdevice/core/usbdevice.c
	../../../components/legacy/usb/usbdevice/core/usbdevice_core.c
)

SET(RT_SMP_SOURCES
)

SET(RT_UTILITIES_SOURCES
	../../../components/utilities/ulog/ulog.c
	../../../components/utilities/ulog/backend/console_be.c
)

# Library search paths
# Library local macro definitions
SET(RT_DEVICEDRIVERS_DEFINES
	__RT_IPC_SOURCE__
)

SET(RT_KERNEL_DEFINES
	__RT_KERNEL_SOURCE__
)

# Library dependencies
SET(RT_COMPILER_LIBS
	c
	m
)

# Libraries
ADD_LIBRARY(rtt_Compiler OBJECT ${RT_COMPILER_SOURCES})
ADD_LIBRARY(rtt_CPP OBJECT ${RT_CPP_SOURCES})
ADD_LIBRARY(rtt_DeviceDrivers OBJECT ${RT_DEVICEDRIVERS_SOURCES})
ADD_LIBRARY(rtt_Drivers OBJECT ${RT_DRIVERS_SOURCES})
ADD_LIBRARY(rtt_Filesystem OBJECT ${RT_FILESYSTEM_SOURCES})
ADD_LIBRARY(rtt_Finsh OBJECT ${RT_FINSH_SOURCES})
ADD_LIBRARY(rtt_Kernel OBJECT ${RT_KERNEL_SOURCES})
ADD_LIBRARY(rtt_ktime OBJECT ${RT_KTIME_SOURCES})
ADD_LIBRARY(rtt_libcpu OBJECT ${RT_LIBCPU_SOURCES})
ADD_LIBRARY(rtt_Libraries OBJECT ${RT_LIBRARIES_SOURCES})
ADD_LIBRARY(rtt_POSIX OBJECT ${RT_POSIX_SOURCES})
ADD_LIBRARY(rtt_rt_usbd OBJECT ${RT_RT_USBD_SOURCES})
ADD_LIBRARY(rtt_Utilities OBJECT ${RT_UTILITIES_SOURCES})

# Interface libraries
ADD_LIBRARY(rtt_smp INTERFACE)

# Private macros
TARGET_COMPILE_DEFINITIONS(rtt_DeviceDrivers PRIVATE ${RT_DEVICEDRIVERS_DEFINES})
TARGET_COMPILE_DEFINITIONS(rtt_Kernel PRIVATE ${RT_KERNEL_DEFINES})

# Interface library search paths
TARGET_LINK_LIBRARIES(rtt_Compiler INTERFACE ${RT_COMPILER_LIBS})

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${RT_APPLICATIONS_SOURCES})
TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf
	rtt_Compiler
	rtt_CPP
	rtt_DeviceDrivers
	rtt_Drivers
	rtt_Filesystem
	rtt_Finsh
	rtt_Kernel
	rtt_ktime
	rtt_libcpu
	rtt_Libraries
	rtt_POSIX
	rtt_rt_usbd
	rtt_Utilities
	rtt_smp
)

ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
	COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf rtthread.bin
	COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# if custom.cmake is exist, add it
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
endif()
