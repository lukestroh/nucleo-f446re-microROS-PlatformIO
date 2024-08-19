##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.2.0-B44] date: [Fri Aug 16 13:44:48 PDT 2024] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = nucleo-f446re-microROS-PlatformIO


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
src/main.c \
src/stm32f4xx_it.c \
src/stm32f4xx_hal_msp.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
src/system_stm32f4xx.c \
src/freertos.c \
src/stm32f4xx_hal_timebase_tim.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f446xx.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F446xx


# AS includes
AS_INCLUDES = 

# C includes # Edited for platformio
C_INCLUDES =  \
-Iinc \
-I~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Inc \
-I~/.platformio/packages/framework-stm32cubef4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I~/.platformio/packages/framework-stm32cubef4/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I~/.platformio/packages/framework-stm32cubef4/Drivers/CMSIS/Include \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F446RETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# micro-ROS addons
#######################################
LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

# Add micro-ROS utils
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

# Set here the custom transport implementation
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c

print_cflags:
	@echo $(CFLAGS)

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
