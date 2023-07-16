TARGET = asac-esc
OPENOCD = /home/victor/opt/openocd-0.12.0/bin/openocd
OPENOCD_TARGET = target/stm32c0x.cfg
OPENOCD_INTERFACE = interface/stlink.cfg

DEBUG = 1
OPT = -O0
#OPT = -O3

BUILD_DIR = build

C_SOURCES =  \
src/main.c \
src/hal.c \
src/state.c \
src/stm32c0xx_it.c \
src/stm32c0xx_hal_msp.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_adc.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_adc_ex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_dma.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_dma_ex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_rcc.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_rcc_ex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_flash.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_flash_ex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_gpio.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_pwr.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_pwr_ex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_cortex.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_uart.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_exti.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_tim.c \
drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_tim_ex.c \
src/system_stm32c0xx.c

# ASM sources
ASM_SOURCES =  \
startup_stm32c011xx.s


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


OBJDUMP = $(PREFIX)objdump

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m0


# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32C011xx


# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-Iinclude \
-Idrivers/STM32C0xx_HAL_Driver/Inc \
-Idrivers/STM32C0xx_HAL_Driver/Inc/Legacy \
-Idrivers/CMSIS/Device/ST/STM32C0xx/Include \
-Idrivers/CMSIS/Include \
-IInc \
-IDrivers/STM32C0xx_HAL_Driver/Inc \
-IDrivers/STM32C0xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32C0xx/Include \
-IDrivers/CMSIS/Include


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
LDSCRIPT = STM32C011F6Px_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
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


flash: all
	${OPENOCD} -f ${OPENOCD_INTERFACE} -f ${OPENOCD_TARGET} -c "program ${BUILD_DIR}/${TARGET}.elf verify reset exit"

disasm:
	${OBJDUMP} $(BUILD_DIR)/$(TARGET).elf -D > DISASM

clean:
	-rm -fR $(BUILD_DIR)


.PHONY = all clean flash

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
