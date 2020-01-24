
## Configurations for Build ##

PROJECT = ML_IOT_FreeRTOS
PROJECT_DIR = .

TOOLCHAIN = /home/hs/embedded/toolchain/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi
LIB_PATH = /home/hs/embedded/toolchain/gcc-arm-none-eabi-7-2018-q2-update/lib/gcc/arm-none-eabi/7.3.1/

# Library Configurations
USE_NANO = 1
USE_STDLIB = 1
DEFAULT_NEWLIB_SYSCALL = 1

# C Standard Configurarion
C_STANDARD = c11
CXX_STANDARD = c++17

# DSP and NN library Configuration, For NN DSP should be enabled
USE_DSP_LIB = 1
USE_CMSIS_NN = 1


# FreeRTOS Plus library API selection
USE_FREERTOS_TCP = 1


BUILD_DIR = $(PROJECT_DIR)/DEBUG
BUILD_OUTPUT = $(PROJECT_DIR)/ELF
LINKER_SCRIPT = $(PROJECT_DIR)/linker/linker_script.ld


# JLink Configurations
DEVICE = XMC4500-1024
INTERFACE = swd
SPEED = 4000


RM = rm -rf

CC = $(TOOLCHAIN)-gcc
CXX = $(TOOLCHAIN)-g++
AS = $(TOOLCHAIN)-as
OBJCPY = $(TOOLCHAIN)-objcopy
OBJDUMP = $(TOOLCHAIN)-objdump
SIZE = $(TOOLCHAIN)-size
LD = $(TOOLCHAIN)-ld
GDB = $(TOOLCHAIN)-gdb


# Processor Flags
CORTEX_CORE = m4
CORTEX_MODE = -mthumb
FPU = hard


# Compilation Flags

HEADERS = $(addprefix -I, $(INCLUDES_DIR))

ARCH_FLAGS = -mfloat-abi=$(FPU) -mfpu=fpv4-sp-d16 -mcpu=cortex-$(CORTEX_CORE) $(CORTEX_MODE)
LIBS = -lc -lm #-lg -lstdc++

AS_FLAGS += $(ARCH_FLAGS)

PREPROCESSORS_FLAGS = -DXMC4500_F100x1024 -D__FPU_PRESENT=1 -D__MPU_PRESENT=1 -DXMC_ETH_PHY_KSZ8081RNB

CC_OPTIMIZATIONS = -O1 -ffunction-sections -fdata-sections
CXX_OPTIMIZATIONS = -O1 -ffunction-sections -fdata-sections -finline-small-functions -findirect-inlining

CC_WARNINGS = -Wall #-Werror
CXX_WARNINGS = -Wall

CC_MISC = -ffast-math -funsigned-bitfields
CXX_MISC = -ffast-math -fno-exceptions -fno-rtti -funsigned-bitfields
C_FLAGS += -c -g -gdwarf-2 $(PREPROCESSORS_FLAGS) #-MMD -MP

CFLAGS += -std=$(C_STANDARD) $(CC_OPTIMIZATIONS) $(CC_WARNINGS) $(CC_MISC) $(ARCH_FLAGS) $(HEADERS)
CXX_FLAGS += -std=$(CXX_STANDARD) $(CxXX_OPTIMIZATIONS) $(CXX_WARNINGS) $(CXX_MISC) $(ARCH_FLAGS) $(HEADERS)


# Library inclusion for linking

LD_FLAGS += $(ARCH_FLAGS)


# Standard library Configurations
ifeq ($(USE_NANO), 1)
LD_FLAGS += -specs=nano.specs 
endif

ifeq ($(USE_STDLIB), 0)
LD_FLAGS += -nodefaultlibs -nostdlib
endif

ifeq ($(DEFAULT_NEWLIB_SYSCALL), 1)
LD_FLAGS += -specs=nosys.specs
endif


LD_FLAGS += -nostartfiles -Xlinker --gc-sections
LD_FLAGS += -fno-non-call-exceptions -fno-use-cxa-atexit -fno-common -ffreestanding
LD_FLAGS += -L "gcc-arm-none-eabi-7-2018-q2-update/lib/gcc/arm-none-eabi/7.3.1/thumb/v7e-m/fpv4-sp/hard/" -lgcc 
LD_FLAGS += -L "gcc-arm-none-eabi-7-2018-q2-update/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard/" $(LIBS)

ifeq ($(USE_DSP_LIB), 1)
LD_FLAGS += -static -L "$(PROJECT_DIR)/Libraries/CMSIS/DSP" -larm_cortexM4lf_math
#LD_FLAGS += -Wl,--whole-archive $(PROJECT_DIR)/Libraries/CMSIS/DSP/libarm_cortexM4lf_math.a
endif


# Include build file
-include build.mk


.PHONY: all clean flash


SECONDARY_FLASH = $(BUILD_OUTPUT)/$(PROJECT).hex
SECONDARY_LIST = $(BUILD_OUTPUT)/$(PROJECT).lst
SECONDARY_IMAGE = $(BUILD_OUTPUT)/$(PROJECT).img
SECONDARY_SYMBOLS = $(BUILD_OUTPUT)/$(PROJECT).syms
SECONDARY_SIZE = $(PROJECT).siz


# Targets #
all: $(BUILD_OUTPUT)/$(PROJECT).elf secondary-outputs
	@echo 'Finished Building.'


$(BUILD_OUTPUT)/$(PROJECT).elf: $(OBJECTS)
	mkdir -p $(dir $@)
	@echo 'Building target: $@'
	@echo 'Invoking linker..'
	$(CXX) $^ -Wl,-Map,$(BUILD_OUTPUT)/$(PROJECT).map -o $@ -T $(LINKER_SCRIPT) $(LD_FLAGS)
	@echo 'Finshed Building target: $@'
	@echo '.'


$(BUILD_OUTPUT)/$(PROJECT).syms: $(BUILD_OUTPUT)/$(PROJECT).elf
	@echo 'Generating symbols..'
	$(OBJDUMP) -t $< > $@
	@echo '.'


$(BUILD_OUTPUT)/$(PROJECT).lst: $(BUILD_OUTPUT)/$(PROJECT).elf
	@echo 'Invoking list generation..'
	$(OBJDUMP) -D -S $< > $@
	@echo '.'


$(BUILD_OUTPUT)/$(PROJECT).hex: $(BUILD_OUTPUT)/$(PROJECT).elf
	echo 'Invoking hex generation..'
	$(OBJCPY) -O ihex $< $@
	@echo '.'


$(BUILD_OUTPUT)/$(PROJECT).img: $(BUILD_OUTPUT)/$(PROJECT).elf
	@echo 'Invoking Image creation'
	$(OBJCPY) -O binary $< $@


$(PROJECT).siz: $(BUILD_OUTPUT)/$(PROJECT).elf
	@echo 'Generating size..'
	$(SIZE) --format=berkley $<
	@echo '.'


flash: $(BUILD_OUTPUT)/$(PROJECT).hex
	JLinkExe -if $(INTERFACE) -device $(DEVICE) -speed $(SPEED)


qemu: $(BUILD_OUTPUT)/$(PROJECT).img
	@echo 'Invoking Image creation'
	$(GDB) $(BUILD_OUTPUT)/$(PROJECT).elf


clean:
	$(RM) $(BUILD_DIR) $(BUILD_OUTPUT)


secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_IMAGE) $(SECONDARY_LIST) $(SECONDARY_SYMBOLS) $(SECONDARY_SIZE)


# End of Makefile


