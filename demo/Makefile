
## Configurations for Build ##

PROJECT = Demo
PROJECT_DIR = .

TOOLCHAIN = gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi
LIB_PATH = gcc-arm-none-eabi-7-2018-q2-update/lib/gcc/arm-none-eabi/7.3.1/


USE_NANO = 0
C_STANDARD = c11
CXX_STANDARD = c++17


BUILD_DIR = $(PROJECT_DIR)/DEBUG
LINKER_SCRIPT = linker_script.ld


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
LIBS = -lc -lm -lg -lstdc++

AS_FLAGS += $(ARCH_FLAGS)

PREPROCESSORS_FLAGS = -DXMC4500_F100x1024 -D__FPU_PRESENT -D__MPU_PRESENT

CC_OPTIMIZATIONS = -O0 -ffunction-sections -fdata-sections
CXX_OPTIMIZATIONS = -O0 -ffunction-sections -fdata-sections -finline-small-functions -findirect-inlining

CC_WARNINGS = -Wall -Werror
CXX_WARNINGS = 

CC_MISC = -ffast-math -funsigned-bitfields
CXX_MISC = -ffast-math -fno-exceptions -fno-rtti -funsigned-bitfields
C_FLAGS += -c -g -gdwarf-2 $(PREPROCESSORS_FLAGS) #-MMD -MP

CFLAGS += -std=$(C_STANDARD) $(CC_OPTIMIZATIONS) $(CC_WARNINGS) $(CC_MISC) $(ARCH_FLAGS) $(HEADERS)
CXX_FLAGS += -std=$(CXX_STANDARD) $(CxXX_OPTIMIZATIONS) $(CXX_WARNINGS) $(CXX_MISC) $(ARCH_FLAGS) $(HEADERS)


# Library inclusion for linking

LD_FLAGS += $(ARCH_FLAGS)

# Standard library Nano
ifeq ($(USE_NANO), 1)
LD_FLAGS += -specs=nano.specs -specs=nosys.specs
endif

LD_FLAGS += -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections
LD_FLAGS += -fno-non-call-exceptions -fno-use-cxa-atexit -fno-common -ffreestanding
#LD_FLAGS += -L "gcc-arm-none-eabi-7-2018-q2-update/lib/gcc/arm-none-eabi/7.3.1/thumb/v7e-m/fpv4-sp/hard/" -lgcc 
#LD_FLAGS += -L "gcc-arm-none-eabi-7-2018-q2-update/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard/" $(LIBS)


# Include build file
-include build.mk


.PHONY: all clean flash


SECONDARY_FLASH = $(PROJECT).hex
SECONDARY_LIST = $(PROJECT).lst
SECONDARY_SIZE = $(PROJECT).siz


# Targets #
all: $(PROJECT).elf secondary-outputs
	@echo 'Finished Building.'


$(PROJECT).elf: $(OBJECTS)
	@echo 'Building target: $@'
	@echo 'Invoking linker..'
	$(CXX) $^ -Wl,-Map,$(PROJECT).map -o $@ -T $(LINKER_SCRIPT) $(LD_FLAGS)
	@echo 'Finshed Building target: $@'
	@echo '.'


$(PROJECT).syms: $(PROJECT).elf
	@echo 'Generating symbols..'
	$(OBJDUMP) -t > $@
	@echo '.'


$(PROJECT).lst: $(PROJECT).elf
	@echo 'Invoking list generation..'
	$(OBJDUMP) -D -S $< > $@
	@echo '.'


$(PROJECT).hex: $(PROJECT).elf
	echo 'Invoking hex generation..'
	$(OBJCPY) -O ihex $< $@
	@echo '.'


$(PROJECT).siz: $(PROJECT).elf
	@echo 'Generating size..'
	$(SIZE) --format=berkley $<
	@echo '.'


flash: $(PROJECT).hex
	JLinkExe -if $(INTERFACE) -device $(DEVICE) -speed $(SPEED)


clean:
	$(RM) $(BUILD_DIR) $(PROJECT).elf $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(PROJECT).map $(PROJECT).syms


secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)


