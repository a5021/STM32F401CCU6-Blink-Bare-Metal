TARGET = project
BUILD_DIR = build

ifeq (0, 1)
  ifeq ($(MAKECMDGOALS), debug)
    BUILD_DIR = build-debug
  else
    BUILD_DIR = build-release
  endif
endif

SRC = ./src/main.c ./src/system_stm32f4xx.c
ASM = ./src/startup_stm32f401xc.s
LDS = STM32F401CCUX_FLASH.ld
MCU = -mcpu=cortex-m4 -mthumb
DEF = -DSTM32F401xC
INC = -I./inc -I./inc/CMSIS
OPT = -O3 -g0 -flto

ifdef GCC_PATH
  TOOLCHAIN = $(GCC_PATH)/arm-none-eabi-
else
  TOOLCHAIN = arm-none-eabi-
endif

CC = $(TOOLCHAIN)gcc
AS = $(TOOLCHAIN)gcc -x assembler-with-cpp
CP = $(TOOLCHAIN)objcopy
SZ = $(TOOLCHAIN)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

FLAG = $(MCU) $(DEF) $(INC) -Wall -Werror -Wextra -Wpedantic -fdata-sections -ffunction-sections

JLINK_FLAGS = -openprj./stm32f401cc.jflash -open$(BUILD_DIR)/$(TARGET).hex -auto -hide -exit -jflashlog./jflash.log

ifeq ($(OS), Windows_NT)

    FLAG += -D WIN32
    ifeq ($(PROCESSOR_ARCHITEW6432), AMD64)
        FLAG += -D AMD64
    else
        ifeq ($(PROCESSOR_ARCHITECTURE), AMD64)
            FLAG += -D AMD64
        endif
        ifeq ($(PROCESSOR_ARCHITECTURE), x86)
            FLAG += -D IA32
        endif
    endif

    STLINK = ST-LINK_CLI.exe
    STLINK_FLAGS = -c UR -V -P $(BUILD_DIR)/$(TARGET).hex -HardRst -Run

    JLINK = JFlash.exe

else

    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S), Linux)
        FLAG += -D LINUX
    endif
    ifeq ($(UNAME_S), Darwin)
        FLAG += -D OSX
    endif
    UNAME_P := $(shell uname -p)
    ifeq ($(UNAME_P), x86_64)
        FLAG += -D AMD64
    endif
    ifneq ($(filter %86, $(UNAME_P)),)
        FLAG += -D IA32
    endif
    ifneq ($(filter arm%, $(UNAME_P)),)
        FLAG += -D ARM
    endif

    STLINK = st-flash
    STLINK_FLAGS = --reset --format ihex write $(BUILD_DIR)/$(TARGET).hex

    JLINK = JFlashExe

endif

FLAG += -MMD -MP -MF $(@:%.o=%.d)

LIB = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDS) $(LIB) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all:: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJ = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.c=.o)))
vpath %.c $(sort $(dir $(SRC)))

OBJ += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM:.s=.o)))
vpath %.s $(sort $(dir $(ASM)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(FLAG) $(OPT) $(EXT) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(FLAG) $(OPT) $(EXT) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Makefile
	$(CC) $(OBJ) $(LDFLAGS) $(OPT) $(EXT) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

debug:: OPT = -Og -g3 -gdwarf
debug:: FLAG += -DDEBUG
debug:: all

# Display compiler version information.
gccversion::
	@$(CC) --version

# Program the device using st-link.
program:: $(BUILD_DIR)/$(TARGET).hex
	$(STLINK) $(STLINK_FLAGS)

# Program the device using jlink.
jprogram:: $(BUILD_DIR)/$(TARGET).hex
	$(JLINK) $(JLINK_FLAGS)

clean::
	rm -fR $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
