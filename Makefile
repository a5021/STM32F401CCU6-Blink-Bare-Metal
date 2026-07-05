TARGET = project
BUILD_DIR = build
CMSIS_CORE_DIR   = CMSIS/core
CMSIS_DEVICE_DIR = CMSIS/device

CMSIS_CORE_RAW    = https://raw.githubusercontent.com/ARM-software/CMSIS_5/master/CMSIS/Core/Include
CMSIS_DEVICE_RAW  = https://raw.githubusercontent.com/STMicroelectronics/cmsis_device_f4/master
SVD_ZIP_URL       = https://raw.githubusercontent.com/stm32-rs/stm32-rs/master/svd/vendor/en.stm32f4-svd.zip
SVD_FILE          = STM32F401.svd
CMSIS_LICENSE_URL = https://raw.githubusercontent.com/ARM-software/CMSIS_5/master/LICENSE.txt
ST_LICENSE_URL    = https://raw.githubusercontent.com/STMicroelectronics/cmsis_device_f4/master/LICENSE.md

ifdef GCC_PATH
  TOOLCHAIN = $(GCC_PATH)/arm-none-eabi-
else
  TOOLCHAIN = arm-none-eabi-
endif

CC = $(TOOLCHAIN)gcc
AS = $(TOOLCHAIN)gcc -x assembler-with-cpp
CP = $(TOOLCHAIN)objcopy
SZ = $(TOOLCHAIN)size

MCU = -mcpu=cortex-m4 -mthumb
DEF = -DSTM32F401xC
INC = -I$(CMSIS_CORE_DIR) -I$(CMSIS_DEVICE_DIR)
OPT = -O3 -g0 -flto

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

FLAG = $(MCU) $(DEF) $(INC) -Wall -Werror -Wextra -Wpedantic -fdata-sections -ffunction-sections

JLINK_FLAGS = -openprj./stm32f401cc.jflash -open$(BUILD_DIR)/$(TARGET).hex -auto -hide -exit -jflashlog./jflash.log

ifeq ($(OS), Windows_NT)
  FLAG += -D WIN32
  CURL = curl.exe
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
  CURL = curl
  STLINK = st-flash
  STLINK_FLAGS = --reset --format ihex write $(BUILD_DIR)/$(TARGET).hex
  JLINK = JFlashExe
endif

FLAG += -MMD -MP -MF $(@:%.o=%.d)

LDSCRIPT = STM32F401CCUX_FLASH.ld
LIB = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIB) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--no-warn-rwx-segments

SRC = main.c crt0.c $(CMSIS_DEVICE_DIR)/system_stm32f4xx.c
ASM = $(CMSIS_DEVICE_DIR)/startup_stm32f401xc.s

all: deps $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

CMSIS_CORE_FILES = core_cm4.h cmsis_version.h cmsis_compiler.h cmsis_gcc.h cmsis_iccarm.h mpu_armv7.h
CMSIS_DEVICE_FILES = stm32f4xx.h stm32f401xc.h system_stm32f4xx.h system_stm32f4xx.c startup_stm32f401xc.s
IAR_DIR            = ide/EWARM
IAR_STARTUP        = $(IAR_DIR)/startup_stm32f401xc.s
MDK_DIR            = ide/MDK-ARM
MDK_STARTUP        = $(MDK_DIR)/startup_stm32f401xc.s

.PHONY: deps download download_cmsis download_svd download_iar_startup download_mdk_startup download_licenses

download_cmsis: | $(CMSIS_CORE_DIR) $(CMSIS_DEVICE_DIR)
	@for f in $(CMSIS_CORE_FILES); do \
	  if [ ! -f "$(CMSIS_CORE_DIR)/$$f" ]; then \
		echo "  Downloading: $$f"; \
		$(CURL) -fsSL -o "$(CMSIS_CORE_DIR)/$$f" "$(CMSIS_CORE_RAW)/$$f" || exit 1; \
	  fi \
	done
	@for f in $(CMSIS_DEVICE_FILES); do \
	  case "$$f" in \
	    system_stm32f4xx.c)   p="Source/Templates/$$f" ;; \
	    startup_stm32f401xc.s) p="Source/Templates/gcc/$$f" ;; \
	    *)                    p="Include/$$f" ;; \
	  esac; \
	  if [ ! -f "$(CMSIS_DEVICE_DIR)/$$f" ]; then \
		echo "  Downloading: $$f"; \
		$(CURL) -fsSL -o "$(CMSIS_DEVICE_DIR)/$$f" "$(CMSIS_DEVICE_RAW)/$$p" || exit 1; \
	  fi \
	done

download_svd:
	@if [ ! -f "$(SVD_FILE)" ]; then \
	  echo "  Downloading: $(SVD_FILE)"; \
	  $(CURL) -fsSL -o "stm32f4_svd.zip" "$(SVD_ZIP_URL)" || exit 1; \
	  unzip -j -o "stm32f4_svd.zip" "*/$(SVD_FILE)" -d . || exit 1; \
	  rm -f "stm32f4_svd.zip"; \
	fi

download_iar_startup: | $(IAR_DIR)
	@if [ ! -f "$(IAR_STARTUP)" ]; then \
	  echo "  Downloading: $(IAR_STARTUP)"; \
	  $(CURL) -fsSL -o "$(IAR_STARTUP)" "$(CMSIS_DEVICE_RAW)/Source/Templates/iar/startup_stm32f401xc.s" || exit 1; \
	fi

download_mdk_startup: | $(MDK_DIR)
	@if [ ! -f "$(MDK_STARTUP)" ]; then \
	  echo "  Downloading: $(MDK_STARTUP)"; \
	  $(CURL) -fsSL -o "$(MDK_STARTUP)" "$(CMSIS_DEVICE_RAW)/Source/Templates/arm/startup_stm32f401xc.s" || exit 1; \
	fi

download_licenses:
	@for pair in \
	  "$(CMSIS_LICENSE_URL) $(CMSIS_CORE_DIR)/LICENSE.txt" \
	  "$(ST_LICENSE_URL)    $(CMSIS_DEVICE_DIR)/LICENSE.md"; do \
	  set -- $$pair; url="$$1"; file="$$2"; \
	  if [ ! -f "$$file" ]; then \
		echo "  Downloading: $$file"; \
		$(CURL) -fsSL -o "$$file" "$$url" || exit 1; \
	  fi \
	done

download: download_cmsis download_svd download_iar_startup download_mdk_startup download_licenses

deps: download_cmsis download_svd download_licenses

# Pattern rules: if make needs a CMSIS file that doesn't exist, download it on demand.
# This allows `make all` to work on a fresh checkout without a separate `make deps` step.
$(CMSIS_CORE_DIR)/%: | $(CMSIS_CORE_DIR)
	$(CURL) -fsSL -o "$@" "$(CMSIS_CORE_RAW)/$*"

$(CMSIS_DEVICE_DIR)/%: | $(CMSIS_DEVICE_DIR)
	@name="$*"; \
	  case "$$name" in \
	    system_stm32f4xx.c)   url="$(CMSIS_DEVICE_RAW)/Source/Templates/$$name" ;; \
	    startup_stm32f401xc.s) url="$(CMSIS_DEVICE_RAW)/Source/Templates/gcc/$$name" ;; \
	    *)                    url="$(CMSIS_DEVICE_RAW)/Include/$$name" ;; \
	  esac; \
	  $(CURL) -fsSL -o "$@" "$$url"

$(CMSIS_CORE_DIR):
	mkdir -p $@

$(CMSIS_DEVICE_DIR):
	mkdir -p $@

$(IAR_DIR):
	mkdir -p $@

$(MDK_DIR):
	mkdir -p $@

OBJ = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.c=.o)))
vpath %.c $(sort $(dir $(SRC)))

OBJ += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM:.s=.o)))
vpath %.s $(sort $(dir $(ASM)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(FLAG) $(OPT) $(EXT) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(FLAG) $(OPT) $(EXT) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Makefile
	$(CC) $(OBJ) $(LDFLAGS) $(OPT) $(EXT) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

debug: OPT = -Og -g3 -gdwarf
debug: FLAG += -DDEBUG
debug: all

gccversion:
	@$(CC) --version

program: $(BUILD_DIR)/$(TARGET).hex
	$(STLINK) $(STLINK_FLAGS)

jprogram: $(BUILD_DIR)/$(TARGET).hex
	$(JLINK) $(JLINK_FLAGS)

clean:
	rm -fR $(BUILD_DIR)

clean_all: clean
	rm -fR $(CMSIS_CORE_DIR) $(CMSIS_DEVICE_DIR) && rm -f $(SVD_FILE) && rm -f $(IAR_STARTUP) $(MDK_STARTUP) ide/SES/STM32F401x_Vectors.s ide/SES/STM32F4xx_Startup.s ide/SES/thumb_crt0.s

-include $(wildcard $(BUILD_DIR)/*.d)
