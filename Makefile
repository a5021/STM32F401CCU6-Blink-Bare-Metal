TARGET = project
BUILD_DIR = build
SDK_DIR = CMSIS

CMSIS_CORE_RAW    = https://raw.githubusercontent.com/ARM-software/CMSIS_5/master/CMSIS/Core/Include
CMSIS_DEVICE_RAW  = https://raw.githubusercontent.com/STMicroelectronics/cmsis_device_f4/master
SVD_ZIP_URL       = https://raw.githubusercontent.com/stm32-rs/stm32-rs/master/svd/vendor/en.stm32f4-svd.zip
SVD_FILE          = STM32F401.svd
LICENSES_DIR      = LICENSES
CMSIS_LICENSE_URL = https://raw.githubusercontent.com/ARM-software/CMSIS_5/master/LICENSE.txt
ST_LICENSE_URL    = https://raw.githubusercontent.com/STMicroelectronics/cmsis_device_f4/master/LICENSE.md

CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

MCU = -mcpu=cortex-m4 -mthumb
DEF = -DSTM32F401xC
INC = -I$(SDK_DIR)
OPT = -O3 -g0 -flto

ifdef GCC_PATH
  TOOLCHAIN = $(GCC_PATH)/arm-none-eabi-
else
  TOOLCHAIN = arm-none-eabi-
endif

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
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIB) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

SRC = main.c crt0.c $(SDK_DIR)/system_stm32f4xx.c
ASM = $(SDK_DIR)/startup_stm32f401xc.s

all: deps $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

CMSIS_CORE_FILES = core_cm4.h cmsis_version.h cmsis_compiler.h cmsis_gcc.h mpu_armv7.h
CMSIS_DEVICE_FILES = stm32f4xx.h stm32f401xc.h system_stm32f4xx.h system_stm32f4xx.c startup_stm32f401xc.s
IAR_DIR            = ide/EWARM
IAR_STARTUP        = $(IAR_DIR)/startup_stm32f401xc.s
MDK_DIR            = ide/MDK-ARM
MDK_STARTUP        = $(MDK_DIR)/startup_stm32f401xc.s

.PHONY: deps download download_cmsis download_svd download_iar_startup download_mdk_startup download_licenses

download_cmsis: | $(SDK_DIR)
	@for f in $(CMSIS_CORE_FILES); do \
	  if [ ! -f "$(SDK_DIR)/$$f" ]; then \
		echo "  Downloading: $$f"; \
		$(CURL) -sSL -o "$(SDK_DIR)/$$f" "$(CMSIS_CORE_RAW)/$$f"; \
	  fi \
	done
	@for f in $(CMSIS_DEVICE_FILES); do \
	  case "$$f" in \
	    system_stm32f4xx.c)   p="Source/Templates/$$f" ;; \
	    startup_stm32f401xc.s) p="Source/Templates/gcc/$$f" ;; \
	    *)                    p="Include/$$f" ;; \
	  esac; \
	  if [ ! -f "$(SDK_DIR)/$$f" ]; then \
		echo "  Downloading: $$f"; \
		$(CURL) -sSL -o "$(SDK_DIR)/$$f" "$(CMSIS_DEVICE_RAW)/$$p"; \
	  fi \
	done

download_svd:
	@if [ ! -f "$(SVD_FILE)" ]; then \
	  echo "  Downloading: $(SVD_FILE)"; \
	  $(CURL) -sSL -o "stm32f4_svd.zip" "$(SVD_ZIP_URL)"; \
	  unzip -j -o "stm32f4_svd.zip" "*/$(SVD_FILE)" -d .; \
	  rm -f "stm32f4_svd.zip"; \
	fi

download_iar_startup: | $(IAR_DIR)
	@if [ ! -f "$(IAR_STARTUP)" ]; then \
	  echo "  Downloading: $(IAR_STARTUP)"; \
	  $(CURL) -sSL -o "$(IAR_STARTUP)" "$(CMSIS_DEVICE_RAW)/Source/Templates/iar/startup_stm32f401xc.s"; \
	fi

download_mdk_startup: | $(MDK_DIR)
	@if [ ! -f "$(MDK_STARTUP)" ]; then \
	  echo "  Downloading: $(MDK_STARTUP)"; \
	  $(CURL) -sSL -o "$(MDK_STARTUP)" "$(CMSIS_DEVICE_RAW)/Source/Templates/arm/startup_stm32f401xc.s"; \
	fi

download_licenses: | $(LICENSES_DIR)
	@for pair in \
	  "$(CMSIS_LICENSE_URL) $(LICENSES_DIR)/CMSIS_5_LICENSE.txt" \
	  "$(ST_LICENSE_URL)    $(LICENSES_DIR)/cmsis_device_f4_LICENSE.md"; do \
	  set -- $$pair; url="$$1"; file="$$2"; \
	  if [ ! -f "$$file" ]; then \
		echo "  Downloading: $$file"; \
		$(CURL) -sSL -o "$$file" "$$url"; \
	  fi \
	done

download: download_cmsis download_svd download_iar_startup download_mdk_startup download_licenses

deps: download_cmsis download_svd download_licenses

$(LICENSES_DIR):
	mkdir -p $@

$(SDK_DIR):
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
	rm -fR $(SDK_DIR) $(LICENSES_DIR) && rm -f $(SVD_FILE) && rm -f $(IAR_STARTUP) $(MDK_STARTUP) ide/SES/STM32F401x_Vectors.s ide/SES/STM32F4xx_Startup.s ide/SES/thumb_crt0.s

-include $(wildcard $(BUILD_DIR)/*.d)
