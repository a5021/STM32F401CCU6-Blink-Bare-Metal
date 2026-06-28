# STM32F401CCU6-Blink-Bare-Metal

[![Build](https://github.com/a5021/STM32F401CCU6-Blink-Bare-Metal/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/STM32F401CCU6-Blink-Bare-Metal/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM32F401CCU6-00A9E0)]() [![Core](https://img.shields.io/badge/Core-Cortex--M4-00A9E0)]() [![License](https://img.shields.io/badge/License-MIT-yellow)]()

RTC-wakeup blink demo for STM32F401CCU6. On each RTC wakeup event it flashes the PC13 LED three times rapidly and prints the current date and time via USART6 at 115200 baud. Register-level, no HAL.

<p align="center">
  <img src="https://i.postimg.cc/XNTZF1v4/IMG-20191004-183423-cr.jpg">
</p>

## Features

- Register-level, bare-metal firmware (no HAL, no CMSIS-DSP)
- System clock 100 MHz from HSE 25 MHz via PLL (25 / 15 x 120 / 2)
- RTC with LSE oscillator, wakeup timer period ~16 s
- Three fast LED flashes (30 ms each toggle) on each RTC wakeup
- USART6 output: date and time in `DD.MM.YYYY HH:MM:SS` format at 115200 baud
- Instruction & data cache enabled, prefetch active, 2 wait states
- Sleep between wakeup events

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | STM32F401CCU6 (Cortex-M4, 256 KB Flash, 64 KB RAM) |
| Clock | HSE 25 MHz -> PLL -> 100 MHz SYSCLK |
| Cache | 2 WS, ICache + DCache + prefetch |
| LED | PC13 (on-board), push-pull output |
| RTC | LSE 32768 Hz, wakeup timer |
| UART | USART6 on PA11, 115200 baud, 8N1 |
| Debug | SWD on PA13/PA14 |

## Build (Makefile)

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc` 13.x or later)
- GNU Make
- curl, unzip

### Build

```sh
make
```

All third-party files (CMSIS headers, SVD, IDE startup files) are downloaded automatically at build time.

Output in `build/`: `project.elf`, `project.hex`, `project.bin`.

### Targets

| Target | Description |
|--------|-------------|
| `make` | Download deps + build |
| `make download_cmsis` | Download CMSIS core + STM32F4xx headers |
| `make download_svd` | Download STM32F401.svd |
| `make download_iar_startup` | Download IAR startup file |
| `make download_mdk_startup` | Download MDK-ARM startup file |
| `make download` | Download all of the above |
| `make program` | Flash via ST-LINK |
| `make jprogram` | Flash via J-Link |
| `make clean` | Remove build artifacts |
| `make clean_all` | Remove build artifacts + all downloaded files |

## Build (IDE)

Each IDE folder has a README with setup and CLI build instructions.

| IDE | Project | CLI Tool |
|-----|---------|----------|
| SEGGER Embedded Studio | `ide/SES/Project.emProject` | `emBuild.exe` |
| Keil MDK-ARM | `ide/MDK-ARM/Project.uvprojx` | `UV4.exe` |
| IAR EWARM | `ide/EWARM/Project.eww` | `IarBuild.exe` |

Run `make download` from the project root before opening the IDE to fetch dependencies.

## Project Structure

```
.
+-- main.c                        Application + init
+-- main.h                        Macros, clock tree, GPIO helpers
+-- crt0.c                        __libc_init_array stub (SES bundled GCC)
+-- Makefile                       GCC build + download targets
+-- STM32F401CCUX_FLASH.ld        GNU linker script
+-- LICENSE                        MIT License
+-- NOTICE.md                      Third-party licenses
+-- stm32f401cc.jflash            J-Flash project
+-- project.jdebug                 J-Link debugger project
+-- CMSIS/                         Downloaded at build time (gitignored)
+-- ide/
    +-- EWARM/                     IAR Embedded Workbench project
    +-- MDK-ARM/                   Keil MDK-ARM project
    +-- SES/                       SEGGER Embedded Studio project
```

## Clock Tree

```
HSE 25 MHz
  |
  v
PLLM = 15   25 / 15 = 1.667 MHz (VCO input)
PLLN = 120  1.667 x 120 = 200 MHz (VCO output)
PLLP = /2   200 / 2 = 100 MHz
  |
  v
SYSCLK = 100 MHz
  |
  +-- AHB1 (HPRE = /1) -> HCLK = 100 MHz
  |     +-- APB1 (PPRE1 = /2) -> PCLK1 = 50 MHz
  |     +-- APB2 (PPRE2 = /1) -> PCLK2 = 100 MHz
  |     +-- SysTick timer (AHB/8 = 12.5 MHz)
  |
  +-- FLASH: 2 WS, ICache + DCache + Prefetch ON
```

## RTC Configuration

| Parameter | Value |
|-----------|-------|
| Clock source | LSE (32768 Hz) |
| Asynchronous prescaler (PREDIV_A) | 128 |
| Synchronous prescaler (PREDIV_S) | 256 |
| Wakeup timer reload (WUTR) | 2047 |
| Wakeup interval | ~16 seconds |

## Firmware Flow

```
Reset -> main()
  |
  init_sys()         RCC: HSE -> PLL -> 100 MHz
  |                  FLASH: 2 WS, cache + prefetch
  |                  PWR: enable backup domain
  |                  RCC: LSE ON, RTC clock = LSE
  |                  RTC: init time/date, wakeup timer
  |                  SysTick: start (for DELAY_MS)
  |
  init_gpio()        GPIOA: USART6 TX (AF8)
  |                  GPIOC: PC13 output (LED)
  |
  init_usart(115200) USART6: 115200 baud, TX only
  |
  loop:
    wait RTC wakeup flag (WUTF)
    3 fast flashes (PC13 toggle, 30 ms each)
    print_date() -> USART6: "DD.MM.YYYY HH:MM:SS\n"
```

## License

Source code is MIT (see `LICENSE`). Third-party components downloaded at build time are covered by their own licenses (see `NOTICE.md`).
