# STM32F401CCU6-Blink-Bare-Metal

[![Build](https://github.com/a5021/STM32F401CCU6-Blink-Bare-Metal/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/STM32F401CCU6-Blink-Bare-Metal/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM32F401CCU6-00A9E0)]() [![Core](https://img.shields.io/badge/Core-Cortex--M4F-00A9E0)]() [![License](https://img.shields.io/badge/License-MIT-yellow)]()

RTC-wakeup blink demo for STM32F401CCU6. On each RTC wakeup event it flashes the PC13 LED three times rapidly and prints the current date and time via USART6 at 115200 baud. Register-level, no HAL.

<p align="center">
  <img src="https://i.postimg.cc/XNTZF1v4/IMG-20191004-183423-cr.jpg">
</p>

## Features

- Register-level, bare-metal firmware (no HAL, no CMSIS-DSP)
- System clock 100 MHz from HSE 25 MHz via PLL (25 / 15 x 120 / 2)
- RTC with LSE oscillator, wakeup timer period 2048 cycles
- Three fast LED flashes (30 ms each toggle) on each RTC wakeup
- USART6 output: date and time in `DD.MM.YYYY HH:MM:SS` format at 115200 baud
- Instruction & data cache enabled, prefetch active, 2 wait states
- Sleep between wakeup events (WFI equivalent via polling)

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | STMicroelectronics STM32F401CCU6 (ARM Cortex-M4F, 256 KB Flash, 64 KB RAM) |
| Clock | HSE 25 MHz → PLL → 100 MHz SYSCLK |
| Cache | 2 WS, ICache + DCache + prefetch enabled |
| LED | PC13 (on-board), push-pull output |
| RTC | LSE 32768 Hz, wakeup timer |
| UART | USART6 on PA11, 115200 baud, 8N1 |
| Debug | SWD on PA13/PA14 |

## Pin Assignment

| Signal | Pin | Peripheral | Notes |
|--------|-----|------------|-------|
| LED | PC13 | GPIO output | On-board LED, push-pull |
| USART6 TX | PA11 | AF8 | 115200 baud |
| SWCLK | PA14 | AF0 | SWD |
| SWDIO | PA13 | AF0 | SWD |

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
  +-- AHB1 (HPRE = /1) → HCLK = 100 MHz
  |     +-- APB1 (PPRE1 = /2) → PCLK1 = 50 MHz
  |     +-- APB2 (PPRE2 = /1) → PCLK2 = 100 MHz
  |     +-- SysTick timer (AHB/8 = 12.5 MHz)
  |
  +-- FLASH: 2 WS, ICache + DCache + Prefetch ON
```

## Firmware Architecture

```
  Reset
    |
  main()
    |
  init_sys()         RCC: HSE → PLL → 100 MHz SYSCLK
    |                FLASH: 2 WS, cache + prefetch
    |                PWR: enable backup domain
    |                RCC: LSE ON, RTC clock = LSE
    |                RTC: init time/date, wakeup timer
    |                SysTick: start (used for DELAY_MS)
    |
  init_gpio()        GPIOA: USART6 TX (AF8), PA0 output, PA1 input
    |                GPIOC: PC13 output (LED)
    |
  init_usart(115200) USART6: 115200 baud, TX only
    |
  main loop
    |
  +-- wait RTC wakeup flag (WUTF)
  |     |
  |     +-- 3 fast flashes (PC13 toggle, 30 ms each)
  |     |
  |     +-- print_date() → USART6: "DD.MM.YYYY HH:MM:SS\n"
  |
  (repeat)
```

## RTC Configuration

| Parameter | Value |
|-----------|-------|
| Clock source | LSE (32768 Hz) |
| Asynchronous prescaler (PREDIV_A) | 128 |
| Synchronous prescaler (PREDIV_S) | 256 |
| Wakeup timer reload (WUTR) | 2047 |
| Wakeup interval | ~16 seconds |

## Getting Started

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- GNU Make

### Build

```sh
make
```

Output in `build/`: `project.elf`, `project.hex`, `project.bin`.

### Flash

```sh
make program       # ST-LINK
make jprogram       # J-Link
```

## Project Structure

```
src/
+-- main.c                       Application + RTC + GPIO + USART init
+-- system_stm32f4xx.c           CMSIS system initialisation
+-- startup_stm32f401xc.s        GCC startup
inc/
+-- main.h                       Macros, clock tree, GPIO helpers
+-- cmsis_*.h, core_cm4.h        CMSIS-CORE headers
+-- stm32f401xc.h, stm32f4xx.h   MCU headers
+-- system_stm32f4xx.h           System header
Makefile                         GCC build system
STM32F401CCUX_FLASH.ld           Linker script
stm32f401cc.jflash               J-Flash project
project.jdebug                   J-Link debugger project
ide/
+-- EWARM/                       IAR Embedded Workbench project
+-- MDK-ARM/                     Keil uVision project
+-- SES/                         Segger Embedded Studio project
