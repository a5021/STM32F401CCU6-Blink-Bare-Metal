# IAR Embedded Workbench (EWARM)

## Prerequisites

1. **IAR Embedded Workbench for ARM** (tested with v9.x)
2. **GNU Make** (optional, for downloading dependencies)

## Setup

Run from the project root **before opening the project** in IAR EWARM:

    make download_cmsis download_iar_startup

This downloads CMSIS headers and the IAR-specific startup file into `CMSIS/` and `ide/EWARM/`.

## Build (Command Line)

    "C:\Program Files\IAR Systems\Embedded Workbench 9.x\common\bin\IarBuild.exe" ide/EWARM/STM32F401CC.ewp -build "STM32F401CCU6 Blink Bare Metal"

Adjust the IAR path to match your installed version.

## Build (GUI)

Open `ide/EWARM/Project.eww` in IAR Embedded Workbench and build.
