# Keil MDK-ARM

## Prerequisites

1. **Keil MDK-ARM** with ARM Compiler (v5/v6)
2. **GNU Make** (optional, for downloading dependencies)

## Setup

Run from the project root:

    make download_cmsis download_mdk_startup

This downloads CMSIS headers and the MDK (ARM) startup file into `CMSIS/` and `ide/MDK-ARM/`.

## Build (Command Line)

    "C:\Keil_v5\UV4\UV4.exe" -b ide/MDK-ARM/Project.uvprojx
    "C:\Keil_v5\UV4\UV4.exe" -r ide/MDK-ARM/Project.uvprojx -j0   # rebuild all

Adjust the Keil path to match your installed version.

> Output: `Objects/Project.hex`, build log: `Objects/Project.build_log.htm`.

## Build (GUI)

Open `ide/MDK-ARM/Project.uvprojx` in Keil μVision and build.
