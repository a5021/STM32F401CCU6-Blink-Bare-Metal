# SEGGER Embedded Studio (SES)

## Prerequisites

1. **SEGGER Embedded Studio** (tested with v8.28)
2. **GNU Make** (optional, for downloading dependencies)

## Setup (Dependencies)

Run from the project root to download CMSIS and SVD file:

    make

SES uses its bundled GCC with the **SES Smart Linker** (memory map + placement XML instead of a GNU linker script). This enables automatic section discovery, call‑graph‑based dead code elimination, and link‑time optimization.

No additional startup files are required (the GCC startup is already downloaded via `download_cmsis`).

## Build (Command Line)

    "C:\Program Files\SEGGER Embedded Studio 8.28\bin\emBuild.exe" -config Debug   ide/SES/Project.emProject
    "C:\Program Files\SEGGER Embedded Studio 8.28\bin\emBuild.exe" -config Release ide/SES/Project.emProject

Adjust the SES path to match your installed version.

## Build (GUI)

Open `ide/SES/Project.emProject` in SEGGER Embedded Studio and build (Debug or Release).

> **Note**: SES uses its bundled toolchain (no newlib). An empty `__libc_init_array` stub is provided in `crt0.c` at the project root.
