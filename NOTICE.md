# Third-Party Dependencies

Source code in this repository is licensed under the MIT License (see `LICENSE`).

The following third‑party components are **not part of this repository**. They are downloaded separately at build time via `make` and are the property of their respective owners, covered by their own licenses:

| Component | License | Owner |
|-----------|---------|-------|
| CMSIS Core headers | Apache 2.0 | Arm Limited |
| STM32F4xx device headers, system and startup files | BSD-3-Clause | STMicroelectronics |
| STM32F401.svd (from stm32-rs) | MIT | stm32-rs contributors |

Full license texts are downloaded to `LICENSES/` when running `make download`.
