# Science Module Nucleo

By: **York University Rover Team (YURS)** — Lassonde School of Engineering

![Platform](https://img.shields.io/badge/Platform-STM32-blue)
![Language](https://img.shields.io/badge/Language-C%2FC++-orange)
![Communication](https://img.shields.io/badge/Bus-CAN-green)
![System](https://img.shields.io/badge/System-Rover%20Control-lightgrey)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

This repository contains the **embedded firmware** for the **Science Module**, developed by the **York University Rover Team (YURS)**. The firmware targets the **Nucleo-F302R8** and is designed to support modular rover subsystems including servo control, 7-in-1 science sensor, CO2 sensor, pump control, hydrogen sensor and weight cell. The goal of this project is to provide a **a modular platform to process science data** onboard the rover for the science task. By leveraging real time embedded firmware and a distributed **CAN bus network**, the system enables reliable communication between subsystem controllers while maintaining deterministic timing and low power consumption.bedded firmware and a distributed **CAN bus network**, the system enables reliable communication between subsystem controllers while maintaining deterministic timing and low power consumption.

---

## Repository Structure
```
science-module-nucleo/
│
├── firmware/
│ ├── Core/               <- Application source code
│ │ ├── Inc/              <- Header files
│ │ ├── Src/              <- Source files
│ │ └── Startup/          <- Startup code
│ ├── Drivers/            <- STM32 HAL and CMSIS drivers
│ └── nucleo-template.ioc          <- STM32CubeMX project file
│
├── .github/              <- CI/CD workflows
├── .gitignore            <- Git ignore rules
├── README.md             <- Project documentation
├── workflow-guide.md     <- Workflow instructions
└── LICENSE               <- License file
```
