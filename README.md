# ECE-5780-Project-GBJCTS: Simple Quadcopter
This is the final project for an embedded systems design course.

This project involves developing a simple quadcopter to demonstrate proficiency in communication protocols, interrupts, system control, and sensor integration.

Weekly Milestones:
1. Development Environment & Hardware Setup
2. IMU Sensor Interface
3. Altitude Sensor Interface
4. Radio Receiver Interface
5. ESC Control and Motor PWM Output
6. Feedback Control Implementation
7. Final System Integration & Flight Testing


## Hardware Components (tentative)
**Flight Controller:** STM32F072 Discovery Board  
**IMU:** LSM6DS3 on a NOYITO breakout board  
**Altitude Sensor:** VL53L1X on an Adafruit breakout board  
**Radio:** Team BlackSheep Crossfire Nano  
<!-- **SBUS Inverter:** 74HC14   -->
**ESCs:** XILO 40A BLHeli_S ESC  
**Motors:** EMAX ECO II 2207 2400KV  
**Power:** 4S LiPo battery  
**BEC:** MatekSys Micro BEC  

**Quadcopter Frame:** GEPRC GEP-Mark4 5"  
**Propellers:** HQProp Ethix S3  


## Features
### Current
- none implemented

### Planned
- radio controlled thrust input (altitude control)
- stable flight (pitch and roll axes)
- fixed altitude hover


## Toolchain Setup
This project uses a bare-metal embedded toolchain based on GCC, Make, and OpenOCD.

### Required Tools
- **GNU ARM Embedded Toolchain** (compiler)  
- **Make** (build system)  
- **OpenOCD** (flashing/debugging)  
- **ST-Link** (hardware debug probe)
<!-- - **STM32CubeMX**: [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)   -->   

<!-- ### Recommended
- **Visual Studio Code:** [Download here](https://code.visualstudio.com/)  
- **VSCode Extensions:**  
   - [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)  -->

---

### macOS
Install Homebrew if needed: https://brew.sh/

```bash
brew install --cask gcc-arm-embedded
brew install openocd stlink make
```

### Linux
```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi make openocd stlink-tools
```

### Windows
Recommended to use WSL. Install Windows Subsystem for Linux and use the Linux instructions above.  
Setup guide: https://learn.microsoft.com/en-us/windows/wsl/install


## Project Info
**Status:** In Progress  
**Language:** C  
**License:** MIT License – see [LICENSE](./LICENSE)  
**Authors:** [G. Bostram](https://github.com/GunnarBostrom), [J. Canada](https://github.com/JC919), [T. Stratton](https://github.com/POACH3)  
**Semester:** Spring 2026  
**Start Date:** 17-MAR-2026