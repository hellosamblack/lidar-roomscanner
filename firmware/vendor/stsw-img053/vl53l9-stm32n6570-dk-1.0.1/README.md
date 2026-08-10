# STM32N6570-DK Firmware for VL53L9

## Disclaimer

> Use of control adjustments, or procedures other than those specified in product datasheet may result in hazardous product behavior.

## Introduction

This package provides a STM32N6570-DK firmware project with example applications showcasing the VL53L9 Time-of-Flight 3D LiDAR sensor. The applications illustrate the complete flow: initializing the platform, configuring the sensor through the driver, retrieving frames, and processing them through the software ISP.


## Package Layout

This release package is a self-contained STM32CubeIDE project. Its content is
listed below.

```text
├── app/
│   ├── simple_ranging_[i3c|csi]/   Basic ranging application
│   └── postprocess_[i3c|csi]/      Extends `simple_ranging` with software ISP and display output
├── drivers/
│   └── vl53l9/                     Core driver exposing functions to configure the sensor and retrieve data
├── interface/
│   ├── platform/                   Platform utilities (bus management, event handling, profiling, display, ...)
│   └── vl53l9/                     Platform operations required by the core driver (read/write, delay, ...)
├── lib/
│   ├── media-object/               Media Object class definitions
│   └── transform/                  VL53L9 postprocessing library (software ISP)
├── platform/
│   ├── Drivers/                    CMSIS + STM32N6xx HAL drivers
│   ├── vl53l9-stm32n6570-dk.ioc    STM32CubeMX project file
│   └── FSBL/                       First Stage Boot Loader
│       ├── Core/                   Peripheral and system initialization (CubeMX-generated)
│       ├── .project / .cproject    STM32CubeIDE project files
│       ├── STM32N657X0HXQ.ld       Linker script
│       └── *.launch                Run/Debug configurations (one per application)
├── CHANGELOG.md                    Revision log
├── LICENSE.md                      License terms
└── README.md                       This file
```


## Applications

| Application | Description |
|-------------|-------------|
| `simple_ranging_[i3c|csi]` | Basic ranging application. Initializes the sensor, applies a ranging profile, triggers frames and retrieves them. Baseline for sensor bring-up. |
| `postprocess_[i3c|csi]` | Extends `simple_ranging` by routing each frame through the VL53L9 software ISP (`transform` library), with the resulting depth image rendered on the on-board LCD. |


## Quickstart with STM32CubeIDE

### Open and import the project

1. Launch STM32CubeIDE and import the project available within the `platform` folder.

    *File -> Import -> Existing Projects into Workspace -> Select root directory -> Browse -> `platform` -> Finish*

2. Only the `FSBL` project is necessary (other projects can be unchecked during import).


### Select active build configuration and build project

1. Choose the application to build from the available build configurations.

    *Project -> Build Configurations -> Set Active -> `<application>`*

2. Build the project with the selected build configuration.

    *Project -> Build Project*


### Run / Debug the firmware

On the STM32N6570-DK, the boot mode is selected with the on-board BOOT0 and BOOT1 switches:

| Boot mode       | BOOT0 | BOOT1 | Description                                          |
| --------------- | :---: | :---: | ---------------------------------------------------- |
| Debug           |   L   |   H   | Development boot, firmware loaded from RAM           |
| Boot from flash |   L   |   L   | Application loaded from the external flash on reset  |

#### Run / Debug from RAM

Set the BOOT1 switch to H to load the firmware from RAM, then:

1. Run the firmware without debug mode.

    *Run -> Run Configurations*

    **OR**

2. Debug the firmware.

    *Run -> Debug Configurations*


#### Boot from flash

> The STM32N6 has no internal user flash, so the firmware is stored in the external flash memory and loaded at startup. On reset, the device's boot ROM reads the signed FSBL image from the external flash and copies it into RAM before executing it, which is why the binary must first be signed and then programmed at the external flash base address (`0x70000000`). This process relies on the STM32CubeProgrammer tools — `STM32_SigningTool_CLI` to sign the binary and `STM32_Programmer_CLI` (together with the matching external loader) to write it to the flash — so make sure STM32CubeProgrammer is installed and available before proceeding.

> Note: the commands below are run from the STM32CubeProgrammer installation folder.

1. Set the BOOT1 switch to H.

2. Sign the binary file produced by the build:

    ```
    ./STM32_SigningTool_CLI -bin "<your_app>.bin" -nk -t fsbl -hv 2.3 -o "<your_app>_trusted.bin" --align
    ```

3. Flash the signed binary file to the external flash:

    ```
    ./STM32_Programmer_CLI -c port=SWD mode=HOTPLUG ap=1 -el ./ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr -hardRst -w "<your_app>_trusted.bin" 0x70000000
    ```

4. Set the BOOT1 switch to L and press the reset button to load the application from the external flash memory.


### Serial output

The firmware reports the postprocessing library capabilities, the frame counter and the measured frame rate over the serial interface.

To view this output, connect a serial terminal to the board and set the baud rate to 921600 bps.
