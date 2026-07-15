## __53L9A1_PostprocessSingle Application Description__

This application demonstrates single-sensor post-processing with the VL53L9CX ToF 3D LiDAR. It captures raw measurement frames from the sensor through I3C, applies the transform pipeline using the device calibration data, and produces a processed depth stream on the STM32 host.

Data resulting from the acquisition is read in an asynchronous way through a DMA allowing to reduce the CPU load. A system of double buffering is used to process the previous frame while the current one is being transmitted.

The example can be customized to output other streams among the available ones. Available controls and and stream capabilities of the postprocessing pipeline are displayed on the serial port at the application startup. You can also refer to the `vl53l9-transform-c` documentation for a complete description of the capabilities of the library.

The measured frame rate is printed on the serial port. Setting the `PRINT_FRAME` flag enables the printing of the frame mapping depth value with ASCII characters. Please note that this affects the performances.

The serial port must be configured with the following settings:
- Baudrate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1


### __Keywords__

ToF, I3C, DMA, VCOM


### __Hardware and Software environment__

  - This example runs on STM32 Nucleo boards with X-NUCLEO-53L9A1 expansion board.

  - This example has been tested with STMicroelectronics NUCLEO-H563ZI evaluation boards and can be easily tailored to any other supported
    device and development board.


### __How to use it?__

In order to make the program work, you must do the following :

 - WARNING: before opening the project with any toolchain be sure your folder installation path is not too in-depth since the toolchain may report errors after building.

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example
 - Alternatively, you can load the pre-built binaries in "Binary" folder included in the distributed package
 - Open the STM32 VCOM port in your preferred serial monitor with the aforementioned settings to see the application's output
