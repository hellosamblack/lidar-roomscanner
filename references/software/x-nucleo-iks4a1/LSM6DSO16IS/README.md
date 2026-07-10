# LSM6DSO16IS
Arduino library to support the LSM6DSO16IS 3D accelerometer and 3D gyroscope with ISPU

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSO16ISSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSO16ISSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

## Examples

* LSM6DSO16IS_DataLog_Terminal: This application shows how to get data from LSM6DSO16IS accelerometer and gyroscope and print them on terminal.

* LSM6DSO16IS_ISPU_Sensor_Fusion: This application implements the sensor fusion of the accelerometer and gyroscope, configured in high-performance mode at 104 Hz. The configuration generates an interrupt on INT1 when the quaternion for the new sample is computed and available in the output registers.

* LSM6DSO16IS_ISPU_Tap: This application implements the tap detection solution based on the accelerometer data. The configuration generates an interrupt on INT1 when the tap event for the new sample is computed and available in the output registers.


## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSO16IS

The LSM6DSO16IS datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dso16is.html

You can find other examples on ISPU at  
https://github.com/STMicroelectronics/ispu-examples/tree/master/ism330is_lsm6dso16is 