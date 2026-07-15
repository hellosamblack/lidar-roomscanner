# vl53l9-common

The `vl53l9-common` component is a set of shared code used across applications. It is mainly composed of driver bindings, platform helpers and utilities.

## platform

The `vl53l9_platform` module exposes platform services such as: 

- Power control (power on, power off, reset)
- Event management (enable, disable, acknowledge)
- Profiling
- Helpers for I2C and I3C interfaces

## driver

The `vl53l9_platform` module contains bindings for the `vl53l9` driver that enable read/write operations for register level access.

## device

The `vl53l9_device` module stores device descriptors for sensor instances.

The `vl53l9_device_t` object is meant to store the hardware configuration elements such as:

- Bus description: handle, type (i.e. i2c/i3c), default address
- Power voltage
- Clock frequency 
- GPIO pin mapping: interrupt, shutdown, synchronization 

When updating the peripheral configuration on the STM32 or using custom hardware, the device descriptors should be adapted to match your setup.

## utils

The `vl53l9_utils` module implements utility helpers that allow to:

- Configure sensor use cases (pre-defined ranging profiles configurations)
- Compute output resolution
- Extract metadata from raw buffers
