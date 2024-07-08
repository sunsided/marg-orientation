# STM32F3 Discovery Test Data

This directory contains an output stream from the
[`sunsided/stm32f3disco-rust`](https://github.com/sunsided/stm32f3disco-rust) project
using [`serial-sensors-proto`](https://crates.io/crates/serial-sensors-proto) version `0.4.0`.

## CSV files

The CSV files contain the actual data. The important parts are:

* `25-acc-i16-x1.csv`, the LSM303DLHC accelerometer
* `30-mag-i16-x1.csv`, the LSM303DLHC magnetometer
* `106-gyro-i16-x1.csv`, the L3GD20 gyroscope
* `130-temp-i16-x1.csv`, the LSM303DLHC magnetometer's temperature reading
* `206-temp-i16-x1.csv`, the L3GD20's temperature reading

The corresponding `lranges` files contain normalization data and generally resemble
information from the datasheets. As soon as the information was sent by the sensor,
the respective CSV columns were populated.

Fake sensors:

* `1-heading-u8-x1.csv`, is a simple heading derived by `atan2` from the magnetometer

## Raw data

By virtue of the byte stuffing, each message packet is prefixed with `0x04` and ends with `0x00`,
such that the sequence `0004` resembles a synchronization point between two messages.
Since this data was serialized with `serial-sensors-proto` version `0.4.0`, which uses
the "version 1" data format, the sequence can be extended to `000401`; the `01` here
s already part of the new data frame (indicating protocol version `1`). All data is encoded
in little-endian format.
