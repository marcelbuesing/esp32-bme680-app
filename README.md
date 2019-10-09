# ESP32-BME680-App
Demonstrate using Rust on an ESP32 to read environmental metrics (temperature, humidity and pressure) using Bosch's BME680 via I2C.

This project is based on the project template for [ctron/rust-esp-container](https://github.com/ctron/rust-esp-container).

Unlike the default template it uses the:

[esp-idf-sys](https://crates.io/crates/esp-idf-sys) crate.

Furthermore this project would not have been possible without the following crates:

- [esp-idf-hal](https://crates.io/crates/esp-idf-hal) crate.
- [esp-idf-alloc](https://crates.io/crates/esp-idf-alloc) crate.

# Building
```
docker run -ti -v $PWD:/home/project:z quay.io/ctron/rust-esp:latest
```

# Flashing
```
sudo esptool.py write_flash 0x10000 build/esp-app.bin
```

or

```
docker run -ti --device=/dev/ttyUSB0 -v $PWD:/home/project:z rust-esp32:latest flash-project
```
