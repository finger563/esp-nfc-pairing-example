# ESP NFC Out of Band (OOB) BLE Pairing Example

Example application for ESP32S3 to demonstrate NFC OOB BLE pairing with an Android
phone.

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules git@github.com:finger563/esp-nfc-pairing-example
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

Example screenshot of the console output from this app:

![CleanShot 2023-07-14 at 12 17 21](https://github.com/finger563/esp-nfc-pairing-example/assets/213467/9b12ebcb-d4d4-434c-a68b-882cc19adbbb)

https://github.com/finger563/esp-nfc-pairing-example/assets/213467/f97294bb-5178-45f0-a895-a1cf329592fa
