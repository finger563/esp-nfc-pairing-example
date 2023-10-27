# ESP NFC Out of Band (OOB) BLE Pairing Example

Example application for ESP32S3 to demonstrate NFC OOB BLE pairing with an Android
phone.

For information regarding BLE OOB Secure Simple Pairing, see section 3.2 of
[this
pdf](https://members.nfc-forum.org/apps/group_public/download.php/18688/NFCForum-AD-BTSSP_1_1.pdf)

For more information, see [this ST25DV OOB Pairing
PDF](https://www.st.com/resource/en/user_manual/um2710-st25dvi2c-outofband-pairing-demonstration-stmicroelectronics.pdf)

https://github.com/finger563/esp-nfc-pairing-example/assets/213467/12bece04-84c9-477d-a241-98e56c582306

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

![CleanShot 2023-10-26 at 15 42 30](https://github.com/finger563/esp-nfc-pairing-example/assets/213467/5e4a26da-7d9a-4525-883b-52887a9f283b)

![image](https://github.com/finger563/esp-nfc-pairing-example/assets/213467/0adadfd9-1756-4004-962d-7352b26aaeb2)

![image](https://github.com/finger563/esp-nfc-pairing-example/assets/213467/316519a2-39ac-4a54-805c-2039f784dfa5)

https://github.com/finger563/esp-nfc-pairing-example/assets/213467/12bece04-84c9-477d-a241-98e56c582306
