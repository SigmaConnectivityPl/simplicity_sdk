# SoC - CS Initiator

The Bluetooth SoC-CS Initiator is a project that can be used to test the Channel Sounding (CS) feature. In the provided CS setup, the application establishes connection to a reflector (Running on the host or SoC), measures in the phase based ranging (PBR) or round trip time (RTT) CS measurement modes and estimates the distance. Moving object tracking algorithm mode and stationary object tracking algorithm mode are also supported for the measurement process. In moving object tracking mode distance will be calculated for every CS procedure while in stationary object tracking mode the calculation requires multiple CS procedures data. While the measurement is still in progress, the percentage of the progress is displayed on the LCD and logged to the console. Once all required CS procedure data is received for the distance measurement, the estimated result will arrive and it will be displayed on the LCD and logged to the console. In order to select different CS modes (PBR/RTT) or algorithm modes (Moving object tracking/Stationary object tracking) the push button(s) can be pressed during device RESET. The push button BTN0 will change the CS mode and BTN1 will change the object tracking mode to the opposite of the default values (which are stored in the initiator_app_config.h).

## Usage
- Build and flash the sample application.

- Pressing BTN0 while resetting the device selects the CS mode.

- Pressing BTN1 while resetting the device selects Stationary Object tracking algorithm mode.

- If neither BTN0 nor BTN1 pressed while resetting the device default CS mode (PBR) and default algorithm mode (Moving Object tracking) will be applied.

- After startup the CS initiator will scan for a device running the "CS RFLCT" sample application.

- When found, the initiator will establish connection to the CS reflector device and will start the distance measurement process.

- The initiator will calculate the distance, display it on the LCD and also send them via UART. Note: in case of stationary object tracking mode multiple CS procedure data will be required. Until the distance measurement is in progress the progress percentage will be displayed on the LCD and logged to the console.


![](./image/cs_lcd.png)

## Known issues and limitations

* Only one initiator instance created
* In case RTT mode used with stationary object tracking algorithm mode the behavior will be the same as RTT with moving object tracking mode.

## Troubleshooting

This sample application does need a bootloader (Bootloader - SoC Bluetooth AppLoader OTA DFU).

## Resources

[Bluetooth Documentation](https://docs.silabs.com/bluetooth/latest/)

[UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf)

[QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf)

[UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf)

[Bluetooth Training](https://www.silabs.com/support/training/bluetooth)

