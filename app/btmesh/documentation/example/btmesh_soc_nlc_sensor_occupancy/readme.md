# Bluetooth Mesh - NLC Occupancy Sensor

The **Bluetooth Mesh - NLC Occupancy Sensor** example is a working example application that you can use as an Occupancy Sensor node in a Networked Lighting Control (NLC) network.

The example is an out-of-the-box Software Demo where the device acts as an Occupancy Sensor in an NLC network. It measures people count and sends the measurement data to a remote device implementing the Basic Lightness Controller NLC profile (e.g. **Bluetooth Mesh - NLC Basic Lightness Controller**). The current measurement is displayed on the LCD (if one is present on the mainboard) and also sent to UART. The example implements the Bluetooth Mesh Occupancy Sensor NLC profile and is based on the Sensor Server Model. This example requires one of the Internal Storage Bootloader (single image) variants, depending on device memory.

![Bluetooth Mesh sensor system - Server](readme_img7.png)

## Getting Started

To learn Bluetooth mesh technology basics, see [Bluetooth Mesh Network - An Introduction for Developers](https://www.bluetooth.com/wp-content/uploads/2019/03/Mesh-Technology-Overview.pdf).

To get started with Bluetooth Mesh and Simplicity Studio, see [QSG176: Bluetooth Mesh SDK v2.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg176-bluetooth-mesh-sdk-v2x-quick-start-guide.pdf).

The term SoC stands for "System on Chip", meaning that this is a standalone application that runs on the EFR32/BGM and does not require any external MCU or other active components to operate.

**Bluetooth Mesh - NLC Occupancy Sensor** example makes Occupancy Sensor measurements and forwards them to another node, for example an NLC Basic Lightness Controller (**Bluetooth Mesh - NLC Basic Lightness Controller**) or a Sensor Client Node (**Bluetooth Mesh - Sensor Client**). This measurement can be faked by pressing the buttons on the mainboard:

- BTN1 increases people count
- BTN0 decreases people count

In case of only one button, a shorter press increases, while a longer one decreases the people count.

To add or remove features from the example, follow this process:

- Add model and feature components to your project
- Optionally configure your Mesh node through the "Bluetooth Mesh Configurator"

![Bluetooth Mesh Configurator](readme_img1.png)

To learn more about programming an SoC application, see [UG472: Silicon Labs Bluetooth ® Mesh Configurator User's guide for SDK v2.x](https://www.silabs.com/documents/public/user-guides/ug472-bluetooth-mesh-v2x-node-configuration-users-guide.pdf).

- Some components are configurable, and can be customized using the Component Editor

![Bluetooth Mesh Components](readme_img8.png)

- Respond to the events raised by the Bluetooth stack
- Implement additional application logic

[UG295: Silicon Labs Bluetooth ® Mesh C Application Developer's Guide for SDK v2.x](https://www.silabs.com/documents/public/user-guides/ug295-bluetooth-mesh-dev-guide.pdf) gives code-level information on the stack and the common pitfalls to avoid.

## Device Firmware Update

Device Firmware Update (DFU) is a new feature introduced in the Bluetooth Mesh Model specification v1.1 that provides a standard way to update device firmware over a Bluetooth mesh network. The example has the Target node functionality enabled by default that is fulfilled by installing the Firmware Update and BLOB Transfer model components:

- Firmware Update Server
- BLOB Transfer Server

![Bluetooth Mesh Firmware Update Components](readme_img9.png)

![Bluetooth Mesh Transfer Components](readme_img10.png)

For more information on the DFU examples, see **AN1370: Bluetooth® Mesh Device Firmware Update Example Walkthrough**. To learn the basics of the Bluetooth Mesh Device Firmware Update specification, see **AN1319: Bluetooth® Mesh Device Firmware Update**.

## Testing the Bluetooth Mesh - NLC Occupancy Sensor Application

To test the application, do the following:

1. Make sure a bootloader is installed. See the Troubleshooting section.
2. Build and flash the **Bluetooth Mesh - NLC Occupancy Sensor** example to your device.
3. Reset the device by pressing and releasing the reset button on the mainboard while pressing BTN0. The message "Factory reset" should appear on the LCD screen.
4. Provision the device in one of three ways:

   - NCP Host provisioner examples, see for example an SDK folder `app/btmesh/example_host/btmesh_host_provisioner` or [github](https://github.com/SiliconLabs/bluetooth_mesh_stack_features/tree/master/provisioning)

   - NCP Commander with NCP target device, see [Bluetooth NCP Commander guide](https://docs.silabs.com/simplicity-studio-5-users-guide/latest/ss-5-users-guide-tools-bluetooth-ncp-commander) or [AN1259: Using the v3.x Silicon Labs Bluetooth Stack in Network Co-Processor Mode](https://www.silabs.com/documents/public/application-notes/an1259-bt-ncp-mode-sdk-v3x.pdf)

   - For Mobile Phone use, see the [QSG176: Bluetooth Mesh SDK v2.x Quick-Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg176-bluetooth-mesh-sdk-v2x-quick-start-guide.pdf) for more information how to download and use the Silicon Labs Bluetooth Mesh application.

   Mobile Phone provisioning is recommended for NLC examples. The mobile application provides a simple network for NLC devices. Provisioning is illustrated in the following figures.

![Bluetooth Mesh start screen](readme_img6.png)

5. Open the app, choose the NLC tab and tap **Scan**.

![Bluetooth Mesh Provision Browser](readme_img2.png)

6. Tap **PROVISION** and continue provisioning.

![Bluetooth Mesh Provisioning Device](readme_img3.png)

7. The device is then automatically provisioned and configured.

8. The next step is to add another one or several NLC devices into your network, including an NLC Basic Lightness Controller, if it has not already been done. This is required to fully test the whole system, for example the friendship and other features. You can then control the **Bluetooth Mesh - NLC Basic Lightness Controller** example by pressing the buttons on the device. Read the applicable example project documentation to learn more.

For more information on the example, see [AN1300: Understanding the Silicon Labs Bluetooth Mesh SDK v2.x Sensor Model Demonstration](https://www.silabs.com/documents/public/application-notes/an1300-understanding-bluetooth-mesh-sensor-model-demo-sdk-2x.pdf).

## Troubleshooting

Note that Software Example-based projects do not include a bootloader. However, they are configured to expect a bootloader to be present on the device. To install a bootloader, from the Launcher perspective's EXAMPLE PROJECTS & DEMOS tab either build and flash one of the bootloader examples or run one of the precompiled demos. Precompiled demos flash a bootloader as well as the application image.

- To flash an OTA DFU-capable bootloader to the device, flash the **Bluetooth Mesh - SoC Switch CTL** demo.
- To flash a UART DFU-capable bootloader to the device, flash the **Bluetooth Mesh - NCP Empty** demo.
- For other bootloader types, create your own bootloader project and flash it to the device before flashing your application.
- When you flash your application image to the device, use the *.hex* or *.s37* output file. Flashing *.bin* files may overwrite (erase) the bootloader.
- For more information, see [UG103.6: Bootloader Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-06-fundamentals-bootloading.pdf) and [UG489: Silicon Labs Gecko Bootloader User's Guide for GSDK 4.0 and Higher](https://cn.silabs.com/documents/public/user-guides/ug489-gecko-bootloader-user-guide-gsdk-4.pdf).

Before programming the radio board mounted on the mainboard, make sure the power supply switch the AEM position (right side) as shown below.

![Radio board power supply switch](readme_img0.png)

## Resources

[Bluetooth Documentation](https://docs.silabs.com/bluetooth/latest/)

[Bluetooth Mesh Network - An Introduction for Developers](https://www.bluetooth.com/wp-content/uploads/2019/03/Mesh-Technology-Overview.pdf)

[QSG176: Bluetooth Mesh SDK v2.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg176-bluetooth-mesh-sdk-v2x-quick-start-guide.pdf)

[AN1315: Bluetooth Mesh Device Power Consumption Measurements](https://www.silabs.com/documents/public/application-notes/an1315-bluetooth-mesh-power-consumption-measurements.pdf)

[AN1316: Bluetooth Mesh Parameter Tuning for Network Optimization](https://www.silabs.com/documents/public/application-notes/an1316-bluetooth-mesh-network-optimization.pdf)

[AN1317: Using Network Analyzer with Bluetooth Low Energy ® and Mesh](https://www.silabs.com/documents/public/application-notes/an1317-network-analyzer-with-bluetooth-mesh-le.pdf)

[AN1318: IV Update in a Bluetooth Mesh Network](https://www.silabs.com/documents/public/application-notes/an1318-bluetooth-mesh-iv-update.pdf)

[AN1300: Understanding the Silicon Labs Bluetooth Mesh SDK v2.x Sensor Model Demonstration](https://www.silabs.com/documents/public/application-notes/an1300-understanding-bluetooth-mesh-sensor-model-demo-sdk-2x.pdf)

[UG295: Silicon Labs Bluetooth Mesh C Application Developer's Guide for SDK v2.x](https://www.silabs.com/documents/public/user-guides/ug295-bluetooth-mesh-dev-guide.pdf)

[UG472: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf)

[Bluetooth Training](https://www.silabs.com/support/training/bluetooth)

## Report Bugs & Get Support

You are always encouraged and welcome to report any issues you found to us via [Silicon Labs Community](https://www.silabs.com/community).
