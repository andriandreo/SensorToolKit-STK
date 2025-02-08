| Tested Targets | ESP32-C3 | ESP32-S3 |
| [XIAO DevKits](https://wiki.seeedstudio.com/SeeedStudio_XIAO_Series_Introduction/) | 

# STK - Firmware

## ESP-IDF Project Layout

The firmware for the ESP32 port of this project follows [the ESP-IDF project layout](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#example-project) in the form of:

```
- myProject/
             - CMakeLists.txt
             - sdkconfig
             - dependencies.lock
             - bootloader_components/ - boot_component/ - CMakeLists.txt
                                                        - Kconfig
                                                        - src1.c
             - components/ - component1/ - CMakeLists.txt
                                         - Kconfig
                                         - src1.c
                           - component2/ - CMakeLists.txt
                                         - Kconfig
                                         - src1.c
                                         - include/ - component2.h
             - managed_components/ - namespace__component-name/ - CMakelists.txt
                                                                - src1.c
                                                                - idf_component.yml
                                                                - include/ - src1.h
             - main/       - CMakeLists.txt
                           - src1.c
                           - src2.c
                           - idf_component.yml
             - build/
```

## Framework Setup

To setup and compile the STK firmware with this configuration:

1. [Install the ESP-IDF for your OS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation) and/or your favourite IDE (e.g. [VSCode](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/));
1. clone this repository recursively (`git clone --recursive https://github.com/andriandreo/SensorToolKit-STK.git`) to download also the [`ad5940lib` from Analog Devices, Inc.](https://github.com/analogdevicesinc/ad5940lib), or manually download/clone both as usual;
1. place the contents of `ad5940lib` inside `Firmware/components/ad5940lib` as depicted;
1. edit `ad5940.h` file to uncomment `#define CHIPSEL_594X      /**< AD5940 or AD5941 */` for selecting AD5940/AD5941 chipsets instead of ADUCM350;
1. edit the `Firmware/main/CMakeLists.txt` to uncomment the desired application, note that `main.c` MUST REMAIN UNCOMMENTED as well;
1. connect the board *via* USB and select the right COM port in the IDE;
1. select the target board *via* `idf.py set-target <target>`;
1. configure the application (ESP32 interfacing, flash memory and Wi-Fi/MQTT credentials) *via* `idf.py menuconfig` or graphically with the selected IDE;
1. finally build the project (`idf.py build`) and flash the firmware to the board (`idf.py flash`) - COM PORT may be selected at this point *via* `idf.py flash -p <port>`.

Further guidance for `idf.py` CLI command tools can be found [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-py.html).

## Data logging

Applications files including Wi-Fi + MQTT implementation will upload the recorded data to the cloud server when the latter is configured.

Moreover, `printf` statements are intented to generate comma-separated value (CSV)-like logs that can be saved with the right serial communication tool when connected to the board running the firmware (e.g. [`tio`](https://github.com/tio/tio) or [Termite](https://www.compuphase.com/software_termite.htm)).
