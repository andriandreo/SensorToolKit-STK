| Tested Targets | ESP32-C3* | ESP32-S3* | nRF52840* |
| -------------- | --------- | --------- | --------- |

\* [XIAO DevKits](https://wiki.seeedstudio.com/SeeedStudio_XIAO_Series_Introduction/)

# STK - Firmware

## ARDUINO Sketch (basic) Layout

```
- Arduino/
             - libraries/  - ad5940lib/ - ad5940.c
                                        - ad5940.h
             - main/       - main.ino
                           - ARDUINOPort.cpp
                           - application.cpp
                           - application_aux.c
                           - application_aux.h
                           - Applications/ - Amperometry/ - AD5940_AMPmain.cpp
                                                          - Amperometric.c
                                                          - Amperometric.h
                                           - Open Circuit Potentiometry/ - AD5940_ADCSeqMUX.cpp
                                                                         - AD5940_OCP-ADCPolling.cpp
                                                                         - ADCSequencer.h
```

## Framework Setup

To setup and compile the STK firmware for ARDUINO-compatible devices:

1. [Install ARDUINO-IDE for your OS](https://www.arduino.cc/en/software) (and/or your favourite IDE);
1. clone this repository recursively (`git clone --recursive https://github.com/andriandreo/SensorToolKit-STK.git`) to download also the [`ad5940lib` from Analog Devices, Inc.](https://github.com/analogdevicesinc/ad5940lib), or manually download/clone both as usual;
1. place the `ad5940lib` folder inside `Firmware/Arduino/libraries/` as depicted;
1. (optional — but recommended to avoid missing dependency issues) copy the contents of `Firmware/Arduino` inside `/path/to/Arduino` of [your local ARDUINO-IDE installation](https://support.arduino.cc/hc/en-us/articles/4415103213714-Find-sketches-libraries-board-cores-and-other-files-on-your-computer);
1. edit `ad5940.h` file to uncomment `#define CHIPSEL_594X      /**< AD5940 or AD5941 */` for selecting AD5940/AD5941 chipsets instead of ADUCM350;
1. copy the desired application source file(s) (and required dependencies) from `Arduino/main/Applications/<CHOSEN_APP>` to `Arduino/main` — **NOTE THAT ONLY ONE `APP.CPP` FILE MUST BE PLACED AT ONCE**;
1. open `Arduino/main` sketch folder with the IDE;
1. connect the board *via* USB and select the right Board Support Package (BSP) and COM port in the IDE;
1. edit the contents of `Arduino/main/ARDUINOPort.cpp` to uncomment one of `#define XIAOnRF`/`#define XIAOS3`/`#define XIAOC3` for configuring the firmware to the desired board target (nRF52840, ESP32-S3 or ESP32-C3, respectively);
1. finally build the project and flash the firmware to the board using `Upload` command button of the IDE.

## Data logging

The firmware port for ARDUINO lacks Wi-Fi + MQTT implementation so far.

However, `printf` statements are intented to generate comma-separated value (CSV)-like logs that can be saved with the right serial communication tool when connected to the board running the firmware (e.g. [`tio`](https://github.com/tio/tio) or [Termite](https://www.compuphase.com/software_termite.htm)).
