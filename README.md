# SensorToolKit-STK [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![GitHub Release](https://img.shields.io/github/v/release/andriandreo/SensorToolKit-STK)](https://github.com/andriandreo/SensorToolKit-STK/releases/latest) [![GitHub All Downloads](https://img.shields.io/github/downloads/andriandreo/SensorToolKit-STK/total)]() [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.15744691.svg)](https://doi.org/10.5281/zenodo.15744691)

<img src="STK.png" alt="STK Board" width="500"/>

A portable potentiostat built upon AD5940/AD5941 Analog Front End (Analog Devices, Inc.) designed for distributed IoT applications interfacing electrochemical sensors.

## About

The **SensorToolKit (STK) Project** is built upon the collaboration of [JLM Innovation GmbH](https://www.jlm-innovation.de/) and [Nanosensors Research Group](https://www.chemosens.recerca.urv.cat/en/research/lines/) ([CHEMOSENS](https://www.chemosens.recerca.urv.cat/en)) at the [Department of Analytical Chemistry and Organic Chemistry](https://www.qaqo.urv.cat/en/) of the [Universitat Rovira i Virgili](https://www.urv.cat/en/).

### Collaborators

- Jan Mitrovics
- Simone Saporito
- Moritz Kleinstraß
- Tomás Cantón
- Andrés Alberto Andreo Acosta

### License

This code is released with a general [MIT licence](https://github.com/andriandreo/SensorToolKit-STK/blob/main/LICENSE).

However, certain pieces of the code are adapted either from [Analog Devices, Inc. examples](https://github.com/analogdevicesinc/ad5940-examples) or [ESP-IDF examples and APIs](https://idf.espressif.com/) ([Espressif Systems](https://www.espressif.com/)), while following the [Analog Devices, Inc. porting guidelines](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/porting_source_code) for different microcontrollers (MCUs). Please, respect their respective licenses when using the related code.

## Firmware 

The tested port for ESP32 MCUs (including full support for ESP32-S3 and ESP32-C3 XIAO boards by [SeeedStudio](https://www.seeedstudio.com/)) and implemented applications/techniques are hosted inside the [`Firmware` folder](https://github.com/andriandreo/SensorToolKit-STK/tree/main/Firmware/ESP-IDF).

Further information related to its setup can be found in the corresponding [`README`](https://github.com/andriandreo/SensorToolKit-STK/tree/main/Firmware/ESP-IDF/README.md) inside.

Another working port for Arduino-compatible MCUs (with tested support for nRF52840, ESP32-S3 and ESP32-C3 XIAO boards by [SeeedStudio](https://www.seeedstudio.com/)) can be found inside the corresponding [`Firmware` folder](https://github.com/andriandreo/SensorToolKit-STK/tree/main/Firmware/Arduino).

## Server Side

The Server Side infrastructure built upon Docker containers and setup information thereof can be found inside the [`Server Side` folder](https://github.com/andriandreo/SensorToolKit-STK/tree/main/Server%20Side).



