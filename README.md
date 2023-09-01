# Carla - Adopticum 🔬 / Magna 🚀

## Summer Project 2023 🌞

This summer, Adopticum and Magna will collaborate on a development project to explore the utilization of simulation engines in conjunction with game engines to detect and classify unknown objects on the ground. 🤝🔍

## Project Overview 🌍

The Carla - Adopticum / Magna project aims to explore the integration of simulation engines and game engines to detect and classify unknown objects on the ground. By utilizing the CARLA simulator, we seek to generate simulated and realistic data that can be used to train machine/deep learning models effectively. The project focuses on emulating optical measurement techniques employed by Adopticum and Magna, reducing the dependency on manual data collection. 

## Installation 🔧📥

### Requirements 📋
The major installations are:
- [CARLA>=0.9.14](https://carla.org/2022/12/23/release-0.9.14/)
    Follow the installation guide for your operating system. The project is developed using the [Linux](https://carla.readthedocs.io/en/latest/build_linux/) installation. We recommend using the [pre-built version](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.14.tar.gz) of CARLA and installing the [additional maps](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.14.tar.gz).
- [Python>=3.8](https://www.python.org/downloads/)
- [Unreal Engine==4.26](https://www.unrealengine.com/en-US/download)
    To install Unreal Engine properly you should follow the instructions on the [CARLA documentation](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/#unreal-engine).
### Carla Environment 🚗
We recommend that you install the Carla environment in a virtual environment and install the Carla Python API via pip.
Make sure that you add the root path to your carla installation to **CARLA_ROOT**. This can be done by adding the following line to your **.bashrc** file:

```bash

export CARLA_ROOT=<path-to-carla>

```
Make sure to replace the `<path-to-carla>` with the path to your Carla installation and restart all your terminals.


<details close>
<summary>👉 command details</summary>
Install the Carla Python API via pip using the following command:

```bash
pip install <path-to-carla>/PythonAPI/carla/dist/carla-<version>-cp<python-version>-cp<python-version>-<os>.whl
```

Make sure to replace the `<path-to-carla>` with the path to your Carla installation, `<version>` with the version of your Carla installation, `<python-version>` with the version of Python you are using, and `<os>` with your operating system. For example, if you are using Python 3.8 on Linux and have installed Carla 0.9.14, you should run the following command:


```bash
pip install ~/carla/PythonAPI/carla/dist/carla-0.9.14-cp38-cp38-linux_x86_64.whl
```

</details>

### Python Environment 🐍
To install the required Python packages, run the following command in the root directory of the project:
```bash
pip install -r requirements.txt
```

## Usage 📚

Explain how to use the project, including instructions for running simulations, configuring parameters, and accessing the generated data. 

## Features 💎

- Simulated and realistic data generation using the CARLA simulator. 🚗🌍
- Training machine/deep learning models to efficiently detect and classify unknown objects. 🤖🦾
- Emulation of optical measurement techniques for accurate representation. 🔫
- Reduction in manual data collection efforts. 📉⏳
- Cost and time savings for data collection processes. 💰 

## Roadmap 🚧🗺️

Outline the future plans and potential enhancements for the project. Include upcoming features, improvements, or research areas that will be explored. 

## Contributors 👥
### Code Contributors 🖥️
- [Theodor Jonsson](https://github.com/Tottowich) - [LinkedIn](https://se.linkedin.com/in/theodor-jonsson-32534b228)
### Assets Contributors 🎨
- [Ruben Forsgren]() - [LinkedIn](https://se.linkedin.com/in/ruben-forsgren-3214a9229)
### Project Management 📊
- [Adopticum](https://www.adopticum.se/sv-SE) 🔬 - [LinkedIn](https://se.linkedin.com/company/adopticum)
- [Magna](https://www.magna.com/) 🚀 - [LinkedIn](https://ca.linkedin.com/company/magna-international)

## License 🔏

The Carla - Adopticum / Magna project is released under the [MIT License](https://opensource.org/licenses/MIT).

## Changelog 📜📅🔄

<!-- Document the updates, progress, and version history of the project in a clear and organized manner. Follow a consistent format that includes the version number, date, and a summary of the changes made. -->

All the major changes can be found in the [pages/CHANGELOG.md](pages/CHANGELOG.md) file. Below you can find examples per described category. Along with some examples of changelogs.

### [Date]📅
| Category                | Example description                                |
|-------------------------|----------------------------------------------------|
| **Added 📥**            | Added new feature X                                |
| **Changed 🔄**          | Updated function Y to improve performance          |
| **Removed 🗑️**          | Removed deprecated API Z                           |
| **Fixed 🔧**            | Fixed bug causing application crash                |
| **Security 🔒**         | Implemented enhanced encryption for user data      |
| **Deprecated 📛**       | Marked method A as deprecated, use method B instead|
| **Breaking Changes 🚨** | Renamed class C, update references accordingly     |
| **Documentation 📚**    | Updated API documentation for better clarity       |
| **Maintenance 🧹**      | Cleaned up codebase, removed unused variables      |
| **Performance 🚀**      | Optimized database queries for faster response time|
| **Refactoring 📦**      | Extracted reusable components from module D        |
| **Style 🎨**            | Applied consistent coding style across the project |
| **Tests 🧪**            | Added unit tests for module E                      |
| **Other 📦**            | Miscellaneous updates and improvements             |


<details close>
<summary>👉 Example of a changelog</summary>

### [2023-06-01]📅
- **Added 📥:** Implemented data generation using CARLA simulator
- **Fixed 🔧:** Fixed bug causing application crash

Summary of changes made in this version.

### [2023-06-10]📅
- **Changed 🔄:** Updated function Y to improve performance
- **Removed 🗑️:** Removed deprecated API Z


Summary of changes made in this version.

</details>

## Acknowledgements 🙏
- [CARLA](https://carla.org/) 🚗🌍
- [Adopticum](https://www.adopticum.se/sv-SE) 🔬
- [Magna](https://www.magna.com/) 🚀