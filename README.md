# RTS project

***Author: Maxime Vaidis <maxime.vaidis@norlab.ulaval.ca>***

***Update: September 30th 2023 by Maxime Vaidis***

***Code to process RTS data***

This RTS library aims to develop the processing of Robotic Total Stations data in mobile robotics.
Codes, datasets and examples are provided in this repository.
This library is linked to the project RTS of the [Norlab Laboratory](https://norlab.ulaval.ca/).

***Contents:***

* [Installation](#installation)
* [Manual](#manual)
* [Datasets](#datasets)
* [Contributing utilities](#contributing-utilities)
* [Contributing](#contributing)
* [Trouble](#trouble)
* [Papers linked](#papers-linked)

## Installation

RTS_Extrinsic_Calibration is supported on **Python 3.8.14** on **Ubuntu 22.04**.
You might also want to use a [virtual environment](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/blob/main/doc/install_in_virtualenv.md) for the installation.
The `requirements.txt` file contains all the packages needed for the installation.

## Manual

Details about the code are in the [Wiki](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/wiki).

## Datasets

A 2023 dataset consisting of `.bag` files with RTS data is available [Here](https://github.com/norlab-ulaval/Norlab_wiki/wiki/RTS%E2%80%90GT-Dataset-2023) with a details explanation about the data collected.

The RTS-GT dataset was taken with two different robotic platform and contains over 49 kilometers of prism trajectories tracked by three RTSs.

Note: the dataset of 2023 was collected through ROS 1 for the RTS data, and ROS 2 for the robots' data

## Contributing Utilities

A few "inoffical" scripts for special use-cases are collected in the `contrib/` directory of the repository. 
They are inofficial in the sense that they don't ship with the package distribution and thus aren't regularly tested in continuous integration.

## Contributing

Patches are welcome, preferably as pull requests.

## Trouble

**First aid:**
* Check the [Wiki](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/wiki)
* Check the [previous issues](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/issues?q=is%3Aissue+is%3Aclosed)
* Open a [new issue](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/issues)

## License

[MIT](https://mit-license.org/)

If you use this package for your research, a footnote with the link to this repository is appreciated: github.com/norlab-ulaval/RTS_Extrinsic_Calibration.

## Papers linked

[2023: RTS-GT: Robotic Total Stations Ground Truthing dataset (Submitted to ICRA 2024)](https://arxiv.org/abs/2309.11935)

[2023: Uncertainty analysis for accurate ground truth trajectories with robotic total stations (IROS 2023)](https://arxiv.org/abs/2308.01553)

[2023: Extrinsic calibration for highly accurate trajectories reconstruction (ICRA 2023)](https://ieeexplore.ieee.org/document/10160505)

[2021: Accurate outdoor ground truth based on total stations (CRV 2021)](https://ieeexplore.ieee.org/abstract/document/9469468)
