# RTS project

***Author: Maxime Vaidis <maxime.vaidis@norlab.ulaval.ca>***

***Code to process RTS data***

This RTS library aims to develop the processing of Robotic Total Stations data in mobile robotics.
Codes, datasets and examples are provided in this repository.
This library is linked to the project RTS of the [Norlab Laboratory](https://norlab.ulaval.ca/).

***Contents:***

* [Installation](#installation)
* [Manual](#manual)
* [Datasets](#datasets)
* [Contributing utilities](#contributing utilities)
* [Contributing](#contributing)
* [Trouble](#trouble)
* [Papers linked](#papers linked)

## Installation

RTS_Extrinsic_Calibration is supported on **Python 3.8.14** on **Ubuntu 22.04**.
You might also want to use a [virtual environment](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/blob/main/doc/install_in_virtualenv.md) for the installation.
The `requirements.txt` file contains all the packages needed for the installation.

## Manual

Details about the code are in the [Wiki](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/wiki).

## Datasets

A 2022 dataset consisting of `.bag` files with RTS data is available [Here](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/wiki) with a details explanation about the data collected.

The direct link to the 2022 dataset for download is http://norlab.s3.valeria.science/Vaidis2022_dataset/Vaidis2022_dataset.zip?AWSAccessKeyId=H3XB5NQ8TRQ0XCHFG8CB&Expires=2269184931&Signature=JfN9%2F19LYGzulyhdmlT%2F%2B1jjw5I%3D (44GB).

The 2022 dataset was taken with two different robotic platform and contains over 30 kilometers of prism trajectories tracked by three RTSs.

Note: the dataset of 2022 was collected through ROS 1.

## Contributing Utilities

A few "inoffical" scripts for special use-cases are collected in the `contrib/` directory of the repository. 
They are inofficial in the sense that they don't ship with the package distribution and thus aren't regularly tested in continuous integration.

## Contributing

Patches are welcome, preferably as pull requests.

## Trouble

**First aid:**
* check the [Wiki](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/wiki)
* check the [previous issues](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/issues?q=is%3Aissue+is%3Aclosed)
* open a [new issue](https://github.com/norlab-ulaval/RTS_Extrinsic_Calibration/issues)

## License

[MIT](https://mit-license.org/)

If you use this package for your research, a footnote with the link to this repository is appreciated: github.com/norlab-ulaval/RTS_Extrinsic_Calibration.

## Papers linked

[2022: Extrinsic calibration for highly accurate trajectories reconstruction](https://arxiv.org/abs/2210.01048)

[2021: Accurate outdoor ground truth based on total stations](https://arxiv.org/abs/2104.14396)
