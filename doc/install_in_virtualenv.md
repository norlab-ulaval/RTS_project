# Installation in a virtual environment

Virtual environments allow you to install Python packages in an isolated environment.
This is usually a good idea because it reduces the risk that you mess up your system's Python packages by installing globally with `pip`.
Additionally, you can have multiple environments in parallel that don't interfere with each other.

## venv

`venv` is highly recommended, it makes using virtual environments.

Below are installation instructions for Ubuntu.
If you use any other OS, see the documentation for how to install it on your system:

* venv documentation: https://docs.python.org/3/library/venv.html

### venv installation on Ubuntu

***The following steps have been verified on Ubuntu 20. They probably also work on other Debian-based Linux distros.***

Install `venv` with the Python version you have:
```shell
python3 -m pip install --upgrade pip
pip3 install virtualenv
```

## Setting up a virtualenv for RTS_Extrinsic_Calibration library

Once `venv` is installed, we can create the virtual environment.
```shell
python3 -m venv /path/to/new/virtual/environment
```

To activate the environment, type:
```shell
source /path/to/new/virtual/environment/bin/activate
```

Install the library package and its dependencies inside the virtual environment:
```shell
pip3 install -r requirements.txt
```
Now, the package should be installed in the virtualenv and you can use it.

To leave the virtualenv, close the shell or type:
```shell
deactivate
```
(activate again with `source /path/to/new/virtual/environment/bin/activate`)

To delete the environment:
```shell
rm -r /path/to/new/virtual/environment
```
