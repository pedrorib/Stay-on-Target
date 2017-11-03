# Stay-on-Target

Stay on target is a 3-axial micropositioning platform developed by Fernando Franco, Marília Silva, Miguel Neto and Pedro Ribeiro - PhD students at INESC-MN.
This repository stores all the code developed for both the host (written in MATLAB language) and the raspberry Pi positioner controller (written with Bash script and Expect script).

For this code to work on your platform, you must install the following dependencies on the host computer:
* MATLAB R2017a or later
* Raspberry Pi support package 

And you must have a Raspberry Pi 3 with BLE connectivity, running the MATLAB modified raspbian OS with the following dependency installed:
* Expect 5.54
