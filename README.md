<h1 align="center">EKF localization system for NAO robot</h1>
<p align="center">Implementation of localization system for robot soccer based on inertial sensors</p>


## Pre-requisites
[Numpy](https://numpy.org/)

[Scipy](https://www.scipy.org/)



## Running the code

#### Copying code to robot

```bash
# Make the script executable
$ chmod +x send.sh

# Send files to NAO robot
$ ./send.sh <robot-ip>

```

#### Running the code on robot

```bash
# Connect to NAO robot
$ ssh nao@<robot-ip>

# Access the project folder
$ cd naoqi/ekf-localization

# Run the code
$ python threads.py

```




