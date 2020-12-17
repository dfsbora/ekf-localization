<h1 align="center">EKF localization system for NAO robot</h1>
<p align="center">Implementation of localization system for robot soccer based on inertial sensors</p>


<p align="center">
  <img alt="GitHub" src="https://img.shields.io/github/license/dfsbora/ekf-localization">
</p>


## Pre-requisites
Developed on Python 2.7

🤖 NAO robot must meet the following pre-requisites:

* NAOqi (Verified with 2.8.5.10)

* [Numpy](https://numpy.org/) (Verified with 1.10.4)

* [Scipy](https://www.scipy.org/) (Verified with v1.2.3)



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


## License

This project is under the license [MIT](./LICENSE).

Made by Débora Ferreira. 🤖💚
