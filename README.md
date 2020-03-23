# Localization via Particle Filter
Udacity Self-Driving Car Engineer Nanodegree Program

This project utilizes a particle filter to estimate the position of a moving object, using noisy measurements of surrounding landmarks. Maintaining low error values in X, Y, and heading is the main focus of this project, as any object tracking software would be useless in the real-world if it is not accurate.

[//]: # (Image References)
[image1]: Progress1.jpg "Runtime Example"
[image2]: Progress2.jpg "Success!"

## Basic Build Instructions

Running the code requires connecting to the Udacity CarND Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by executing in the Linux bash:

```
$ git clone https://github.com/briansfma/CarND-Kidnapped-Vehicle-Project
$ cd CarND-Kidnapped-Vehicle-Project
$ mkdir build
$ cd build
$ cmake ..
$ make $*
$ ./particle_filter
```

## Running the Program

If it is not running already, launch the project.

```
$ cd CarND-Kidnapped-Vehicle-Project
$ cd build
$ ./particle_filter
```

When `particle_filter` has initialized successfully, it will output to terminal

```
Listening to port 4567
```

Launch the simulator `term2_sim.exe`. Select "Project 3: Kidnapped Vehicle" from the menu. Upon successful connection to the simulator, `particle_filter` will output to terminal

```
Connected!!!
```

Click the "Start" button and the simulator will run. `particle_filter` will begin outputting highest and average weights (representing likelihoods of particle guesses being correct) to the terminal. The error values will be outputted to the simulation screen itself under "x   y   yaw". Green lines represent measurements from the car in reality, wheras blue lines represent those measurements as projected from the current belief of the car's position.

![alt text][image1]

For reference, the simulator is self-grading, so upon completion of the run sequence, the simulator should have a message like:

![alt text][image2]

## Other Important Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
