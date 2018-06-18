# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

**This file is updated based on the assignment submission.**

[//]: # (Image References)
[image1]: ./ukf-result.png

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

## Relevant Implementation Details.
This section describes the files modified / extended to implement the Extended Kalman filter.


### `ukf.cpp` and `ukf.h`
These files contain main sensor data processing objects and associated functions. The main implementation logic includes the following.
1. Variables and matrices initialization including (x_, P_, Xsig_pred_ H_, R_, and noise parameters, etc.)
2. Handling first frame with appropriate position values assignment
3. Updating x_ and P_ matrices based on elapsed time between measurements, in prediction.
4. Invoking relevant update step as per sensor data.

The following situations are handled appropriately.
1. Avoid division by zero: Ensure that position values are non_zero (atleast one of the position coordinate either px or py shall be non_zero) to avoid division by zero.
2. Normalize Angles: The angle part of  y (difference between measurement and prediction) shall be normalized between +pi and -pi for correct RMSE value computation and correct predictions and updates.

Rest of the implementation is taken from class quizzes.

### `tools.cpp`
This file contains essentially functions to normalize angles, without excessive loops and RMSE error computation.

## Observations
I exercised the implementation in various scenarios as described in this section.

### Observation 1:  Data set 1, with sensor fusion (laser + radar); std_a_ = 0.5; std_yawdd_ = 0.5; and diagnal elements of P_ are initialized to 0.25

The RMSE values observed were well below the limits of RMSE <= (Px:0.09, Py:0.10, Vx:0.40, Vy:0.30), i.e. **(Px:0.0621, Py:0.0836, Vx:0.2940, Vy:0.1790)**, as captured in the following picture.
![alt text][image1]


### Observation 2: Observations with Fusion (Laser (R) + Radar (R)), Laser only ('L') and  Radar only ('R').
I observed three combinations of RMSE measurement process with L, R and Fusion (L+R).  The observations are captured in the following table.
As expected the RMSE numbers are the best (lowest) for fusion, which is the advantage of Unscented Kalman filter.
Laser performed better than Radar in single sensor processing, as expected.

 |Num| std_a_ |std_yawdd_|P_ diagnal |Mode|Dataset|RMSE-Px|RMSE-Py|RMSE-Vx|RMSE-Vy|
 |:-:|:------:|:--------:|:---------:|:--:|:-----:|:-----:|:-----:|:-----:|:-----:|
 |1  |  0.75  |   0.75   |    1.0    | L+R|   1   | 0.0629| 0.0841| 0.3314| 0.2138|
 |2  |  0.60  |   0.60   |    1.0    | L+R|   1   | 0.0614| 0.0858| 0.3816| 0.2585|
 |3  |  0.50  |   0.45   |    1.0    | L+R|   1   | 0.0617| 0.0858| 0.3308| 0.2144|
 |4  |  0.50  |   0.50   |    0.25   | L+R|   1   | 0.0621| 0.0836| 0.2940| 0.1790|
 |5  |  0.50  |   0.50   |    0.25   | L  |   1   | 0.1266| 0.0989| 0.7098| 0.2340|
 |6  |  0.50  |   0.50   |    0.25   | R  |   1   | 0.1535| 0.1930| 0.3228| 0.2276|

### Observation 3: Comparison between  EKF and UKF observations.
Overall UKF performed far better than EKF.  The following are the data.

 |Num|Noise_ax|Noise_ay|Initial (vx, vy)|Mode|Dataset|RMSE-Px|RMSE-Py|RMSE-Vx|RMSE-Vy|
 |:-:|:------:|:------:|:--------------:|:--:|:-----:|:-----:|:-----:|:-----:|:-----:|
 |1  |    9   |   9    |      (0,0)     | L+R|   1   | 0.0973| 0.0855| 0.4513| 0.4399|
 
 |Num| std_a_ |std_yawdd_|P_ diagnal |Mode|Dataset|RMSE-Px|RMSE-Py|RMSE-Vx|RMSE-Vy|
 |:-:|:------:|:--------:|:---------:|:--:|:-----:|:-----:|:-----:|:-----:|:-----:|
 |4  |  0.50  |   0.50   |    0.25   | L+R|   1   | 0.0621| 0.0836| 0.2940| 0.1790|

Overal, this assignment offered me good grasp the implmentation of EKF and helps further understanding theory behind. I would like do NIS analysis later.




1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

