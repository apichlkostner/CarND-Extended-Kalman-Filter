# Extended Kalman Filter Project

# Summary

This project implements sensorfusion of radar and lidar sensor using an extened kalman filter.

The steps of this project are the following:

* Implementation of the extended kalman filter
* Connection of the kalman filter with the simulator using websockets
* Running the simulator and process the sensor data sent over the websockets
* Tracking the object with position and velocity
* Comparing the result with the ground truth as RSME
* Checking the results with only one of the sensors used

# The filter

## System
The system model uses position and velocity and assumes a constant veloctiy vector.

## Radar sensor
Measures position and velocity of the object in polar coordinates. Since the measurement function is not linear an extended kalman filter step in needed for the correction.

## Lidar sensor
Measures only the position of the object. The measurement function is linear.

# Implementation

## Classes
* `FusionEKF`
  * is called by `main()` with new measurements
  * initializes the kalman filter
  * calls the kalman filter for prediction and update steps
* `kalman_filter`
* `System`
  * system matrix
  * covariance matrix
* `Measurement`
  * covariance matrix
  * `LidarMeasurement`
    * measurement matrix for kalman filter
  * `RadarMeasurement`
    * coordinate transformation polar <-> cartesian
    * jacobian for extended kalman filter
* `Tools`
  * calculation of RMSE

## Optimization
* If two measurements arrive at nearly the same time only one predict step is done but two update steps.
* If the first measurement is from a lidar sensor the velocity can't be initialized. This is then done as soon as the first radar measurement arrives.
* Smaller methods are implemented directly in the header file.

# Results

## Expectations
Radar and lidar sensors complement each other with their different characteristics.

The position obtained by radar sensors have a lower spatial resolution but can also detect the speed ob an object. With lidar sensors the spatial resolution is higher but the objects speed can't be measured directly.

Since this implementation models the object with a constant velocity vector the result on the object here which moves on a circular trajectory are not the best. In the next project a better system model which is based on a constant steering angle in combination with an unscented kalman filter will be used.

## Result

As expected the result with both sensors are the best:

![Result with both radar and lidar](readme_files/result_radar_lidar.png)

With only radar the position is not very exact:

![Result with radar only](readme_files/result_radar_only.png)

With only lidar the velocity is not very exact. And since the velocity is not initialized by the radar measurement the position is also not good averaged over the complete trajectory. If the radar measurements are only used to initialize the velocity the averaged result is much better (X: 0.22, Y: 0.15, VX: 2.1, VY: 1.3)

![Result with lidar only](readme_files/result_lidar_only.png)

# Build

## Dependencies

* uWebSocketIO
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
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
