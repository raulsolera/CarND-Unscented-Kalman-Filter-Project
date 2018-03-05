# CARND Unscented Kalman Filter Project

[//]: # (Image References)

[image1]: ./visualizations/report_images/a30yrd30_NIS_consistency_check.png "NIS check 30/30"
[image2]: ./visualizations/report_images/a3yrd3_NIS_consistency_check.png "NIS check 3/3"
[image3]: ./visualizations/report_images/a1yrd3_NIS_consistency_check.png "NIS check 1/3"
[image4]: ./visualizations/report_images/a1yrd1_NIS_consistency_check.png "NIS check 1/1"
[image5]: ./visualizations/report_images/a05yrd1_NIS_consistency_check.png "NIS check 0.5/1"
[image6]: ./visualizations/report_images/a05yrd05_NIS_consistency_check.png "NIS check 0.5/0.5"
[image7]: ./visualizations/report_images/a025yrd025_ini0303_NIS_consistency_check.png "NIS check 0.25/0.25"
[image8]: ./visualizations/report_images/a01yrd01_ini0303_NIS_consistency_check.png "NIS check 0.1/0.1"

[image9]: ./visualizations/report_images/a05yrd05_ini0303_xy_position_comparison.png "XY position"
[image10]: ./visualizations/report_images/a05yrd05_ini0303_independent_x_y_position_comparison.png "Independent X Y position"
[image11]: ./visualizations/report_images/a05yrd05_ini0303_independent_vx_vy_position_comparison.png "Independent vx vy position"

[image12]: ./visualizations/report_images/a30yrd30_xy_position_comparison.png "XY position"
[image13]: ./visualizations/report_images/a30yrd30_independent_x_y_position_comparison.png "Independent X Y position"
[image14]: ./visualizations/report_images/a30yrd30_independent_vx_vy_position_comparison.png "Independent vx vy position"


Self-Driving Car Engineer Nanodegree Program

This project consist in the implementation of a kalman filter in C++ and testing it with a simulator provided by Udacity ([download here](https://github.com/udacity/self-driving-car-sim/releases)) that generates noisy Radar an Lidar measurements. The communication between the simulator and the EKF is done using WebSocket using the uWebSockets implementation (this repository includes scripts for Mac Os: install-mac.sh and Ubuntu: install-ubuntu.sh to install uWebSockets).


## Compiling and executing the project

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Clone the repo and navigate to it on a Terminal.
2. (Within the project repo) mkdir build
3. cd build
4. cmake ..
5. make
6. ./UnscentedKF


## Unscented Kalman filter for Dummies

A Kalman filter is an algorithm that "_uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone_" ([wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)).

In practice this consists in predicting a future state based in a transition model (from actual state to future state) that has some noise and model inaccuracies (predict step) and correct this prediction based in a measurement provided by a sensor that also has noise (update state).

There exists different Kalman filter variations that deal with the non-linearity of the transition model or the projection of the state into the measurement space. One of them is the [Exended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) that was implemented in the previous [Extended Kalman Filter Project](https://github.com/raulsolera/CarND-Extended-Kalman-Filter-Project) and which is based in linearization using the first order Taylor expansion.

Other is the [Unscented Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter) which is used in this [project](https://github.com/raulsolera/CarND-Unscented-Kalman-Filter-Project) that deals with non-linearity using sigma points that are used in every non-linear transformation: sigma points are transformed and then the expected value (mean) and covariance matrix is found following the scheme:

Prediction Step:
- Generate sigma points
- Predict sigma point
- Calculate mean and covariance

Update step:
- Predict measurement (project predicted sigma points into the measurement space)
- Update state


## Kalman filter implementation

This algorithm has been implemented in C++ in the source code files provided in the src directory using the templates provided by Udacity with the following changes:
- **main.cpp**: code writing records to a file.
- **ukf.cpp**: variable initialization, initialization of Unscented Kalman Filter, predict step and update step for Radar and Lidar.
- **tools.cpp**: implementation of method CalculateRMSE to calculate the mean square error of the algorithm.


## Kalman filter tunning

There are some parameters that have impact in the filter results, in this project we will tune the acceleration and yaw rate dot noise in order to get the smaller RMSE. To do so we will use the consistency check NIS using different parameters with the following sequence:

|     acc. noise | yaw rate noise |
|----------------|:--------------:|
|          30    |          30    |
|           3    |           3    |
|           1    |           3    |
|           1    |           1    |
|           0.5  |           1    |
|           0.5  |           0.5  |
|           0.25 |           0.25 |
|           0.1  |           0.1  |

The following images shows the NIS consistency check for Radar and Lidar for parameter values 30/30, 3/3, 0.5/0.5, 0.1/0.1:
##### Parameter values 30/30
![alt text][image1]
##### Parameter values 3/3
![alt text][image2]
##### Parameter values 0.5/0.5
![alt text][image6]
##### Parameter values 0.1/0.1
![alt text][image8]

3/3 and 0.5/0.5 seem to be parameters in a reasonable range.

## RMSE results

The RMSE error can also be influenced by the initialization done for covariance matrix and velocity.
- In the case of the covariance matrix values of 1 and 0.3 (the value for radar standard deviation in radius) have been tested for px and py.
- In the case of velocity 0 and 5 m/s values have been tested for the best veral result so far (which correspond to the 0.5/0.5 (0.3/0.3) combination)

These combinations result in the following RMSE results that yield the best value for the parameter values of:
- Acceleration / Yaw rate dot noise = 0.5 / 0.5
- Covariance matrix sigma x & sigma y = 0.3 / 0.3
- Initial velocity = 0 m/s

#### Best result
| Parameters acc / yrd noise (px/py/v ini) |     px     |     py     |     vx    |     vy    |
|------------------------------------------|:----------:|:----------:|:---------:|:---------:|
| **0.5/0.5 (0.3/0.3/0)**                  | **0.0610** | **0.0850** | **0.329** | **0.209** |

#### Overall results
| Parameters acc / yrd noise (px/py/v ini) |     px     |     py     |     vx    |     vy    |
|------------------------------------------|:----------:|:----------:|:---------:|:---------:|
| 30/30 (1/1/0)                            |     0.0967 |     0.1202 |     0.866 |     0.969 |
| 3/3 (1/1/0)                              |     0.0748 |     0.0885 |     0.383 |     0.309 |
| 1/3 (1/1/0)                              |     0.0648 |     0.0865 |     0.361 |     0.270 |
| 1/1 (1/1/0)                              |     0.0647 |     0.0836 |     0.335 |     0.219 |
| 0.5/1 (1/1/0)                            |     0.0596 |     0.0872 |     0.334 |     0.221 |
| 0.5/0.5 (1/1/0)                          |     0.0611 |     0.0859 |     0.330 |     0.213 |
| **0.5/0.5 (0.3/0.3/0)**                  | **0.0610** | **0.0850** | **0.329** | **0.209** |
| 0.5/0.5 (0.3/0.3/5)                      |     0.0605 |     0.0886 |     0.187 |     0.387 |
| 0.25/0.25 (0.3/0.3/0)                    |     0.0669 |     0.0943 |     0.343 |     0.228 |
| 0.1/0.1 (0.3/0.3/0)                      |     0.1252 |     0.1347 |     0.416 |     0.311 |


## Result visualization

Visualization of predicting power of the filter with parameters already tuned and the initial parameter values:

### Paremeters already tuned
![alt text][image9]
![alt text][image10]
![alt text][image11]

### Initial Paremeter values
![alt text][image12]
![alt text][image13]
![alt text][image14]

### Conclusion

Whereas X, Y prediction seems comparable for the initial parameter values, the velocity results show the improvements that can be achieve with an appropriate paremeters tunning up.





