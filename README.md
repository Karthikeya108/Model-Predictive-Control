# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation and Parameter Values

 N (timestep length) is set to 12.
 I tried different values for N for instance
 N = 50 
 
 dt (elapsed duration between timesteps) is set to 0.1
 I tried different values for instance 0.05 (as in the MPC quiz) - But this led to oscillating motion of the vehicle.
 
 ref_v (reference velocity) is set to 100 MPH
 
 # Steps:

main.cpp

1. Fetch the state ``px, py, v, psi`` and the control ``steering angle & throttle``
2. Set the latency to 100 ms (0.1s)
3. Fetch the reference trajectory ``ptsx`` and ``ptsy``
4. Convert the coordinates to vehicle coordinate system
5. Apply 3rd order polynomial fitting.
6. Compute ``cross track error (cte)`` and ``orientation error (epsi)``
7. Predict and set the ``state`` accounting for ``latency``
8. Using the MPC solver predict the optimum ``steering angle`` and ``throttle``


MPC Implementation (MPC.cpp)

Referenced from https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp
 
With the folling changes:

1. Used 3rd order polynomial instead of linear model


 
 
 
