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
 N = 25   Works fine for sometime but eventually the vehicle drives out of the track.
 N = 5    The vehicle drives out of the track in few seconds.
 
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

1. Multipled by constants to increase the weigths of costs
```
 // Cost for CTE, psi error and velocity
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += 2500 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2500 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Costs for steering (delta) and acceleration (a)
    for (unsigned int t = 0; t < N-1; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 100 * CppAD::pow(vars[a_start + t], 2);
    }

    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (unsigned int t = 0; t < N-2; t++) {
      fg[0] += 50 * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 50 * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
2. Used 3rd order polynomial instead of linear model

```
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));
```
3. Followed other hints from the lessons.


 
 
 
