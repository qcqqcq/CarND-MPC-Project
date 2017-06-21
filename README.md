# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


---
## Model
Basic MPC model. State is:

* x position
* y position
* psi (vehicle orientation angle in radians)
* v (velocity in m/s)
* cte (cross track error. Difference between reference line and actual vehicle position
* epsi (error in psi.  Difference between steering angle and desired steering angle)

Actuators are steering angle (delta) and throttle (a). First step is to
convert way points to the vehicle coordinate system using the standard
translation and rotation equations.

Then, a polynomial is fit to the way points to find the reference line, f.
From there desired position is f(x) and desired vehicle angle (psides) 
is the f'(x)

Update equations follow basic dynamics and trigonometry. For all variables,
1 means next time step and 0 means current time step. 
      x1 = x0 + v0 * cos(psi0) * dt
      y1 = y0 + v0 * sin(psi0) * dt
      psi1 = psi0 + v0 * delta0 * dt / Lf
      v1 = v0 + a0 * dt
      cte1 = (f0-y0) + v0 * sin(epsi0) * dt
      epsi1 = (psi0-psides0) + (v0 * delta0 / Lf * dt)

One sticking point is what is the meaning of a?  
and what are the units?  IF a is the actual acceleration
in m/sec^2 then it could make sense to predict future v by adding a * dt
but if "a" is just some normalized measurement of how much
throttle and brake is applied then the conversion is less 
clear. Also, friction is ignored even though it appears
friction is built into the simulator

## Cost

Cost function was a weighted sum of various costs:

* cross track error
* vehicle in vehicle orientation
* difference in speed from reference velocity
* steering angle magnitude
* throttle/brake magnitude
* steering angle difference between t+1 and t
* throttle/brake difference btween t+1 and t

The weights were chosen emperically. Early attempts had the green desired path osciallate around the yellow reference path. This is solved by increasing the weight of the cross track error. Ocillations of the vehicle itself were solved by increasing the weight of the steering angle magnitude. In order to allow slower speeds around turns, the speed weight was kept small.  

## Time steps
Settled on dt = 0.10 mainly due to reviewer's choice. Higher values such as 0.25 were more stable and worked fine, but may have hid issues with latency calculations.  


Number of time stamps (N) chosen after dt was determined. Increased until the green path line seemed about 3 seconds in front of vehicle. 


Pre-processing of input data was minimal, except to project global map coordinates to local car coorindates.

## Latency
Latency was dealt with by first deciding a latency time period. 0.100 seconds was chosen to match the forced 100ms in the project. This ignores other sources of latency which seemed small compared to 100ms. Then the state was predicting into the future using the above update equations to calculate the new state. The same equations and assumptions were used for both the MPC updtes and the latency updates. The new state then goes into the solver. 




---


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

