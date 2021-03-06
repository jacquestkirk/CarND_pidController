## Comments from Jason
### Effect of each parameter
Parameter|direction|effect|example
| ------------- |:-------------:| -----:|-----:|
P|high|large overshoot|(https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/highP.mp4)
P|low|drifts towards the outside of the track|(https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/lowP.mp4)
I|high|drifts towards the outside of the track| (https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/HighI.mp4)
I|low|drift then sudden correction|(https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/LowI.mp4)
D|high|unstable, steering angle oscillates back and forth|(https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/HighD.mp4)
D|low|a lot of overshoot|(https://github.com/jacquestkirk/CarND_pidController/blob/master/videos/Low%20D.mp4)

Low D and large P have a similar effect of caussing the car to overshoot the 0 cte mark. Low P means the car has to get further away from the desired path before it gets pulled back in. This cause it to drift toward the outside of the curve (which actually does seem like more natural driving). Low I has a similar effect. A larger error is tolerated until P can ramp up and pull the car back in. A high D makes the system unstable, it pulls back too far each time, causing the steering angle to oscillate wildly. I'm not really sure why a high I behaves similarly to a low P. 

### Hyperparameter choice

Note: I wrote this up optimizing while running with Visual Studio and a debugger. I later realized that results are different when using bash. The writeup is based on the visual studio version, since I already have plots generated and the theory behind choosing the hyperparameters has not changed. 

I used the twiddle algorithm to decide on optimial values for P, I, and D. The square sum of the cross-track error was used as the cost function. The twiddle algorithm can be enabled by setting the useTwiddle flag in line 20 of [main.cpp.](https://github.com/jacquestkirk/CarND_pidController/blob/master/src/main.cpp), and the algorithm is found in the PID::Twiddle() function in [PID.cpp](https://github.com/jacquestkirk/CarND_pidController/blob/master/src/PID.cpp).

A reset occurs after MAX_STEPS steps (currently set to 1500). After which the next set of hyperparameters is run. 

A plot of hyperparameters and MSE over time is shown below. 
![alt text](https://github.com/jacquestkirk/CarND_pidController/blob/master/TwiddleParameters.jpg)

### Other reflections:

If I were to re-do this project I might choose a different cost function. For a smoother ride, we might peanalize sudden changes in steering angle at the expense of a higher CTE. For example, the last curve in the video is very choppy. I would probably get a car-sick driving in that car. In addition, I think drifting very close to the sides of the road should be penalized more than slight offsets in the middle of the road. Since most of the road is straight track. small offsets in the middle of the road dominate the error. Perhaps a peice-wise function is more appropriate, which could penalize very little for small offsets and a lot if the car almost touches the lane lines. 

I would also have liked to add the dt term in calculating the derivative and integral. This would allow sensor information to come in at irregular intervals. Unfortunately, I was unable to find out how to pull a time value out of the json file. 

Furthermore, the twiddle flow could be optimized for speed by stopping when a max CTE is reached. Often, while training with twiddle, the car would run off the road and start driving in circles. A max CTE, set to the width of the road, could kill this paramater set much faster. 

## Original .md

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

