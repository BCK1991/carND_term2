# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

# Describe the effect each of the P, I, D components had in your implementation.

P (Proportional) : The P determines the feedback response of the filter based on the difference between target (set) value and the process (current) variable. We can call this difference 'cross term error' or simply 'error'. In short, increasing the P gain increases the speed of the control system's response, meaning the system will reach to the target value faster. However; the bigger P gain will cause the process variable to oscillate since there will be overshoots. Too big P gain will cause the system to become unstable, so it will never converge to the target value.

I (Integral) : The I determines the feedback response of the filter based on the sum of the all 'error' term over the time. The I plays important role for the control system to converge since even a small error will cause a big 'total error' over the time and depending on the magnitude of the I, will push system to converge to the target value. A high 'I' term combined with a very large set point might cause the system to go into the situtation called 'integral windup'. (more in : https://en.wikipedia.org/wiki/Integral_windup)

D (Derivative) : The D determines the feedback response of the filter based on the difference between the current error term and the error term from the one sample earlier (in discrete systems), or simply the rate of change of the process variable. The 'D' term will determine how strongly the system will respond to the abrupt(er) changes in the error term. If the 'D' is big in magnitude, the system will tend to go unstable due to even a small noise.

# Describe how the final hyperparameters were chosen.

Firstly, I tried to values from PID lectures [PID - (0.2 3.0 0.01)] as initial values to run around the track to optimize further with my twiddle function. However, it was impossible to even reach to the first turn, the car became unstable and moved out of the track. My attempts to correct it manually (without any feedback from my twiddle) has failed.
At this point, I started to check Udacity Forum and tried to find a reasonable value that would somehow keep my car in the track enough time to get some values out of the twiddle function. Followings are chosen: [PID - (0.25, 0.005, 2.0)]. These values also did not help long enough to get multiple twiddle feedback. Based on my observation on the behaviour of the car, I tried to manually fine-tune the values in multiple steps: \s\s

  |//pid.Init(0.25, 0.005, 2.0);  -> system not reacting on time, counterbalancing with increasing D   
  |...   
  |...   
  |//pid.Init(0.25, 0.005, 3.0); -> overshoot observed, counterbalancing with decreasing P   
  |...   
  |...   
  |//pid.Init(0.15, 0.005, 3.0); ->overshoot and integral windup observed, counterbalancing with decreasing P and decreasing I   
  |...   
  |...   
  |//pid.Init(0.12, 0.001, 3.0); -> first set of values that allows the car to finish the track.   
  
  From this point on, I ran the track multiple times with each value and collected debug prints from my twiddle function. I allocated 50 cycles for the new control values to dwell and 600 cycles to observe the behavior. Based on the report, I tried to fine tune the coefficients and re-ran the test. I collected 4 reports and cross-checked error values among them at specific sampling points to find the final values.   
  Method:   
  1- Set/Tweek P,I,D   
  2- Run over the track once   
  3- Collect data points from the twiddle (report 1-2-3-4 attached to the submission)   
  4- Repeat 1,2,3 for 4 times   
  5- Cross check between reports   
     
**The final input pid.Init(0.145, 0.001, 3.53);  
  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 





