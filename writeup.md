# CarND-Controls-MPC writeup
Self-Driving Car Engineer Nanodegree Program

---

## Notes
* Had to run the simulator under host windows, and code under linux VM, since simulator was unusably slow in VM.
* Used this to link VM to host: https://discussions.udacity.com/t/running-simulator-on-windows-and-code-on-ubuntu/255869
* Based most of my code on the MPC lessons and the MPC quiz at https://github.com/udacity/CarND-MPC-Quizzes
* Lots of forum inspiration for this project, referred to forums many times
* Yellow reference line behaves erratically sometimes, seems due to latency and high turn rates, see https://discussions.udacity.com/t/yellow-line-behaves-erratically-with-latency/482087, https://discussions.udacity.com/t/why-my-yellow-line-keep-swing-when-drive-speed-is-big/350140

[//]: # (Image References)
[image1]: ./MPC.PNG
[image2]: ./MPC2.PNG
[image3]: ./MPC3.PNG

![alt text][image1]
![alt text][image2]
![alt text][image3]

## The Model
I used the same kinematic model as in the MPC quiz/lessons. This is a simple model that does not account for dynamics, but a simulation doesn't fully account for dynamics anyway and it is close enough for this purpose. The state of this model has position (x,y) heading or yaw (psi) and velocity (v). It could probably be expanded to include d(psi)/dt. The actuators are the steering and throttle of the vehicle.

Update equations capture the effect of actuators on the kinematic propagation of state in time. These equations are found in class FG_eval, line 121-126 of MPC.cpp. They are structured as contraints, i.e. The future value of state must be equal to current state propagated through kinematics and actuation.

## Timestep Length and Elapsed Duration (N & dt)
N x dt is the horizon time, which is how long you are looking forward in time for your model. I eventually chose 2 seconds, as this is enough to capture upcoming cornering, without taking to long to optimise or allowing model error to become a problem. Over longer horizons the model will lose accuracy, complexity and processing time increases, and the reference line does not always stay in the middle of the road as noted above - predicting over a longer horizon when the reference is off is counter productive. Shorter horizons do not capture enough of the required future behaviour to work well.

I chose a reference speed of 60mph in order to maintain control of the vehicle. At this speed, a reaction time of 100ms seems to be sufficient, so I used this as my dt. N is then calculated as 20. This seemed to work ok in practise, though I'm sure it could be tuned and optimised for better performance.

## Polynomial Fitting and MPC Preprocessing
The cross-track-error and heading error are much simpler to calculate in the car reference frame (especially if you make a few assumptions as I do) so I first moved everything into the car reference frame.

Car position and heading are set to zero, and velocity as reported from the simulator is in miles per hour. Calculations for kinematics are done in SI units, so v is converted to m/s before anything else.

Waypoints are transformed though translation and rotation into the car reference frame as below:

waypoint_car(x) = cos(psi) * (waypoint_map(x)-car_map(x)) + sin(psi) * (waypoint_map(y)-car_map(y));

waypoint_car(y) = -sin(psi) * (waypoint_map(x)-car_map(x)) + cos(psi) * (waypoint_map(y)-car_map(y));

A third order polynomial is fitted to waypoints. Making the assumption that the heading of the car does not deviate significantly from the desired heading (must initialise well, poor recovery from significant heading errors!), the cross track error is simply the polynomial evaluated at x=0, which is the car x position in the car frame. Similarly, the car frame and this assumption simplifies the heading error calculation since the derivative of the polynomial is just the linear coefficient, because all other terms are cancelled by x=0. The heading in the car reference frame is zero, so the heading error is just -atan(linear coefficient).

## Model Predictive Control with Latency
The project requires a 100ms control latency. I dealt with this by propagating the state and errors forward by 100ms though the kinematic equations with actuation before submitting to the MPC solver. When the delay is implemented, the control output from the solver are for the state the car is in when it recieves the controls. Further work could be done here by accounting for computional delay of the solver. Also, I think the wobbling of the yellow line when turning quickly has something to do with delays, but I haven't fully figured that out yet.

The cost function took some tuning to get right. I started with defualt costs from the MPC quiz: cost due to the error values, speed - reference speed, actuation, and changing actuation. This "worked", but was very unstable. First I increased the cost on steering angle, and on the gap between sequential actuations to make the controller stable. I also wanted a way to make the car slow down around corners, so I tried adding a cost due to throttle x steering. This improved performance somewhat. However, the better option was adding a high cost to speed x heading x heading (multiply heading twice to make it more punishable than speed). This results in a smooth drive with reasonable performance.
