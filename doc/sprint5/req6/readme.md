The objective for this requirement was to understand how high-level actuator commands (steering  angle and acceleration) can be transformed into low-level actuator commands (PWM duty cycles). 

I read research articles about how to accomplish this task from a self-driving car perspective [1,2]. The typical control method for the actuators in proportional-integral-derivative control (PID). PID control reacts to errors like disturbances and parameter changes in order online in order to maintain desired functioning of  the system [4]. The control law is a function of the error signal, its derivative, and its integral, with associated gains for each which must be identified.

To identify the controller gains, we create a model of the system then optimize the gains accordingly. In [2], the CIFER software package was used to perform identification from measurements of how the steering angle changed with the PWM cycle. The resulting transfer function was used to create a feedback loop in SIMULINK, which could tune the PID controller.

On the other hand, MATLAB has the built-in system identification toolbox [6]. The identified system can be easily used in the PID Tuner app [7] in order to obtain the controller gains.

Finally, the resulting controller should be tested with a different data set than from which it was derived, as demonstrated in [2]. The rise time, or the time which the system takes to go from 10% to 90% of the set point, can be verified to meet real-time requirements.

We will perform this process twice, once for steering and once for throttle/braking. The throttle/braking system is only slightly complicated by the fact there is one input (acceleration) and two outputs (brake PWM cycle, throttle PWM cycle). Simply, if the acceleration is positive, we will set the throttle setpoint to the acceleration and the brake setpoint to zero. Conversely, if the acceleration is negative, we will set the throttle setpoint to zero and the brake setpoint to the acceleration.


[1] "Towards Fully Autonomous Driving: Systems and Algorithms"; Levinson et al (2011)
[2] "Implementation of Steering-by-Wire Control System for Electric Golf Cart"; Phetnok et al (2019)
[3] "Self-driving car ISEAUTO for research and education"; Sell et al (2018)
[4] "Perception, Planning, Control, and Coordination for Autonomous Vehicles"; Pendleton et al (2017)
[5] "Comprehensive Identiﬁcation from FrEquency Responses"; San Jose State University; https://www.sjsu.edu/researchfoundation/resources/flight-control/cifer.php
[6] "System Identification Toolbox"; MATLAB; https://www.mathworks.com/help/ident/index.html?s_tid=hc_product_card
[7] "PID Tuner"; MATLAB; https://www.mathworks.com/help/control/ref/pidtuner-app.html