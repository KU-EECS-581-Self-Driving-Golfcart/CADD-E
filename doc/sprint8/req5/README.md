This requirement required redesigning the model to be an open-loop simulation instead of a closed-loop simulation as previous. To communicate with the simulation that will close the model loop, I implemented ROS publishers and subscribers for the data which was previously generated by the built-in driving scenario reader. This is also the next step for enabling code generation.

I added appropriate publisher and subscriber blocks. I created custom ROS messages for the lanes, state, and radar detection information. I then importated these into MATLAB for use in the model. I also exported the car simulation as a subsystem and added ROS publishers and subscribers for it.