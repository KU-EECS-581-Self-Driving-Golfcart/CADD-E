The objective of this requirement was to correct the errors causing issues in the simulink model for simulating the car's navigation system. 

I fixed the errors in the model using the datatype conversion block. This caused the correct datatypes to propogate through the model, resolving the datatype errors within the vehicle model simulation block. 

I then made improvements to the structure of the model by separating the longitudinal and lateral control components and adding a sub-block for the collision space detection subsystem.

I also implemented a stop button which can send a boolean signal to the model during execution, thereby simulating the user of the golf cart pressing thee stop button.

The included screenshot shows the model running correctly.  
