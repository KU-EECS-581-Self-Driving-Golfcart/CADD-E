# Requirement 10
## Read in target location from UI ROS publisher

To satisfy this requirement, I created a ROS publisher in the main control loop which reads in a 2-character string published from the UI server

The first character represents the target hole [1-9]
The second character denotes the location we want to go to at that hole:
- "H" = Hole
- "B" = Black tee box
- "R" = Bronze tee box
- "S" = Silver tee box
- "G" = Gold tee box

Here is a screenshot of test data being sent from the UI (left) to the main control loop (right):
![Target location pubsub output](ros_loc_pubsub.png 'Target location pubsub output')