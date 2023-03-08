# Requirement 9
## Convert global coordinates to a localized Eucliden system

To satisfy this requirement, I used the [latlon2local](https://www.mathworks.com/help/driving/ref/latlon2local.html) MATLAB function and converted the global coordinates of all the map nodes to Cartesian coordinates. I used the minimum longitude and longitude of all the nodes as the origin for the system so node of the nodes would have negative coordinates.

Here is a plot of the new waypoints. Notice the axes are on the order of hundreds:
![](euclidean_plot.jpg 'Printed 3D model of DeepRacer mount')