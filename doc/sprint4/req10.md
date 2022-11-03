# Req 10

Between the two broad categories of graph search based and interpolating curve based path planning, I have chosen to take the interpolating curve based approach for our path planning module. Beyond this, I have determined that using Frenet Frames for path planning is in our best interest. This method of path planning takes a reference path and generates a set of terminal states and paths to these terminal states using 4th or 5th order polynomials. We can then choose the optimal path based on a cost function which would avoid obstacle collision, minimize reference path deviation, etc.

Some relevant resources:
- [This explanatory video](https://www.youtube.com/watch?v=DhP3jiC9YX0)
- [This MatLab object](https://www.mathworks.com/help/nav/ref/trajectoryoptimalfrenet.html)