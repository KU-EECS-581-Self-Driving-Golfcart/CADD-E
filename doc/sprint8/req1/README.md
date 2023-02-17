# Requirement 1
## Find closest node to GPS position using R Tree algo

I implemented the R Tree data structure in C++ [here](https://github.com/cskroonenberg/R_Tree). This data structure indexes nodes based on spacial locality. I was not able to test it on the map data because I was not able to move the map data over to C++. This will be done next sprint.

Using the test code found in `r_tree_test.cpp`, I generated an R Tree for 49 points and manually plotted the results.

Test program output (showing point placement within the tree):
![R Tree Test Output 1](R_Tree_1.png 'R Tree Test Output 1')
![R Tree Test Output 2](R_Tree_2.png 'R Tree Test Output 2')

These results are more easily visualized in an annotated scatterplot:

![R Tree Test Output Visualized](R_Tree_Results.png 'R Tree Test Output Visualized')

The code to generate the scatterplot is found in `R_Tree_Test_Plot.py`.