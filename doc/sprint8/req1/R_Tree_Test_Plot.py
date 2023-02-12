# importing the required module
import matplotlib.pyplot as plt
  
x = []
y = []

for i in range(7):
    for j in range(7):
        x.append(i + 0.1*j)
        y.append(j + 0.1*i)

# plotting the points 
plt.scatter(x, y)
  
# naming the x axis
plt.xlabel('x - axis')
# naming the y axis
plt.ylabel('y - axis')
  
# giving a title to my graph
plt.title('My first graph!')
  
# function to show the plot
plt.show()