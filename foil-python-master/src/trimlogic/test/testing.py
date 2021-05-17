
import matplotlib.pyplot as plt
x = [10, 10, 8]
# corresponding y axis values
y = [50, 70, 60]

# plotting the points
plt.plot(x, y)

# naming the x axis
plt.xlabel('x - axis')
# naming the y axis
plt.ylabel('y - axis')

# giving a title to my graph
plt.title('My first graph!')

# function to show the plot
plt.show()