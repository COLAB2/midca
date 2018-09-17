import numpy as np
import matplotlib.pyplot as plt
 
# data to plot
n_groups = 2
means_frank = (100, 100)
means_guido = (100, 54)
 
# create plot
fig, ax = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.35
opacity = 0.8
 
rects1 = plt.bar(index, means_frank, bar_width,
                 alpha=opacity,
                 color='b',
                 label='EXAgent')
 
rects2 = plt.bar(index + bar_width, means_guido, bar_width,
                 alpha=opacity,
                 color='g',
                 label='GMAgent')
 
plt.xlabel('Problems')
plt.ylabel('Performance (% of goal completed)')
# plt.title('Scores by person')
# plt.xticks(index + bar_width, ('A', 'B', 'C', 'D'))
plt.legend()
 
plt.tight_layout()
plt.show()