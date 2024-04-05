from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt
import time



my_data = genfromtxt('small_track.csv', delimiter=',')
my_data = my_data[:,[1,2]]
my_data = my_data[1:,:]
print(my_data)

# fig = plt.figure()
# plt.ion()
#plt.legend(loc='upper left')
plt.scatter(my_data[:,0],my_data[:,1], s=10, c='r', marker="o", label='landmark')
# plt.plot(x,y) 
plt.show()
