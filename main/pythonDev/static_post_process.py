# import glfw modul 
#postprocessing:
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

#*****************************************
#post processing for mass point system
data = np.loadtxt('staticSolution.txt', comments='#', delimiter=',')
n = int(data[0,:].size/3) #number of bodies
data1 = np.zeros((1,n))
for i in range(n): 
    data1[0,i] = data[0,i*3]

refPos = np.linspace(1,n,n)
#for i in range(n-1): #first body is ground object
plt.plot(refPos[:], data1[0,:], 'r-') #plot column 3 over column 0 (time)

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 
plt.savefig("figure.png")

