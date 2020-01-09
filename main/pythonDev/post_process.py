# import glfw modul 
#postprocessing:
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker



#*****************************************
#post processing for mass point system
n = 2 #number of bodies
data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
#data = np.loadtxt('Testmodels/solution/ANCFCable2D_bending_test.txt', comments='#', delimiter=',')

for i in range(n-1): #first body is ground object
    #plt.plot(data[:,0], 1+i+data[:,2+3*i], 'k-') #plot column 3 over column 0 (time)
    plt.plot(data[:,0], data[:,6], 'c-') #plot column i over column 0 (time)
    #plt.plot(data[:,1+3*i]+i+1, data[:,1+3*i+1], 'b-') #x-y plot
ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 
#plt.savefig("figure.png")



##show image
#nx = 100
#ny = 100
#img = np.ndarray([nx,ny,3])
#
#for ix in range(nx):
#    for iy in range(ny):
#        img[ix][iy][0] = ix/nx
#        img[ix][iy][1] = 0
#        img[ix][iy][2] = iy/ny
#
#plt.imshow(img)
#plt.show()


