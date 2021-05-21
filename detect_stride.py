import matplotlib.pyplot as plt
import numpy as np
import math
import time
import matplotlib.animation as animation
from scipy.signal import find_peaks
from scipy import interpolate
import random
from operator import add

fig,axs = plt.subplots(3)

tijd =0.001
t = [0.001]
amplitude=[0.001]*len(t)

def calcstride(x):
    y=-(math.sin(x)-0.5*math.cos(1.5*x)) +random.random()
    return y


def animate(i,xpoints,ypoints):
    
    tijd = i/10
    t.append(tijd)
    amplitude.append(calcstride(tijd))

    xpoints2 = np.array(t)
    ypoints2 = np.array(amplitude)

    xpoints = np.array(t[-50:])
    ypoints = np.array(amplitude[-50:])
    peaksy, _ = find_peaks(amplitude,prominence=1)
    axs[0].cla()
    axs[0].set_xlim(tijd-5,tijd)
    
    axs[0].plot(xpoints2[peaksy],ypoints2[peaksy])
    axs[0].plot(xpoints,ypoints)
    averages = 5
    waitforpeaks=2
    if len(peaksy)==(averages*2)+1+waitforpeaks:
        
        averagelength=round((peaksy[averages*2]-peaksy[0])/averages)

        sampley=[]
        samplex=[]
        taa=[0]*averagelength
        outputsampley=[]
        
        for x in range(waitforpeaks,(averages*2)+waitforpeaks,2):
            
            samplex.append(xpoints2[peaksy[x]:peaksy[x+2]])
            sampley.append(ypoints2[peaksy[x]:peaksy[x+2]])

        for x in range(0,len(samplex)):
            newx= np.linspace(samplex[x][0],samplex[x][-1],averagelength*len(samplex[x]))
            f = interpolate.interp1d(samplex[x],sampley[x])
            interpsample = f(newx)
            print(len(interpsample))

            for i in range(0,averagelength):
                taa[i] = (interpsample[i*len(samplex[x])])

            outputsampley.append(taa)
            taa=[0]*averagelength

        average_stride=[0]*averagelength
            
        for x in range(0,averages):
                average_stride = [average_stride[i] + outputsampley[x][i] for i in range(len(outputsampley[x]))]
        average_stride[:]=[x/(averages) for x in average_stride]
       

        axs[1].plot(average_stride)
        axs[2].plot(outputsampley[1])
ani = animation.FuncAnimation(fig, animate, fargs=(t[-50:], amplitude[-50:]), interval=0)

plt.show()
    

