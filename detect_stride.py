import matplotlib.pyplot as plt
import numpy as np
import math
import time
import matplotlib.animation as animation
from scipy.signal import find_peaks
from scipy import interpolate
import random


fig,axs = plt.subplots(2)

tijd =0.001
t = [0.001]
amplitude=[0.001]*len(t)

def calcstride(x):
    y=-(math.sin(x)-0.5*math.cos(1.5*x)) +random.random()/4
    return y


def animate(i,xpoints,ypoints):
    
    tijd = i/10
    t.append(tijd)
    amplitude.append(calcstride(tijd))

    xpoints2 = np.array(t)
    ypoints2 = np.array(amplitude)

    xpoints = np.array(t[-100:])
    ypoints = np.array(amplitude[-100:])
    peaksy, _ = find_peaks(amplitude,prominence=1)
    axs[0].cla()
    axs[0].set_xlim(tijd-10,tijd)
    
    axs[0].plot(xpoints2[peaksy],ypoints2[peaksy])
    axs[0].plot(xpoints,ypoints)
    averages = 3
    if len(peaksy)==(averages*2)+1:
        
        print(peaksy)
        averagelength=round((peaksy[averages*2]-peaksy[0])/averages)
        print(averagelength)

        sampley=[]
        samplex=[]
        for x in range(0,averages*2,2):
            
            samplex.append(xpoints2[peaksy[x]:peaksy[x+2]])
            sampley.append(ypoints2[peaksy[x]:peaksy[x+2]])

#            f = interpolate.interp1d(samplex, sampley)
#            xnew = np.arange(0, averagelength)
#            sampley = f(sampley)
#            samplex = xnew

        print(sampley)
        axs[1].plot(sampley[0])
ani = animation.FuncAnimation(fig, animate, fargs=(t[-100:], amplitude[-100:]), interval=0)

plt.show()
    

