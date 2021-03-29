import numpy as np                                          #used for the creating of array (example data)
from scipy.ndimage.filters import uniform_filter1d          #moving average filter
import matplotlib.pyplot as plt                             #libary to plot
from mpl_toolkits import mplot3d                            #libary to plot 3d
import pandas as pd                                         #try out for the animation part but he only used it for importing csv files
from matplotlib.animation import FuncAnimation              #matplot animation par
from itertools import count                                 #used to load in data 1 by 1
from pykalman import KalmanFilter                           #kalman filters


#global variables
N = 10                                                      #setting of the moving average filter
x = np.random.random(500)                                   #example data generation
y = np.random.random(500)
z = np.random.random(500)
kf = KalmanFilter(initial_state_mean=0, n_dim_obs=2)
index = count()

def my_filter():                                            #movingaverage filter + plots
     x_co = uniform_filter1d(x, size=N)
     y_co = uniform_filter1d(y, size=N)
     z_co = uniform_filter1d(z, size=N)
     my_plot(x, x_co, 'x_co')
     my_plot(y, y_co, 'y_co')
     my_plot(z, z_co, 'z_co')
     my_3dplot(x_co, y_co, z_co)
     #my_live3dplot(x_co, y_co, z_co)

def my_kalmanfilter():                                      #kalman filter + plots werkt niet
    x_ka = kf.em(x)
    y_ka = kf.em(y)
    y_ka = kf.em(y)
    my_plot(x, x_ka, 'x_ka')
    my_plot(y, y_ka, 'y_ka')
    my_plot(z, z_ka, 'z_ka')
    my_3dplot(x_ka, y_ka, z_ka)


def my_plot(x_1, y_1, title):                               #create the plots
    plt.plot(x_1)
    plt.plot(y_1)
    plt.xlabel(title)
    plt.show()


def my_3dplot(xs, ys, zs):                                  #create 3d plots with all the data
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.plot3D(xs, ys, zs)
    plt.show()


#my_filter()
my_kalmanfilter()
# def my_live3dplot(xs, ys, zs):
#     ani = FuncAnimation(plt.gcf(), animate, interval=500)
#
# def animate(i,xs,ys,zs):
#     ax = plt.axes(projection='3d')
#     x = xs
#     y = ys
#     z = zs
#
#     plt.cla()
#     ax.plot3D(x, y, z, 'red')
#     plt.tight_layout()
#     plt.show()
#
#     # plt.legend(loc='upper left')
#     # plt.tight_layout()









