import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
#
fig = plt.figure()
# fig, ax = plt.subplots()
# xdata, ydata = [], []
# ln, = plt.plot([], [], 'ro')
#
# def init():
#     ax.set_xlim(0,1000)
#     ax.set_ylim(-180,180)
#     return ln,
#
# def update(frame):
#     xdata.append(frame)
#     ydata.append(np.sin(frame))
#     ln.set_data(xdata, ydata)
#     return ln,
#
# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
#                     init_func=init, blit=True)
# plt.show()
# First create some toy data:
x = np.linspace(0, 2*np.pi, 400)
y = np.sin(x**2)

# Create a figure
# plt.figure()

# # Creates a subplot
# ax = fig.subplots()
# ax.plot(x, y)
# ax.set_title('Simple plot')
#
# Creates two subplots and unpacks the output array immediately
ax1, ax2 = fig.subplots(1, 2, sharey=True)
ax1.plot(x, y)
ax1.set_title('Sharing Y axis')
ax2.scatter(x, y)

# Creates four polar axes, and accesses them through the
# returned array
# axes = fig.subplots(2, 2, subplot_kw=dict(polar=True))
# axes[0, 0].plot(x, y)
# axes[1, 1].scatter(x, y)
#
# # Share a X axis with each column of subplots
# fig.subplots(2, 2, sharex='col')
#
# # Share a Y axis with each row of subplots
# fig.subplots(2, 2, sharey='row')
#
# # Share both X and Y axes with all subplots
# fig.subplots(2, 2, sharex='all', sharey='all')
#
# # Note that this is the same as
# fig.subplots(2, 2, sharex=True, sharey=True)
plt.show()