import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np


def func(x, a):
    return a * x

xdata = (0,1,2,3,4,5,6,7,8,9,10,11,12,13,14)
xdata1 = (0,3,5,6,8,10,11,12)

baseline = (0,-7,-18,-25,-28,-39,-46,-53,-60,-63,-74,-81,-85,-95,-102.00)
comet = (0,-4,-4,-7,0,-4,4,4,7,7,7,7,7,11,7)
pekka = (0,7,14,21,25,39,56,56,67,85,85,92,102,124,131)
pony = (0,-8,-11,-14,-20,-22,-25,-31) 

fig = plt.figure()
ax = fig.add_subplot(111)


popt1, pcov = curve_fit(func, xdata, baseline)
plt.scatter(xdata, baseline,color='0.4',label="Baseline")
popt2, pcov = curve_fit(func, xdata, comet)
plt.scatter(xdata, comet,color='b',label="Dog 3 (Exp. 5)")
popt3, pcov = curve_fit(func, xdata, pekka)
plt.scatter(xdata, pekka,color='g',label="Dog 4 (Exp. 6)")
popt4, pcov = curve_fit(func, xdata1, pony)
plt.scatter(xdata1, pony,color='r',label="Exmoor pony (Exp. 7)")
plt.plot(xdata, func(xdata, popt1),"0.4",linestyle='dashed',label='b=%5.2f' % tuple(popt1))
plt.plot(xdata, func(xdata, popt2),"b--",label='b=%5.2f' % tuple(popt2))
plt.plot(xdata, func(xdata, popt3),"g--",label='b=%5.2f' % tuple(popt3))

x_min = 0
x_max = 14                                #min/max values for x axis
x_fit = np.linspace(x_min, x_max, 100)   #range of x values used for the fit function
plt.plot(x_fit, func(x_fit, popt4),"r--",label='b=%5.2f' % tuple(popt4))

plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.tight_layout()

plt.xlabel('Transmissions')
plt.ylabel('Voltage change (mV)')
plt.legend()

ax.set_xlim(left=-0.1)
ax.set_xlim(right=14.1)

ax.yaxis.set_ticks_position('right')
fig.set_size_inches(15.4, 5.9)
fig.savefig('V4.png', format='png', dpi=600)

plt.show()