import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np


def func(x, a):
    return a * x

xdata = (0,1,2,3,4,5,6,8,11,13,14,15,16,18,19,20,21,22,23,24,25,28,29,30,31,33,34,35,36)
baseline = (0,-5,-13,-20,-25,-29,-37,-46,-62,-72,-76,-83,-88,-97,-104,-108,-113,-117,-124,-131,-133,-149,-155,-160,-166,-174,-179,-186,-188)

xdata1=(0,1,2,3,4,5,6,7,8,9,12,13,14,15,18,19,21,22,23,25,26,27,28,29,31,32,33)
dog1=(0,-3,-9,-14,-14,-14,-19,-25,-28,-30,-46,-46,-52,-55,-57,-59,-70,-70,-72,-79,-85,-83,-88,-92,-96,-103,-105)

xdata2=(0,1,3,5,7,11,12,13,17,21,22,23,27,28,29)
dog2back=(0,6,6,-5,-9,-25,-32,-27,-50,-47,-47,-56,-60,-65,-69)

xdata3=(0,1,2,8,11,12,23,27,33,35,36)
dog2chest=(0,-3,23,80,78,73,107,125,137,130,122)

xdata4=(0,17,40,47)
wisent=(0,-59,-118,-147)



fig = plt.figure()
ax = fig.add_subplot(111)

x_min = 0
x_max = 48                                #min/max values for x axis
x_fit = np.linspace(x_min, x_max, 100)   #range of x values used for the fit function

popt1, pcov = curve_fit(func, xdata, baseline)
plt.scatter(xdata, baseline,color='0.4',label="Baseline")
popt2, pcov = curve_fit(func, xdata1, dog1)
plt.scatter(xdata1, dog1,color='b',label="Dog 1 (Exp. 1)")
popt3, pcov = curve_fit(func, xdata2, dog2back)
plt.scatter(xdata2, dog2back,color='g',label="Dog 2 (Exp. 2)")
popt4, pcov = curve_fit(func, xdata3, dog2chest)
plt.scatter(xdata3, dog2chest,color='r',label="Dog 2 (Exp. 3)")
popt5, pcov = curve_fit(func, xdata4, wisent)
plt.scatter(xdata4, wisent,color='m',label="Wisent (Exp. 4)")


plt.plot(x_fit, func(x_fit, popt1),"0.4",linestyle='dashed',label='b=%5.2f' % tuple(popt1))
plt.plot(x_fit, func(x_fit, popt2),"b--",label='b=%5.2f' % tuple(popt2))
plt.plot(x_fit, func(x_fit, popt3),"g--",label='b=%5.2f' % tuple(popt3))
plt.plot(x_fit, func(x_fit, popt4),"r--",label='b=%5.2f' % tuple(popt4))
plt.plot(x_fit, func(x_fit, popt5),"m--",label='b=%5.2f' % tuple(popt5))



plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.tight_layout()

plt.xlabel('Transmissions')
plt.ylabel('Voltage change (mV)')
plt.legend()

ax.set_xlim(left=-0.1)
ax.set_xlim(right=47.1)

ax.yaxis.set_ticks_position('right')
fig.set_size_inches(13.3, 5.9)
fig.savefig('V2.png', format='png', dpi=600)
plt.show()