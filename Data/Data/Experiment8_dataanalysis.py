import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np


def func(x, a):
    return a * x
fig = plt.figure()
ax = fig.add_subplot(111)
xdata = ("2022-12-06 13:37","2022-12-07 13:41","2022-12-11 14:05","2022-12-15 14:32","2022-12-16 14:37","2022-12-18 14:51","2022-12-19 14:58","2022-12-24 07:02","2023-01-17 22:41","2023-01-28 23:06","2023-02-15 07:16","24/02/2023 15:10","2023-03-05 11:13","2023-03-11 15:22","2023-03-21 11:09","2023-03-25 14:57","2023-04-05 15:39","2023-04-17 11:00")
acc = (16191,11947,13024,12497,14595,11276,574,9817,419,349,12182,12692,11796,14842,378,12852,13590,11662)




plt.scatter(xdata, acc,color='r')

plt.xticks(rotation=30, ha='right')

plt.xlabel('Timestamp')
plt.ylabel('VeDBA burst sum')


plt.legend(['Acceleration data from Exmoor pony in experiment 8'], loc=3)
fig.set_size_inches(11, 7)
fig.savefig('Acc.png', format='png', dpi=600)
plt.show()