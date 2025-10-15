# -*- coding: utf-8 -*-
"""
        ································································
        :░█▀█░█▀▄░█▀█░▀▀█░█▀▀░▀█▀░░░█▀▄░█▀█░█▀▄░█▀█░▀█▀░▀█▀░▄▀▄░█░█░█▀▀:
        :░█▀▀░█▀▄░█░█░░░█░█▀▀░░█░░░░█▀▄░█░█░█▀▄░█░█░░█░░░█░░█\█░█░█░█▀▀:
        :░▀░░░▀░▀░▀▀▀░▀▀░░▀▀▀░░▀░░░░▀░▀░▀▀▀░▀▀░░▀▀▀░░▀░░▀▀▀░░▀\░▀▀▀░▀▀▀:
        ································································
······················································································
:░█▄█░█▀▀░▀█▀░░░█░█▄█░█▀▀░█▀▄░░░█▀▀░▀█▀░░░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░▀█▀░█▀▄░█▀▀:
:░█░█░█░█░░█░░▄▀░░█░█░█░█░█░█░░░█▀▀░░█░░░░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░░█░░█▀▄░█▀▀:
:░▀░▀░▀▀▀░▀▀▀░▀░░░▀░▀░▀▀▀░▀▀░░░░▀▀▀░░▀░░░░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀▀▀░▀░▀░▀▀▀:
······················································································


Code de la partie MGD/MGI et calcul de Trajectoire

Responsable : Le Bellec Killian
"""

import numpy as np
from Robot import Robot
import matplotlib.pyplot as plt
from Robot import Robot
from Trajectoire import Trajectoire
from Moteur import Moteur
from Patte import Patte

l1, l2, l3 = [0.7, 0.8, 0.5]

robot = Robot(l1, l2, l3)

m1, m2, m3 = Moteur(0, -np.pi/2, np.pi/2, 0.15), Moteur(0, -np.pi/2, np.pi/2, 0.15), Moteur(0, -np.pi/2, np.pi/2, 0.15)

patte = Patte(l1, l2, l3, [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]], m1, m2, m3)

traj = Trajectoire([np.array([0.7, 0, -1.3]), np.array([0.2, 0, -0.8]), np.array([1.1449747468305833, 1.1449747468305833, 0.919238815542512]),np.array([2.0, 0, -0.0])], [False, True, True], 0.1)
#traj.aff_Traj()

#robot.aff_PosRobot([np.pi/4, np.pi/3, 0, -np.pi/4, np.pi/3, 0, -np.pi/4, np.pi/3, 0, np.pi/4, np.pi/3, 0])


"""
patte.traj_Suivie(traj)

X, Y, Z = [], [], []
T = []

dt = 0.02

for i in range(5000):
    patte.patte_udpate(dt)
    x, y, z = patte.getPosEffecteurRel()
    X.append(x)
    Y.append(y)
    Z.append(z)
    T.append(i*dt)

print(traj.getPEP())

plt.clf()
plt.plot(T, X, label="x = f(t)")
plt.plot(T, Y, label="y = f(t)")
plt.plot(T, Z, label="z = f(t)")
plt.legend()
plt.show()"""
