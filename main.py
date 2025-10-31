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

l1, l2, l3 = [1.776, 0.982, 0.08] #En dm

robot = Robot(l1, l2, l3)

m1, m2, m3 = Moteur(0, -np.pi/2, np.pi/2, 1.04), Moteur(0, -np.pi/2, np.pi/2, 1.04), Moteur(0, -np.pi/2, np.pi/2, 1.04)

patte = Patte(l1, l2, l3,[[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]], m1, m2, m3)

traj = Trajectoire([np.array(patte.MGD(0, 0.05, -0.1)), np.array(patte.MGD(0, 0.9, 0)), np.array(patte.MGD(0, -0.9, 0)),np.array(patte.MGD(0, 0.25, -0.5))], [True, True, False, False,], 0.1)

robot.robot_setTraj(traj)


#robot.aff_RobotAnim(0.15, 400)

fig, axs = plt.subplots(5, sharex=True)

q0, q1, q2 = [], [], []
q3, q4, q5 = [], [], []
RPb0, RPb1 = [],[]
BP0, BP1 = [],[]
TN0, TN1 = [],[]
posS0, posS1 = [], []
T = []
for i in range(500):
    q = robot.PatteHG.getPosAngMot()
    q0.append(q[0])
    q1.append(q[1])
    q2.append(q[2])
    q = robot.PatteBG.getPosAngMot()
    q3.append(q[0])
    q4.append(q[1])
    q5.append(q[2])
    RPb0.append(robot.PatteHG.signal_getRP())
    RPb1.append(robot.PatteBG.signal_getRP())
    robot.robot_udpate(0.5)
    T.append(i*0.5)
    BP0.append(robot.PatteHG.Straj.getBP()[robot.PatteHG.posStraj[1]])
    BP1.append(robot.PatteBG.Straj.getBP()[robot.PatteBG.posStraj[1]])
    TN0.append(robot.PatteHG.signal_getTN())
    TN1.append(robot.PatteBG.signal_getTN())
    posS0.append(robot.PatteHG.posStraj[1])
    posS1.append(robot.PatteBG.posStraj[1])

axs[0].plot(T, q1, label="q1 - HG")
axs[0].plot(T, q2, label="q2 - HG")
axs[0].plot(T, q4, label="q1 - BG")
axs[0].plot(T, q5, label="q1 - BG")
axs[1].plot(T, RPb0, label="HG")
axs[1].plot(T, RPb1, label="BG")

axs[2].plot(T, BP0, label="HG")
axs[2].plot(T, BP1, label="BG")

axs[3].plot(T, TN0, label="HG")
axs[3].plot(T, TN1, label="BG")

axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[3].legend()

axs[2].set_title("RP")
axs[1].set_title("RP poss.")
axs[3].set_title("TN")

axs[4].plot(T, posS0, label="HG")
axs[4].plot(T, posS1, label="BG")
axs[4].legend()
axs[4].set_title("pos. Traj")
axs[4].axhline(y=1.5, color='red')

plt.show()