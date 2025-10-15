"""
Classe Robot, qui s'occupe de simuler le robot
"""
from Objet import Objet
from Patte import Patte
from Moteur import Moteur
from Trajectoire import Trajectoire
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D

class Robot(Objet):
    def __init__(self, l1, l2, l3):
        """
        Paramètres :
            l1, l2, l3 : les longueurs caractéristiques des pattes du robot
        """
        alpha = np.pi/4 #Angle entre le repère du robot et le repère de la patte selon z
        self.d1, self.d2, self.d3, self.e1, self.e2 = 1, 1, 1, 0.5, 0.5 #Carcéristique de la pièce centrale du robot
        self.r1, self.r2, self.r3 = 1, 1, 0 #Distance entre le repère est la patte haute droite par apport au centre de gravité G
        
        #Patte Haute Droite
        TpatteHD = [[np.cos(alpha), -np.sin(alpha), 0, self.r1], [np.sin(alpha), np.cos(alpha), 0, self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mHD1, self.mHD2, self.mHD3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteHD = Patte(l1, l2, l3, TpatteHD, self.mHD1, self.mHD2, self.mHD3)
        
        #Patte Haute Gauche
        TpatteHG = [[np.cos(-alpha), -np.sin(-alpha), 0, self.r1], [np.sin(-alpha), np.cos(-alpha), 0, -self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mHG1, self.mHG2, self.mHG3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteHG = Patte(l1, l2, l3, TpatteHG, self.mHG1, self.mHG2, self.mHG3)
        
        #Patte Basse Droite
        TpatteBD = [[np.cos(np.pi-alpha), -np.sin(np.pi-alpha), 0, -self.r1], [np.sin(np.pi-alpha), np.cos(np.pi-alpha), 0, self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mBD1, self.mBD2, self.mBD3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteBD = Patte(l1, l2, l3, TpatteBD, self.mBD1, self.mBD2, self.mBD3)
        
        #Patte Basse Gauche
        TpatteBG = [[np.cos(np.pi+alpha), -np.sin(np.pi+alpha), 0, -self.r1], [np.sin(np.pi+alpha), np.cos(np.pi+alpha), 0, -self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mBG1, self.mBG2, self.mBG3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteBG = Patte(l1, l2, l3, TpatteBG, self.mBG1, self.mBG2, self.mBG3)
    
    def getPos():
        return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0 , 1]]
    
    def aff_PosRobot(self, q):
        """
        Affiche le robot en fonction des coordonnés articulaires q = [qHD, qHG, qBD, qBG] 
        Où : qXY désigne les coordonnés articulaite de la patte XY
            H -> Haut; B -> Bas
            D -> Droite; G -> Gauche
        """
        qHD1, qHD2, qHD3, qHG1, qHG2, qHG3, qBD1, qBD2, qBD3, qBG1, qBG2, qBG3 = q
        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        ax_echelle = 5
        ax.axes.set_xlim3d(left=-ax_echelle, right=ax_echelle)
        ax.axes.set_ylim3d(bottom=-ax_echelle, top=ax_echelle)
        ax.axes.set_zlim3d(bottom=-2, top=1)
        fig.add_axes(ax)
        
        x = [self.d1-self.e1, self.d1, self.d1, self.d1-self.e1, -self.d1+self.e1, -self.d1, -self.d1, self.e1-self.d1, self.d1-self.e1]
        y = [self.d2, self.d2-self.e2, -self.d2+self.e2, -self.d2, -self.d2, -self.d2+self.e2, self.d2-self.e2, self.d2, self.d2]
        z = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        #Patte Droite Haute
        X_trace = self.PatteHD.X_trace(qHD1, qHD2, qHD3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteHD.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xs = [float(p[0]) for p in X_trace]
        ys = [float(p[1]) for p in X_trace]
        zs = [float(p[2]) for p in X_trace]
        
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
        
        #Patte Gauche Haute
        X_trace = self.PatteHG.X_trace(qHG1, qHG2, qHG3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteHG.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xs = [float(p[0]) for p in X_trace]
        ys = [float(p[1]) for p in X_trace]
        zs = [float(p[2]) for p in X_trace]
        
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
        
        #Patte Droite Basse
        X_trace = self.PatteBD.X_trace(qBD1, qBD2, qBD3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteBD.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xs = [float(p[0]) for p in X_trace]
        ys = [float(p[1]) for p in X_trace]
        zs = [float(p[2]) for p in X_trace]
        
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
        
        #Patte Gauche Basse
        X_trace = self.PatteBG.X_trace(qBG1, qBG2, qBG3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteBG.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xs = [float(p[0]) for p in X_trace]
        ys = [float(p[1]) for p in X_trace]
        zs = [float(p[2]) for p in X_trace]
        
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
        
        corps = [list(zip(x, y, z))]
        ax.add_collection(Poly3DCollection(corps, alpha=0.5))
        ax.scatter([0, 0], [0, 0], [0, 0.01], color="red")
        plt.show()
        
        