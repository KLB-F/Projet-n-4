"""
Classe gérant le modèle Géométrique
"""

import numpy as np
from numpy import matrix,cos, sin
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ModeleGeometrique():
    def __init__(self, l1:float, l2:float, l3:float):
        """
        Permet d'initialiser le modèle géométrique
        
        Paramètres : 
            l1, l2, l3 : les longueurs l1, l2, l3 du schéma cinématique
        """
        self.l1, self.l2, self.l3 = l1, l2, l3
    
    
    def X_trace(self, q1, q2, q3):
        """
        Retourne la valeur de la matrice X_trace
        
        Paramètres : 
            q1, q2, q3 : les coordonnés articulaires
        
        Retourne : 
            X_trace
        """
        return [np.matrix([
[0],
[0],
[0]]), np.matrix([
[self.l1*cos(q1)],
[self.l1*sin(q1)],
[         0]]), np.matrix([
[self.l1*cos(q1) + self.l2*cos(q1)*cos(q2)],
[self.l1*sin(q1) + self.l2*sin(q1)*cos(q2)],
[                    -self.l2*sin(q2)]]), [(self.l1 + self.l2*cos(q2) + self.l3*cos(q2 + q3))*cos(q1), (self.l1 + self.l2*cos(q2) + self.l3*cos(q2 + q3))*sin(q1), -self.l2*sin(q2) - self.l3*sin(q2 + q3)]]
    
    def MGD(self, q1, q2, q3):
        """
        Retourne la position de l'effecteur
        
        Paramètres : 
            q1, q2, q3 : les coordonnés articulaires
            
        Retourne :
            [x, y, z] : la position de l'effecteur dans 0
        """
        return [(self.l1 + self.l2*cos(q2) + self.l3*cos(q2 + q3))*cos(q1), (self.l1 + self.l2*cos(q2) + self.l3*cos(q2 + q3))*sin(q1), - self.l2*sin(q2) - self.l3*sin(q2 + q3)]
    
    def Aff_PosRobot(self, q1, q2, q3):
        """
        Permet d'afficher le MG du robot dans une configuration donnés

        Parameters
        ----------
        q1, q2, q3 : q1, q2, q3 : les coordonnés articulaires
        """
        
        X_trace = self.X_trace(q1, q2, q3)        
        # conversion en coordonnées x,y,z pour matplotlib
        xs = [float(p[0]) for p in X_trace]
        ys = [float(p[1]) for p in X_trace]
        zs = [float(p[2]) for p in X_trace]
        
        # Visualisation
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
        
        ax.view_init(elev=45, azim=45) # réglages de la vue 3D
        
        ax.set_xlabel("X (dm)")
        ax.set_ylabel("Y (dm)")
        ax.set_zlabel("Z (dm)")
        plt.show()
        
    def MGI(self, x, y, z):
        """
        Retourne les coordonnés articulaires en fonction des coordonnés opéraionelles
        
        Paramètres :
            x, y, z : Les coordonnés opérationnels à atteindres
        Retourne : 
            q1, q2, q3 : Les coordonnés articulaires du MGI
        """
        
        q1 = np.arctan2(y, x)
        
        T10 = matrix([[cos(q1), sin(q1), 0, -self.l1],[-sin(q1), cos(q1), 0, 0 ],[0, 0, 1, 0],[0, 0, 0, 1 ]])
        #T10 = np.linalg.inv(matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])@matrix([[1, 0, 0, self.l1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

        xp, yp, zp = (T10@np.transpose(matrix([x, y, z, 1])))[:-1] #On passe dans la base 1
        xp, yp, zp = float(xp), float(yp), float(zp)
        
        q3 = np.arccos(max(min(1,(xp**2+zp**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)), -1)) #Choix de configuration possible + On met des min et max pour que la valeur soit dans [-1, 1] car dans certains cas python donne par exemple 1.0000000022 > 1 -> Erreur
        q2 = +np.arctan2(zp, xp)+np.arctan2(self.l3*np.sin(q3), self.l2+self.l3*np.cos(q3))
        return q1, -q2, q3
    
    def Jacobienne(self, q1, q2, q3) -> [[float]*3]*3:
        """
        Calcul la Jacobienne du modèle
        
        Paramètres : 
            q1, q2, q3 : les coordonnés articulaire
        Retourne :
            J : la Jacobienne
        """
        r = self.l1 + self.l2*np.cos(q2) + self.l3*np.cos(q2 + q3)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

        J = np.array([
            [-r*np.sin(q1), -(self.l2*s2 + self.l3*s23)*np.cos(q1), -self.l3*s23*np.cos(q1)],
            [ r*np.cos(q1), -(self.l2*s2 + self.l3*s23)*np.sin(q1), -self.l3*s23*np.sin(q1)],
            [            0, -(self.l2*c2 + self.l3*c23),            -self.l3*c23          ]
        ])
        return J