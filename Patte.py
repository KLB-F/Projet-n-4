from ModeleGeometrique import ModeleGeometrique
from Objet import Objet
from Moteur import Moteur
from Trajectoire import Trajectoire

import numpy as np

"""
Classe Patte du robot, qui s'occupe de simuler une patte
"""

class Patte(Objet, ModeleGeometrique):
    def __init__(self, l1, l2, l3, Tbase:[[float]*4], m1:Moteur, m2:Moteur, m3:Moteur):
        """
        Initalise la patte

        Parametres
        ----------
        l1, l2, l3 : les longueurs l1, l2, l3 du schéma cinématique
        Tbase : la matrice de transformation homogène du repère de la patte au repère du robot
        m1, m2, m3 : Les moteurs utilisé dans l'articulation
        """
        self.l1, self.l2, self.l3 = l1, l2, l3
        self.Tbase = Tbase
        
        self.M1, self.M2, self.M3 = m1, m2, m3
        
        self.Straj = None #Trajectoire suivie : Aucune au début
        self.posStraj = ["AEP", 0] #Rendue dans le suivie de la trajectoire
        
        # Inutile a priori #self.q1, self.q2, self.q3 = self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()
        
        super().__init__(self.l1, self.l2, self.l3) #On initialise le ModeleGeometrique
    
    def getPos(self):
        return self.Tbase
    
    def getPosEffecteurRel(self):
        """
        Renvoie la position de l'effecteur (x, y, z) relatif au repère de la patte
        """
        return self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng())
    
    def getPosEffecteur(self):
        """
        Renvoie la position de l'effecteur (x, y, z) relatif au repère du robot
        """
        return (self.Tbase*np.matrix(list(self.getPosEffecteurRel())+[1]))[:-1]
    
    def setObjEffecteurRel(self, x, y, z):
        """
        Fixe la position de la patte à atteindre dans son repère
        
        Parametres :
            x, y, z : la position à atteindre
        """
        q1, q2, q3 = self.MGI(x, y, z)
        
        self.M1.set_PosObj(q1)
        self.M2.set_PosObj(q2)
        self.M3.set_PosObj(q3)
        
    def patte_udpate(self, dt):
        """
        Fonction d'udpate de la patte
        
        Paramètres : le pas temporelle dt
        """
        self.M1.mot_update(dt)
        self.M2.mot_update(dt)
        self.M3.mot_update(dt)
        
        #Suivie de trajectoire
        if self.Straj != None:
            if self.posStraj[0] == "AEP":
                if np.linalg.norm(np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))-np.array(self.Straj.getAEP()[self.posStraj[1]])) < 0.001:
                    #On passe à la prochaine position
                    if self.posStraj[1] < len(self.Straj.getAEP())-1:
                        self.posStraj[1] += 1
                        self.setObjEffecteurRel(*self.Straj.getAEP()[self.posStraj[1]])
                    else:
                        self.posStraj = ["PEP", 0]
                        self.setObjEffecteurRel(*self.Straj.getPEP()[0])
            elif self.posStraj[0] == "PEP":
                if np.linalg.norm(np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))-np.array(self.Straj.getPEP()[self.posStraj[1]])) < 0.001:
                    #On passe à la prochaine position
                    if self.posStraj[1] < len(self.Straj.getPEP())-1:
                        self.posStraj[1] += 1
                        self.setObjEffecteurRel(*self.Straj.getPEP()[self.posStraj[1]])
                    else:
                        self.posStraj = ["PEPP", 0]
                        self.setObjEffecteurRel(*self.Straj.getPEPP()[0])
            elif self.posStraj[0] == "PEPP":
                if np.linalg.norm(np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))-np.array(self.Straj.getPEPP()[self.posStraj[1]])) < 0.001:
                    #On passe à la prochaine position
                    if self.posStraj[1] < len(self.Straj.getPEPP())-1:
                        self.posStraj[1] += 1
                        self.setObjEffecteurRel(*self.Straj.getPEPP()[self.posStraj[1]])
                    else:
                        self.posStraj = ["AEPP", 0]
                        self.setObjEffecteurRel(*self.Straj.getAEPP()[0])
            elif self.posStraj[0] == "AEPP":
                if np.linalg.norm(np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))-np.array(self.Straj.getAEPP()[self.posStraj[1]])) < 0.001:
                    #On passe à la prochaine position
                    if self.posStraj[1] < len(self.Straj.getAEPP())-1:
                        self.posStraj[1] += 1
                        self.setObjEffecteurRel(*self.Straj.getAEPP()[self.posStraj[1]])
                    else:
                        self.posStraj = ["AEP", 0]
                        self.setObjEffecteurRel(*self.Straj.getAEP()[self.posStraj[1]])
            
    def getPosAngMot(self):
        """ Retourne la position angulaire de chaque moteur (q1, q2, q3) """
        return self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()
    
    def traj_Suivie(self, trajectoire:Trajectoire):
        self.Straj = trajectoire
        self.setObjEffecteurRel(*self.Straj.getAEP()[0])