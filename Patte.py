from ModeleGeometrique import ModeleGeometrique
from Objet import Objet
from Moteur import Moteur
from Trajectoire import Trajectoire

import csv
import numpy as np

"""
Classe Patte du robot, qui s'occupe de simuler une patte
"""

class Patte(Objet, ModeleGeometrique):
    def __init__(self, l1:float, l2:float, l3:float, Tbase:[[float]*4], m1:Moteur, m2:Moteur, m3:Moteur):
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
        self.posStraj = [True, 0, False, False] #Rendue dans le suivie de la trajectoire
        
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
              if np.linalg.norm(np.array(self.Straj.getTraj()[self.posStraj[1]])-np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))) < 0.01: #Distance < epsilon
                  posFut = self.posStraj[1] + 1
                  if len(self.Straj.getTraj())-1 < posFut:
                      posFut = 0
                  
                  futBP = self.Straj.getBP()[posFut]
                  if futBP == True and self.posStraj[0] == False:
                      self.posStraj[3] = True
                  elif futBP == False and self.posStraj[0] == True:
                      if self.posStraj[2] == False: 
                          return None
                      self.posStraj[2] = False
                      self.posStraj[3] = False
                      
                  self.posStraj[1] = posFut                      
                  self.posStraj[0] = futBP
                  self.setObjEffecteurRel(*self.Straj.getTraj()[self.posStraj[1]])
        
    
    def getPosAngMot(self):
        """ Retourne la position angulaire de chaque moteur (q1, q2, q3) """
        return self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()
    
    def traj_Suivie(self, trajectoire:Trajectoire, decalage=False):
        """
        Permet de faire suivre une trajectoire au Robot
        
        Paramètres : 
            Trajectoire : la trajectoire à suivre
            decalage : permet de fixer s'il y a un décalage
        """
        self.Straj = trajectoire
        
        if decalage == False:
              self.setObjEffecteurRel(*self.Straj.getTraj()[0])
        else:
            #Trouve le dernier RP
            maxi = 0 #Hypothèse maxi > 0
            for i in range(len(self.Straj.getBP())):
                if self.Straj.getBP()[i] == True:
                    maxi = i
            self.posStraj[1] = maxi
            self.setObjEffecteurRel(*self.Straj.getTraj()[maxi])
            self.posStraj[3] = True
        
    def signal_setRP(self, signal:bool) -> None:
        """ Permet d'indiquer si elle à le droit de commencer son mouvement de RP"""
        self.posStraj[2] = signal
    
    def signal_getTN(self) -> bool:
        """ Indique si la patte à finie son mouvement de TN"""
        return self.posStraj[3]

    def signal_getRP(self) -> bool:
        return self.posStraj[0]        
        
    def patte_exportTraj(self, NomFichier:str):
        """ Permet d'exporter la trajectoire de la patte
        
        Paramètres : 
            - NomFichier : Le nom du fichier
        Retourne : 
            - Un fichier csv portant le nom donné et contenant les information de la trajectoire
        """
        R = []
        for i in range(len(self.Straj.getTraj())):
            R.append(list(self.MGI(*self.Straj.getTraj()[i]))+[self.Straj.getBP()[i]])
        
        with open(NomFichier+".csv", 'w', newline='') as csvFichier:
            ecriture = csv.writer(csvFichier, delimiter=';',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            ecriture.writerows(R)
        
        
        