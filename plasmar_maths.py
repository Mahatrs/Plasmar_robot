#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

class PlasmarModel:
    """
    Contient le modèle mathématique (Physique & Cinématique) du robot PlaSMAR.
    """
    def __init__(self):
       # Paramètres physiques du robot
        self.m = 18.0          # Masse (kg)
        self.Vol = 0.0185      # Volume déplacé (m3)
        self.rho = 1000.0      # Densité eau (kg/m3)
        self.g = 9.81          # Gravité (m/s2)

        # Centre de Gravité (PG) et de Flottaison (PB)
        self.pg = np.array([0.0, 0.0, -0.05])
        self.pb = np.array([0.0, 0.0, 0.0])

        # Matrice d'Inertie Diagonale (Io)
        self.Ix = 0.096 # 0.611#
        self.Iy = 0.611
        self.Iz = 0.608

        # Coefficients Hydrodynamiques (Masse ajoutée & Amortissement)
        # Masse ajoutée
        self.Xu_dot = -1.88;
        self.Yv_dot = -18.81;
        self.Zw_dot = -18.81
        self.Kp_dot = 0.0;
        self.Mq_dot = -0.56;
        self.Nr_dot = -0.56

        # Amortissement Linéaire
        self.Xu = -12.85;
        self.Yv = -71.85;
        self.Zw = -71.85
        self.Kp = -0.38;
        self.Mq = -3.88;
        self.Nr = -3.88

        # Amortissement Quadratique
        self.Xuu = -12.85;
        self.Yvv = -1.91;
        self.Zww = -1.91
        self.Kpp = -0.38;
        self.Mqq = -1.96;
        self.Nrr = -1.96


    def S(self, v):
        """Matrice antisymétrique (Skew-symmetric) pour produit vectoriel"""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
    
    def get_M(self):
        """Retourne la matrice de Masse totale"""
        # Masse Corps (Mb)
        I_diag = np.diag([self.Ix, self.Iy, self.Iz])
        S_pg = self.S(self.pg)
        I_o = I_diag - self.m * (S_pg @ S_pg)

        Mb = np.zeros((6, 6))
        Mb[0:3, 0:3] = self.m * np.eye(3)
        Mb[0:3, 3:6] = -self.m * S_pg
        Mb[3:6, 0:3] = self.m * S_pg
        Mb[3:6, 3:6] = I_o

        # Masse Ajoutée : Ma
        Ma = -np.diag([self.Xu_dot, self.Yv_dot, self.Zw_dot,
                       self.Kp_dot, self.Mq_dot, self.Nr_dot])
        
        return Mb + Ma

    def get_C(self, nu):
        """Retourne la matrice de Coriolis"""
        u, v, w, p, q, r = nu
        nu1 = np.array([u, v, w])
        nu2 = np.array([p, q, r])

        # Cb
        I_diag = np.diag([self.Ix, self.Iy, self.Iz])
        S_pg = self.S(self.pg)
        I_corps = I_diag - self.m * (S_pg @ S_pg)

        m_S_nu1 = self.m * self.S(nu1)
        cross_nu2_pg = np.cross(nu2, self.pg)
        m_S_S_nu2_pg = self.m * self.S(cross_nu2_pg)

        Cb = np.zeros((6, 6))
        Cb[0:3, 3:6] = -m_S_nu1 - m_S_S_nu2_pg
        Cb[3:6, 0:3] = -m_S_nu1 - m_S_S_nu2_pg
        Cb[3:6, 3:6] = -self.S(I_corps @ nu2)

        # Ca
        a1 = -self.Xu_dot * u
        a2 = -self.Yv_dot * v
        a3 = -self.Zw_dot * w
        b1 = -self.Kp_dot * p
        b2 = -self.Mq_dot * q
        b3 = -self.Nr_dot * r

        Ca = np.zeros((6, 6))
        Ca[0, 4] = a3;  Ca[0, 5] = -a2
        Ca[1, 3] = -a3; Ca[1, 5] = a1
        Ca[2, 3] = a2;  Ca[2, 4] = -a1
        Ca[3, 1] = a3;  Ca[3, 2] = -a2; Ca[3, 4] = b3;  Ca[3, 5] = -b2
        Ca[4, 0] = -a3; Ca[4, 2] = a1;  Ca[4, 3] = -b3; Ca[4, 5] = b1
        Ca[5, 0] = a2;  Ca[5, 1] = -a1; Ca[5, 3] = b2;  Ca[5, 4] = -b1

        return Cb + Ca

    def get_D(self, nu):
        """Retourne la matrice d'Amortissement"""
        D_coeffs = np.array([
            abs(self.Xu) + abs(self.Xuu) * abs(nu[0]),
            abs(self.Yv) + abs(self.Yvv) * abs(nu[1]),
            abs(self.Zw) + abs(self.Zww) * abs(nu[2]),
            abs(self.Kp) + abs(self.Kpp) * abs(nu[3]),
            abs(self.Mq) + abs(self.Mqq) * abs(nu[4]),
            abs(self.Nr) + abs(self.Nrr) * abs(nu[5])
        ])
        return np.diag(D_coeffs)

    def get_g(self, eta):
        """Retourne le vecteur des forces de rappel"""
        phi = eta[3]
        theta = eta[4]
        
        W = self.m * self.g
        B = self.rho * self.g * self.Vol

        ct = math.cos(theta);
        st = math.sin(theta)
        cp = math.cos(phi);
        sp = math.sin(phi)

        k_vec = np.array([-st, ct*sp, ct*cp])

        fg = -W * k_vec
        fb = B * k_vec

        tg = np.cross(self.pg, fg)
        tb = np.cross(self.pb, fb)

        return np.concatenate((fg + fb, tg + tb))


    def get_J_inv(self, eta):
        """
        Retourne l'inverse de la Jacobienne
        """
        phi, theta, psi = eta[3], eta[4], eta[5]
        
        cpsi, spsi = math.cos(psi), math.sin(psi)
        ct, st = math.cos(theta), math.sin(theta)
        cp, sp = math.cos(phi), math.sin(phi)

        R = np.array([
            [cpsi*ct, -spsi*cp + cpsi*st*sp, spsi*sp + cpsi*st*cp],
            [spsi*ct, cpsi*cp + spsi*st*sp, -cpsi*sp + spsi*st*cp],
            [-st,     ct*sp,                 ct*cp]
        ])
        
        J1_inv = R.T

        J2_inv = np.array([
            [1, 0,      -st],
            [0, cp,  ct*sp],
            [0, -sp, ct*cp]
        ])

        J_inv = np.zeros((6, 6))
        J_inv[0:3, 0:3] = J1_inv
        J_inv[3:6, 3:6] = J2_inv
        
        return J_inv
