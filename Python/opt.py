# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 18:00:18 2019

@author: jagov
"""

from phe import paillier as phe
import numpy as np




public_key, private_key = phe.generate_paillier_keypair()
a1 = public_key.encrypt(5)
b1 = private_key.decrypt(a1)

A = np.mat('1,0;-1,0;0,1;0,-1')
b = np.mat('2;-1;1.5;-.5')

b_T0 = public_key.encrypt(b[0,0])
b_T1 = public_key.encrypt(b[1,0])
b_T2 = public_key.encrypt(b[2,0])
b_T3 = public_key.encrypt(b[3,0])

Q = np.mat('6,2;2,4')
c = np.mat('2;4')

n = 50
mu = np.zeros((4,n+2))
eta = 1

for i in range(n):
    print(i)
    Dg = -A*Q.I*(A.T*mu[:,[i]]+c)-b
    mu[:,[i+1]] = np.maximum(np.zeros((4,1)),mu[:,[i]]+eta*Dg)
    xs = -Q.I*(A.T*mu[:,[i+1]]+c)
    Zs = .5*xs.T*Q*xs+c.T*xs
    