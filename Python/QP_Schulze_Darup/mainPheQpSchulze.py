# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 18:00:18 2019

@author: jagov
"""

from phe import paillier as phe
import numpy as np
import pheMat 
import random
import time 

t0 = time.time()

public_key, private_key = phe.generate_paillier_keypair()
r = random.SystemRandom().randrange(1,2**16)
#a1 = public_key.encrypt(5)
#b1 = private_key.decrypt(a1)
#A1 = np.array([[1,2],[3,4],[5,6],[7,8]])
#A2 = np.array([[8,-1],[-7,5],[2,3],[-4,6]])
#A1_e = pheMat.encrypt_ndarray(public_key,A1)
#A2_e = pheMat.encrypt_ndarray(public_key,A2)

A = np.array([[1,0],[-1,0],[0,1],[0,-1]])
b = np.array([[2],[-1],[1.5],[-.5]])    

Q = np.array([[6,2],[2,4]])
Qi = np.linalg.inv(Q)
c = np.array([[2],[4]])

n = 50
mu1 = np.zeros((4,1))
mu = np.zeros((4,n+2))
eta = 1

b_e = pheMat.encrypt_ndarray(public_key,b)
c_e = pheMat.encrypt_ndarray(public_key,c)
mu1_e = pheMat.encrypt_ndarray(public_key,mu1)

t1 = time.time()

for i in range(n):
    Dg = -A @Qi @(A.T @mu[:,[i]] + c) - b
    mu[:,[i+1]] = np.maximum(np.zeros((4,1)), mu[:,[i]] + eta*Dg)
    xs = -Qi @(A.T @mu[:,[i+1]] + c)
    Zs = .5*xs.T @Q @xs + c.T @xs
    
t2 = time.time()

for i in range(n):
    tl0 = time.time()
    Dg_e = -A @Qi @(A.T @mu1_e + c_e) - b_e
    mu1_be = r*(mu1_e + eta*Dg_e)
    tl1 = time.time()
    mu1_b = np.maximum(np.zeros((4,1)),mu1_be.decrypt(private_key))
    tl2 = time.time()
    mu1_e = pheMat.encrypt_ndarray(public_key,1/r*mu1_b)
    tl3 = time.time()
    
t3 = time.time()

xs1_e = -Qi @(A.T @mu1_e + c_e)
xs1 = xs1_e.decrypt(private_key)
Zs1 = .5*xs1.T @Q @xs1 + c.T @xs1    


t4 = time.time()