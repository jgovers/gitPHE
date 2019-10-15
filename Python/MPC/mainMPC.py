# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 14:02:07 2019

@author: jagov
"""

#from phe import paillier as phe
import numpy as np
#import pheMat as pheMat
#import random
import time

t0 = time.time()

#public_key, private_key = phe.generate_paillier_keypair()
#r = random.SystemRandom().randrange(1,2**16)

n = 50
mu = np.zeros((4,1))
eta = 1

LTI_A = np.array([[1,0,.1,0],[0,1,0,.1],[0,0,1,0],[0,0,0,1]])
LTI_B = np.array([[0,0],[0,0],[1,0],[0,1]])
LTI_C = np.array([[1,0,0,0],[0,1,0,0]])
LTI_D = np.array([[0,0],[0,0]])

x0 = np.array([[0],[0],[0],[0]])
WP = np.array([[20],[0],[0],[0]])

vMax = 5

wQ = np.array([[5,0,0,0],[0,5,0,0],[0,0,1,0],[0,0,0,1]])
wR = np.array([[1,0],[0,1]])

Q = np.array([[8.1729,0,5.0696,0,3.0164,0],[0,8.1729,0,5.0696,0,3.0164],[5.0696,0,5.3425,0,2.1654,0],[0,5.0696,0,5.3425,0,2.1654],[3.0164,0,2.6154,0,3.2145,0],[0,3.0164,0,2.6154,0,3.2145]])
Qi = np.linalg.inv(Q)
cx = np.array([[11.0329,0,8.2762,0],[0,11.0329,0,8.2762],[7.2710,0,5.7967,0],[0,7.2710,0,5.7967],[4.0091,0,3.4173,0],[0,4.0091,0,3.4173]])

#for i in range(n):
#    Dg = 