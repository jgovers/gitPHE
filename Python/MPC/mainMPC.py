# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 14:02:07 2019

@author: jagov
"""
#%%
#from phe import paillier as phe
import numpy as np
#import slycot
import control
import binMPC as mpc
#import pheMat 
#import random
import time
#import matplotlib
import matplotlib.pyplot as plt

#%%
t0 = time.time()

#public_key, private_key = phe.generate_paillier_keypair()
#r = random.SystemRandom().randrange(1,2**16)
T = 75
n = 50
mu = np.zeros((4,1))
eta = 1             
N = 3               # Control horizon

#%%
G = mpc.LTI()
G.A = np.array([[1,0,.1,0],[0,1,0,.1],[0,0,1,0],[0,0,0,1]])
G.B = np.array([[0,0],[0,0],[1,0],[0,1]])
G.C = np.array([[1,0,0,0],[0,1,0,0]])
G.D = np.array([[0,0],[0,0]])

x0 = np.array([[0],[0],[0],[0]])
WP = np.array([[20],[0],[0],[0]])

w = mpc.Weight()

w.Q = np.array([[5,0,0,0],[0,5,0,0],[0,0,1,0],[0,0,0,1]])    # State weight matrix Q
w.R = np.array([[1,0],[0,1]])                                # Control input weight matrix R
w.P = control.dare(G.A,G.B,w.Q,w.R)[0]

predMat = mpc.generate_prediction_matrices(G,N)
costMat = mpc.generate_cost_matrices(predMat,w,N)

vMax = 5
xBound = 5

Ac = np.array([[0,1,0,0],[0,-1,0,0],[0,0,1,0],[0,0,-1,0],[0,0,0,1],[0,0,0,-1]])
bc = np.array([[xBound],[xBound],[vMax],[vMax],[vMax],[vMax]])

#%%
x = np.zeros((4,T+1))
x[:,[0]] = x0

for k in range(T):
    c = costMat.c@(x[:,[k]]-WP)
    consMat = mpc.generate_constraint_matrices(predMat,Ac,bc,x[:,[k]],N)
    mu = np.zeros((24,1))
    Dg = np.zeros((24,n))
    
    for i in range(n):
        Dg[:,[i]] = -consMat.A@costMat.Qi@(consMat.A.T@mu + c) - consMat.b
        mu = np.maximum(np.zeros((24,1)),mu + eta*Dg[:,[i]])
    
    us = -costMat.Qi@(consMat.A.T@mu + c) 
    Zs = .5*us.T@costMat.Q@us + c.T@us
    
    u = us[0:2,]
    x[:,[k+1]] = G.A@x[:,[k]] + G.B@u
           
        
#%%
fig1, ax1 = plt.subplots(2)
ax1[0].plot(np.linspace(0,0.1*T,T+1),x[0,:])
ax1[1].plot(np.linspace(0,0.1*T,T+1),x[2,:])
ax1[0].set(xlabel='Time[s]',ylabel='X pos [m]')
ax1[0].grid()
ax1[1].set(xlabel='Time[s]',ylabel='X vel [m/s]')
ax1[1].grid()
    
#fig2, ax2 = plt.subplots()
#for i in range(24):
#    ax2.plot(np.linspace(0,n-1,n),Dg[i,])
    
    
    
    