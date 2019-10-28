# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 14:02:07 2019

@author: jagov
"""
#%% Import libraries
from phe import paillier as phe
import numpy as np
import control
import binMPC as mpc
import pheMat 
import random
import time
import matplotlib.pyplot as plt

#%% Init
t0 = time.time()    # Timer 0

public_key, private_key = phe.generate_paillier_keypair()   # Public and private keypair
r = random.SystemRandom().randrange(1,2**16)                # Random obfuscation variable
T = 20      # Control horizon
n = 20      # Optimization horizon
eta = 1     # Some optimiation stepsize variable
N = 3       # Control horizon

#%% Definitions 
G = mpc.LTI()   # State-space system
G.A = np.array([[1,0,.1,0],[0,1,0,.1],[0,0,1,0],[0,0,0,1]])
G.B = np.array([[0,0],[0,0],[1,0],[0,1]])
G.C = np.array([[1,0,0,0],[0,1,0,0]])
G.D = np.array([[0,0],[0,0]])

x0 = np.array([[0],[0],[0],[0]])    # Initial state
WP = np.array([[5],[0],[0],[0]])   # Waypoint

w = mpc.Weight()    # Empty weights class
w.Q = np.array([[5,0,0,0],[0,5,0,0],[0,0,1,0],[0,0,0,1]])   # State weight matrix Q
w.R = np.array([[1,0],[0,1]])                               # Control input weight matrix R
w.P = control.dare(G.A,G.B,w.Q,w.R)[0]                      # Final state matrix P, based off DARE

predMat = mpc.generate_prediction_matrices(G,N)     # Prediction matrices P and S
costMat = mpc.generate_cost_matrices(predMat,w,N)   # Cost matrices Q, inverse Q (Qi) and c

vMax = 5    # Bound on velocity
xBound = 5  # Bound on position

Ac = np.array([[0,1,0,0],[0,-1,0,0],[0,0,1,0],[0,0,-1,0],[0,0,0,1],[0,0,0,-1]])     # Constraint matrix A (Ax <= b)
bc = np.array([[xBound],[xBound],[vMax],[vMax],[vMax],[vMax]])                      # Constraint vector b

#%% MPC simulation
x = np.zeros((4,T+1))   # States
x[:,[0]] = x0           # Initial state
mu0 = np.zeros((24,1))   # Dual variables
mu0_e = pheMat.encrypt_ndarray(public_key,mu0)
Dg0 = np.zeros((24,n))

t1 = time.time()

for k in range(T):
    print('k:',k+1,'/',T)
    consMat = mpc.generate_constraint_matrices(predMat,Ac,bc,x[:,[k]],N)
    c = costMat.c@(x[:,[k]]-WP)

    b_e = pheMat.encrypt_ndarray(public_key,consMat.b)
    c_e = pheMat.encrypt_ndarray(public_key,c)
    mu_e = mu0_e
    
    for i in range(n):
        print('i:',i+1,'/',n)
        Dg_e = -consMat.A@costMat.Qi@(consMat.A.T@mu_e + c_e) - b_e
        mub_e = r*(mu_e + eta*Dg_e)
        mub = np.maximum(np.zeros((24,1)),mub_e.decrypt(private_key))
        mu_e = pheMat.encrypt_ndarray(public_key,1/r*mub)
    
    mu = mu_e.decrypt(private_key)
    us = -costMat.Qi@(consMat.A.T@mu + c) 
    Zs = .5*us.T@costMat.Q@us + c.T@us
    
    u = us[0:2,]
    x[:,[k+1]] = G.A@x[:,[k]] + G.B@u
           
t2 = time.time()
print('Elapsed time is',t2-t0,'seconds.')
        
#%% Figures
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
    
    
    
    