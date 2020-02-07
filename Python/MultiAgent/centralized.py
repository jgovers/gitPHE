# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 14:27:43 2019

@author: jagov
"""

#%matplotlib qt

#%% Import libraries
#from phe import paillier as phe
import numpy as np
import control
import binMPC as mpc
#import pheMat 
#import random
import time
import matplotlib.pyplot as plt

#%% Init
t0 = time.time()    # Timer 0

#public_key, private_key = phe.generate_paillier_keypair(n_length = 2048)   # Public and private keypair
#r = random.SystemRandom().randrange(1,2**16)                # Random obfuscation variable
T = 20      # Control horizon
n = 20      # Optimization horizon
eta = 1     # Some optimiation stepsize variable
N = 3       # Control horizon

#%% Definitions 
G1 = mpc.LTI()   # State-space system
G1.A = np.array([[1,0,.1,0],[0,1,0,.1],[0,0,1,0],[0,0,0,1]])
G1.B = np.array([[0,0],[0,0],[1,0],[0,1]])
G1.C = np.array([[1,0,0,0],[0,1,0,0]])
G1.D = np.array([[0,0],[0,0]])

G2 = mpc.LTI()   # State-space system
G2.A = np.array([[1,0,.1,0],[0,1,0,.1],[0,0,1,0],[0,0,0,1]])
G2.B = np.array([[0,0],[0,0],[1,0],[0,1]])
G2.C = np.array([[1,0,0,0],[0,1,0,0]])
G2.D = np.array([[0,0],[0,0]])

x01 = np.array([[0],[0],[0],[0]])    # Initial state
WP1 = np.array([[5],[0],[0],[0]])   # Waypoint

x02 = np.array([[0],[0],[0],[0]])    # Initial state
WP2 = np.array([[5],[0],[0],[0]])   # Waypoint

w1 = mpc.Weight()    # Empty weights class
w1.Q = np.array([[5,0,0,0],[0,5,0,0],[0,0,1,0],[0,0,0,1]])   # State weight matrix Q
w1.R = np.array([[1,0],[0,1]])                               # Control input weight matrix R
w1.P = control.dare(G1.A,G1.B,w1.Q,w1.R)[0]                      # Final state matrix P, based off DARE

w2 = mpc.Weight()    # Empty weights class
w2.Q = np.array([[5,0,0,0],[0,5,0,0],[0,0,1,0],[0,0,0,1]])   # State weight matrix Q
w2.R = np.array([[1,0],[0,1]])                               # Control input weight matrix R
w2.P = control.dare(G2.A,G2.B,w2.Q,w2.R)[0]                      # Final state matrix P, based off DARE

predMat1 = mpc.generate_prediction_matrices(G1,N)     # Prediction matrices P and S
costMat1 = mpc.generate_cost_matrices(predMat1,w1,N)   # Cost matrices Q, inverse Q (Qi) and c

predMat2 = mpc.generate_prediction_matrices(G2,N)     # Prediction matrices P and S
costMat2 = mpc.generate_cost_matrices(predMat2,w2,N)   # Cost matrices Q, inverse Q (Qi) and c

vMax1 = 5    # Bound on velocity
xBound1 = 5  # Bound on position

vMax2 = 5    # Bound on velocity
xBound2 = 5  # Bound on position

Ac1 = np.array([[0,1,0,0],[0,-1,0,0],[0,0,1,0],[0,0,-1,0],[0,0,0,1],[0,0,0,-1]])     # Constraint matrix A (Ax <= b)
bc1 = np.array([[xBound1],[xBound1],[vMax1],[vMax1],[vMax1],[vMax1]])                      # Constraint vector b

Ac2 = np.array([[0,1,0,0],[0,-1,0,0],[0,0,1,0],[0,0,-1,0],[0,0,0,1],[0,0,0,-1]])     # Constraint matrix A (Ax <= b)
bc2 = np.array([[xBound2],[xBound2],[vMax2],[vMax2],[vMax2],[vMax2]])                      # Constraint vector b

#%% MPC simulation
x1 = np.zeros((4,T+1))   # States
x1[:,[0]] = x01           # Initial state
x2 = np.zeros((4,T+1))   # States
x2[:,[0]] = x02           # Initial state

mu01 = np.zeros((24,1))   # Dual variables
mu02 = np.zeros((24,1))   # Dual variables
#mu0_e = pheMat.encrypt_ndarray(public_key,mu0)
Dg01 = np.zeros((24,n))
Dg01 = np.zeros((24,n))

t1 = time.time()

for k in range(T):
    print('k:',k+1,'/',T)
    consMat = mpc.generate_constraint_matrices(predMat,Ac,bc,x[:,[k]],N)
    c = costMat.c@(x[:,[k]]-WP)

    b_e = pheMat.encrypt_ndarray(public_key,consMat.b)
    c_e = pheMat.encrypt_ndarray(public_key,c)
    mu_e = mu0_e
    
    # On server
    for i in range(n):
        print('i:',i+1,'/',n)
        Dg_e = -consMat.A@costMat.Qi@(consMat.A.T@mu_e + c_e) - b_e
        mub_e = r*(mu_e + eta*Dg_e)
        mub = np.maximum(np.zeros((24,1)),mub_e.decrypt(private_key))
        mu_e = pheMat.encrypt_ndarray(public_key,1/r*mub)
    
    # On agent
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
    
    
    
    