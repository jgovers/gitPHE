# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 14:02:07 2019

@author: jagov
"""
#%% Import libraries
import numpy as np
import control
import binMPC as mpc
import time
import matplotlib.pyplot as plt

#%% Init
t0 = time.time()    # Timer 0

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
           
t1 = time.time()
print('Elapsed time is',t1-t0,'seconds.')
        
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
    
    
    
    