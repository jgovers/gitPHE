# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 14:27:43 2019

@author: jagov
"""

#%matplotlib qt

#%% Import libraries
import numpy as np
import scipy
import control
import binMPC as mpc
import time
import matplotlib.pyplot as plt

#%% Init
t0 = time.time()    # Timer 0

T = 60     # Control horizon
n = 20      # Optimization horizon
eta = 1     # Some optimiation stepsize variable
N = 3       # Control horizon

#%% Definitions 
G1 = mpc.LTI()   # State-space system
G1.A = np.array([[1,.1],[0,1]])
G1.B = np.array([[0],[1]])
G1.C = np.array([[1,0]])
G1.D = np.array([[0]])

G2 = mpc.LTI()   # State-space system
G2.A = np.array([[1,.1],[0,1]])
G2.B = np.array([[0],[1]])
G2.C = np.array([[1,0]])
G2.D = np.array([[0]])

x01 = np.array([[2],[0]])    # Initial state
WP1 = np.array([[15],[0]])   # Waypoint

x02 = np.array([[0],[0]])    # Initial state
WP2 = np.array([[12.5],[0]])   # Waypoint

vMax1 = 3    # Bound on velocity
xBound1 = 150  # Bound on position

vMax2 = 15    # Bound on velocity
xBound2 = 150  # Bound on position

w1 = mpc.Weight()    # Empty weights class
w1.Q = np.array([[.5,0],[0,1]])   # State weight matrix Q
w1.R = np.array([[1]])                               # Control input weight matrix R
w1.P = control.dare(G1.A,G1.B,w1.Q,w1.R)[0]                      # Final state matrix P, based off DARE

w2 = mpc.Weight()    # Empty weights class
w2.Q = np.array([[.5,0],[0,1]])   # State weight matrix Q
w2.R = np.array([[1]])                                # Control input weight matrix R
w2.P = control.dare(G2.A,G2.B,w2.Q,w2.R)[0]                      # Final state matrix P, based off DARE

predMat1 = mpc.generate_prediction_matrices(G1,N)     # Prediction matrices P and S
costMat1 = mpc.generate_cost_matrices(predMat1,w1,N)   # Cost matrices Q, inverse Q (Qi) and c

predMat2 = mpc.generate_prediction_matrices(G2,N)     # Prediction matrices P and S
costMat2 = mpc.generate_cost_matrices(predMat2,w2,N)   # Cost matrices Q, inverse Q (Qi) and c

costMat = mpc.CostMatrix()
costMat.H = scipy.linalg.block_diag(costMat1.H,costMat2.H)
costMat.Hi = np.linalg.inv(costMat.H)
costMat.F = scipy.linalg.block_diag(costMat1.F,costMat2.F)

predMat = mpc.PredictionMatrix()
predMat.P = scipy.linalg.block_diag(predMat1.P,predMat2.P)
predMat.S = scipy.linalg.block_diag(predMat1.S,predMat2.S)

Ac1 = np.array([[1,0],[-1,0],[0,1],[0,-1]])     # Constraint matrix A (Ax <= b)
AcBar1 = np.kron(np.eye(N+1),Ac1)
bc1 = np.array([[xBound1],[xBound1],[vMax1],[vMax1]])                      # Constraint vector b
bcBar1 = np.kron(np.ones((N+1,1)),bc1)

Ac2 = np.array([[1,0],[-1,0],[0,1],[0,-1]])     # Constraint matrix A (Ax <= b)
AcBar2 = np.kron(np.eye(N+1),Ac2)
bc2 = np.array([[xBound2],[xBound2],[vMax2],[vMax2]])                      # Constraint vector b
bcBar2 = np.kron(np.ones((N+1,1)),bc2)

AcCoupling1 = np.array([[-1,0]])
AcCoupling2 = np.array([[1,0]])
AcCouplingBar1 = np.kron(np.eye(N+1),AcCoupling1)
AcCouplingBar2 = np.kron(np.eye(N+1),AcCoupling2)
AcCouplingBar = np.append(AcCouplingBar1,AcCouplingBar2,axis=1)

bcCoupling = np.array([[0]])
bcCouplingBar = np.kron(np.ones((N+1,1)),bcCoupling)

AcBar = scipy.linalg.block_diag(AcBar1,AcBar2)
AcBar = np.append(AcBar,AcCouplingBar,axis=0)

bcBar = np.append(bcBar1,bcBar2,axis=0)
bcBar = np.append(bcBar,bcCouplingBar,axis=0)

#%% MPC simulation
x1 = np.zeros((2,T+1))   # States
x1[:,[0]] = x01           # Initial state
x2 = np.zeros((2,T+1))   # States
x2[:,[0]] = x02           # Initial state

x = np.append(x1,x2,axis=0)

t1 = time.time()

consMat = mpc.ConstraintMatrix()
consMat.A = AcBar@predMat.S

nMu = (N+1)*bc1.shape[0]+(N+1)*bc2.shape[0]+(N+1)*bcCoupling.shape[0]
mu = np.zeros([nMu,1])
mutest = np.zeros([nMu,T])
mutest2 = np.zeros([nMu,T])

for k in range(T):
    print('k:',k+1,'/',T)
    consMat.b = bcBar-AcBar@predMat.P@x[:,[k]]
    xTilde = np.append(x1[:,[k]]-WP1,x2[:,[k]]-WP2,axis=0)
    c = costMat.F@(xTilde)

    b = consMat.b
    mu = np.zeros([nMu,1])

#    mutest[:,[k]] = mu
#    mu[0:(N*bc1.shape[0]),] = mu[bc1.shape[0]:((N+1)*bc1.shape[0]),]
#    mu[((N+1)*bc1.shape[0]):(((N+1)*bc1.shape[0])+(N*bc2.shape[0])),] = mu[(((N+1)*bc1.shape[0])+bc2.shape[0]):(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])),]
#    mu[(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])):(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])+(N*bcCoupling.shape[0])),] = mu[(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])+bcCoupling.shape[0]):(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])+((N+1)*bcCoupling.shape[0])),]
#    mu[(N*bc1.shape[0]):((N+1)*bc1.shape[0]),] = 0
#    mu[(((N+1)*bc1.shape[0])+(N*bc2.shape[0])):(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])),] = 0
#    mu[(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])+(N*bcCoupling.shape[0])):(((N+1)*bc1.shape[0])+((N+1)*bc2.shape[0])+((N+1)*bcCoupling.shape[0])),] = 0
#    mutest2[:,[k]] = mu

    # On server
    for i in range(n):
        print('i:',i+1,'/',n)
        Dg = -consMat.A@costMat.Hi@(consMat.A.T@mu + c) - b
        mub = mu + eta*Dg
        mu = np.maximum(np.zeros_like(mub),mub)
    
    # On agent
    us = -costMat.Hi@(consMat.A.T@mu + c) 
    Zs = .5*us.T@costMat.H@us + c.T@us
    
    u1 = us[0:1,]
    u2 = us[N:N+1,]
    
    x1[:,[k+1]] = G1.A@x1[:,[k]] + G1.B@u1
    x2[:,[k+1]] = G2.A@x2[:,[k]] + G2.B@u2
    
    x[0:2,[k+1]] = x1[:,[k+1]]
    x[2:4,[k+1]] = x2[:,[k+1]]
       
t2 = time.time()
print('Elapsed time is',t2-t0,'seconds.')
        
#%% Figures
fig1, ax1 = plt.subplots(2)
ax1[0].plot(np.linspace(0,0.1*T,T+1),x1[0,:])
ax1[0].plot(np.linspace(0,0.1*T,T+1),x2[0,:])
ax1[1].plot(np.linspace(0,0.1*T,T+1),x1[1,:])
ax1[1].plot(np.linspace(0,0.1*T,T+1),x2[1,:])
ax1[0].set(xlabel='Time[s]',ylabel='X pos [m]')
ax1[0].grid()
ax1[1].set(xlabel='Time[s]',ylabel='X vel [m/s]')
ax1[1].grid()
    
#fig2, ax2 = plt.subplots()
#for i in range(24):
#    ax2.plot(np.linspace(0,n-1,n),Dg[i,])
    
    
    
    