# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 14:27:43 2019

@author: jagov
"""

#%matplotlib qt

#%% Import libraries
#from phe import paillier as phe
import numpy as np
import scipy
import control
import binMPC as mpc
#import pheMat 
import random
import time
import matplotlib.pyplot as plt

#%% Init
t0 = time.time()    # Timer 0

#public_key, private_key = phe.generate_paillier_keypair(n_length = 2048)   # Public and private keypair
r1 = random.SystemRandom().randrange(1,2**16)                # Random obfuscation variable
r2 = random.SystemRandom().randrange(1,2**16)                # Random obfuscation variable
T = 20      # Control horizon
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

x01 = np.array([[0],[0]])    # Initial state
WP1 = np.array([[5],[0]])   # Waypoint

x02 = np.array([[-5],[0]])    # Initial state
WP2 = np.array([[2],[0]])   # Waypoint

w1 = mpc.Weight()    # Empty weights class
w1.Q = np.array([[5,0],[0,1]])   # State weight matrix Q
w1.R = np.array([[1]])                               # Control input weight matrix R
w1.P = control.dare(G1.A,G1.B,w1.Q,w1.R)[0]                      # Final state matrix P, based off DARE

w2 = mpc.Weight()    # Empty weights class
w2.Q = np.array([[5,0],[0,1]])   # State weight matrix Q
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

vMax1 = 5    # Bound on velocity
xBound1 = 5  # Bound on position

vMax2 = 5    # Bound on velocity
xBound2 = 5  # Bound on position

Ac1 = np.array([[1,0],[-1,0],[0,1],[0,-1]])     # Constraint matrix A (Ax <= b)
bc1 = np.array([[xBound1],[xBound1],[vMax1],[vMax1]])                      # Constraint vector b

Ac2 = np.array([[1,0],[-1,0],[0,1],[0,-1]])     # Constraint matrix A (Ax <= b)
bc2 = np.array([[xBound2],[xBound2],[vMax2],[vMax2]])                      # Constraint vector b

AcCoupling = np.array([[-1,0,1,0]])
bcCoupling = np.array([[0]])

Ac = scipy.linalg.block_diag(Ac1,Ac2)
Ac = np.append(Ac,AcCoupling,axis=0)

bc = np.append(bc1,bc2,axis=0)
bc = np.append(bc,bcCoupling,axis=0)

#%% MPC simulation
x1 = np.zeros((2,T+1))   # States
x1[:,[0]] = x01           # Initial state
x2 = np.zeros((2,T+1))   # States
x2[:,[0]] = x02           # Initial state

x = np.append(x1,x2,axis=0)

nMu01 = Ac1.shape[0]*(N+1)
nMu02 = Ac2.shape[0]*(N+1)

mu01 = np.zeros((nMu01,1))   # Dual variables
mu02 = np.zeros((nMu02,1))   # Dual variables

#mu0_e = pheMat.encrypt_ndarray(public_key,mu0)

Dg01 = np.zeros((nMu01,n))
Dg02 = np.zeros((nMu02,n))

t1 = time.time()

for k in range(T):
    print('k:',k+1,'/',T)
    consMat1 = mpc.generate_constraint_matrices(predMat1,Ac1,bc1,x1[:,[k]],N)
    consMat2 = mpc.generate_constraint_matrices(predMat2,Ac2,bc2,x2[:,[k]],N)
    c1 = costMat1.F@(x1[:,[k]]-WP1)
    c2 = costMat2.F@(x2[:,[k]]-WP2)

#    b1_e = pheMat.encrypt_ndarray(public_key,consMat.b)
#    c1_e = pheMat.encrypt_ndarray(public_key,c)
#    mu1_e = mu0_e
    b1 = consMat1.b
    b2 = consMat2.b
    mu1 = mu01
    mu2 = mu02
    
    # On server
    for i in range(n):
        print('i:',i+1,'/',n)
#        Dg_e = -consMat.A@costMat.Qi@(consMat.A.T@mu_e + c_e) - b_e
#        mub_e = r*(mu_e + eta*Dg_e)
#        mub = np.maximum(np.zeros((24,1)),mub_e.decrypt(private_key))
#        mu_e = pheMat.encrypt_ndarray(public_key,1/r*mub)
        Dg1 = -consMat1.A@costMat1.Hi@(consMat1.A.T@mu1 + c1) - b1
        Dg2 = -consMat2.A@costMat2.Hi@(consMat2.A.T@mu2 + c2) - b2
        mub1_e = r1*(mu1 + eta*Dg1)
        mub2_e = r2*(mu2 + eta*Dg2)
        mub1 = np.maximum(np.zeros((nMu01,1)),mub1_e)
        mub2 = np.maximum(np.zeros((nMu02,1)),mub2_e)
        mu1 = 1/r1*mub1
        mu2 = 1/r2*mub2
    
    # On agent
    mu1 = mu1
    mu2 = mu2
    us1 = -costMat1.Hi@(consMat1.A.T@mu1 + c1) 
    us2 = -costMat2.Hi@(consMat2.A.T@mu2 + c2) 
    Zs1 = .5*us1.T@costMat1.H@us1 + c1.T@us1
    Zs2 = .5*us2.T@costMat2.H@us2 + c2.T@us2
    
    u1 = us1[0:1,]
    u2 = us2[0:1,]
    x1[:,[k+1]] = G1.A@x1[:,[k]] + G1.B@u1
    x2[:,[k+1]] = G2.A@x2[:,[k]] + G2.B@u2
           
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
    
    
    
    