# -*- coding: utf-8 -*-
"""
Created on Thu Oct 17 16:00:59 2019

@author: jagov
"""
import numpy as np
import scipy

def generate_prediction_matrices(LTI,N):
    nx,nu = LTI.B.shape
    predMat = PredictionMatrix()
    
    P = np.zeros(((N+1)*nx,nx))
    S = np.zeros(((N+1)*nx,nu*N))

    for i in range(N+1):
        P[i*nx:nx+i*nx,:] = np.linalg.matrix_power(LTI.A,i)
        
    for i in range(1,N+1):
        for j in range(i):
            S[i*nx:nx+i*nx,j*nu:nu+j*nu] = np.linalg.matrix_power(LTI.A,i-j-1)@LTI.B
    
    predMat.P = P
    predMat.S = S
    
    return predMat

def generate_cost_matrices(predMat,weight,N):
    costMat = CostMatrix()
    
    costMat.Q = predMat.S.T@scipy.linalg.block_diag(np.kron(np.eye(N),weight.Q),weight.P)@predMat.S + np.kron(np.eye(N),weight.R)
    costMat.Qi = np.linalg.inv(costMat.Q)
    costMat.c = predMat.S.T@scipy.linalg.block_diag(np.kron(np.eye(N),weight.Q),weight.P)@predMat.P
    
    return costMat

def generate_constraint_matrices(predMat,Ac,bc,x0,N):
    consMat = ConstraintMatrix()
    AcBar = np.kron(np.eye(N+1),Ac)
    bcBar = np.kron(np.ones((N+1,1)),bc)
    
    consMat.A = AcBar@predMat.S
    consMat.b = bcBar - AcBar@predMat.P@x0
    
    return consMat

class LTI: pass

class Weight: pass

class PredictionMatrix: pass

class CostMatrix: pass

class ConstraintMatrix: pass