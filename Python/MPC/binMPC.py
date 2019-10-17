# -*- coding: utf-8 -*-
"""
Created on Thu Oct 17 16:00:59 2019

@author: jagov
"""
import numpy as np

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
    
    
    return costMat

class LTI: pass

class Weight: pass

class PredictionMatrix: pass

class CostMatrix: pass