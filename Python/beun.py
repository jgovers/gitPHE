# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 12:54:24 2019

@author: jagov
"""
from phe import paillier as phe
import numpy as np

public_key, private_key = phe.generate_paillier_keypair()

def encrypt_ndarray(key,plain_mat):
    r,c = plain_mat.shape
    enc_mat = []
    for i in range(r):
            for j in range(c):
                entry = key.encrypt(float(plain_mat[i,j]))
                enc_mat.append(entry)
    return EncryptedMatrix(r,c,enc_mat)

class EncryptedMatrix:
    
    def __init__(self,r,c,enc_mat):
        self.r = r
        self.c = c
        self.enc_mat = enc_mat
    
    def __add__(self,other):
        if isinstance(other,EncryptedMatrix):
            return self._add_EncryptedMatrix(other)
        if isinstance(other,np.ndarray):
            return self._add_ndarray(other)
        else:
            raise TypeError('Type not yet supported')
    
    def __radd__(self,other):
        return self.__add__(other)
    
    def _add_EncryptedMatrix(self,other):
        if self.r != other.r or self.c != other.c:
            raise ValueError('Dimension mismatch')
        else:
            enc_sum = []
            for i in range(self.r*self.c):
                    entry = self.enc_mat[i] + other.enc_mat[i]
                    enc_sum.append(entry)
            return EncryptedMatrix(self.r,self.c,enc_sum)

    def _add_ndarray(self,other):
        r,c = other.shape
        if self.r != r or self.c != c:
            raise ValueError('Dimension mismatch')
        else:
            other_arr = other.reshape(r*c)
            enc_sum = []
            for i in range(self.r*self.c):
                    entry = self.enc_mat[i] + other_arr[i]
                    enc_sum.append(entry)
            return EncryptedMatrix(self.r,self.c,enc_sum)
    
    def decrypt(self,key):
        plain_mat = []
        
        for i in range(self.r*self.c):
            entry = key.decrypt(self.enc_mat[i])
            plain_mat.append(entry)
        
        return np.reshape(plain_mat,(self.r,self.c))

    
A = np.array([[1,0],[-1,0],[0,1],[0,-1]])
b = np.array([[2],[-1],[1.5],[-.5]])    

A_e = encrypt_ndarray(public_key,A)
b_e = encrypt_ndarray(public_key,b)