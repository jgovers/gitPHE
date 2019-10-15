# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 12:54:24 2019

@author: jagov
"""
import numpy as np

def encrypt_ndarray(key,plain_mat):
    r,c = plain_mat.shape
    enc_mat = []
    for i in range(r):
            for j in range(c):
                entry = key.encrypt(float(plain_mat[i,j]))
                enc_mat.append(entry)
    return EncryptedMatrix(r,c,enc_mat)


class EncryptedMatrix:
    __array_ufunc__ = None
    
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
            print(type(other))
            print(other)
            raise TypeError('Type not yet supported')
    
    def __radd__(self,other):
        return self.__add__(other)
    
    def __sub__(self, other):
        return self + (other * -1)

    def __rsub__(self, other):
        return other + (self * -1)
    
    def __mul__(self,other):       
        if isinstance(other,int) or isinstance(other,float):
            return self._mul_scalar(other)
        else:
            print(type(other))
            print(other)
            raise TypeError('Type not yet supported')
    
    def __rmul__(self,other):
        if isinstance(other,int) or isinstance(other,float):
            return self._mul_scalar(other)
        else:
            print(type(other))
            print(other)
            raise TypeError('Type not yet supported')
            
    def __matmul__(self,other):
        print('__matmul__')
        if isinstance(other,np.ndarray):
            r,c = other.shape
            enc_matmul = []
            if self.c != r:
                raise ValueError('Dimension mismatch')
            else:
                for i in range(self.r):
                    for j in range(c):
                        entry = 0
                        for k in range(self.c):
                            entry = entry + self.enc_mat[i*self.c+k]*float(other[k,j])
                        enc_matmul.append(entry)
                return EncryptedMatrix(self.r,c,enc_matmul)
        else:
            raise TypeError('Type not yet supported')
        
    def __rmatmul__(self,other):
        if isinstance(other,np.ndarray):
            r,c = other.shape
            enc_matmul = []
            if c != self.r:
                raise ValueError('Dimension mismatch')
            else:
                for i in range(r):
                    for j in range(self.c):
                        entry = 0
                        for k in range(c):
                            entry = entry + float(other[i,k])*self.enc_mat[k*self.c+j]
                        enc_matmul.append(entry)
                return EncryptedMatrix(r,self.c,enc_matmul)
        else:
            raise TypeError('Type not yet supported')
            
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
                    entry = self.enc_mat[i] + float(other_arr[i])
                    enc_sum.append(entry)
            return EncryptedMatrix(self.r,self.c,enc_sum)
        
    def _mul_scalar(self,scalar):
        enc_mul = []
        for i in range(self.r*self.c):
            entry = self.enc_mat[i]*scalar
            enc_mul.append(entry)
        return EncryptedMatrix(self.r,self.c,enc_mul)
    
    def decrypt(self,key):
        plain_mat = []
        
        for i in range(self.r*self.c):
            entry = key.decrypt(self.enc_mat[i])
            plain_mat.append(entry)
        
        return np.reshape(plain_mat,(self.r,self.c))

