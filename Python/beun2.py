# -*- coding: utf-8 -*-
"""
Created on Fri Oct 11 12:05:29 2019

@author: jagov
"""

class Fruit:
     def __init__(self):
         self.Fruits = {"Apple": 0, "Pear": 1, "Banana": 2}
     def __getitem__(self, key):
         return key