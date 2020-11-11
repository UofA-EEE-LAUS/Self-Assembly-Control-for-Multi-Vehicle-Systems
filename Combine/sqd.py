# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:36:01 2020

@author: 1
"""

import sys

def squared(x):
    y = x * x
    return y

if __name__ == '__main__':
    x = float(sys.argv[1])
    sys.stdout.write(str(squared(x)))