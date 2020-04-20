# Utility functions and structures for Exudyn
"""
Created on Fri Jul 26 10:53:30 2019

@author: Johannes Gerstmayr

goal: support functions, which simplify the generation of models
"""

#pi = np.pi
#sqrt2 = np.sqrt(2.)
pi = 3.1415926535897932 #define pi in order to avoid importing large libraries
sqrt2 = 2.**0.5
g = 9.81 #gravity constant


#item interface diagonal matrix creator
def DiagonalMatrix(rowsColumns, value):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m

eye2D = DiagonalMatrix(rowsColumns=2,value=1.) #2x2 identity matrix
eye3D = DiagonalMatrix(rowsColumns=3,value=1.) #3x3 identity matrix
#eye4D = DiagonalMatrix(rowsColumns=4,value=1.) #4x4 identity matrix

#compute L2 norm for vectors without switching to numpy or math module
def NormL2(vector):
    value = 0
    for x in vector:
        value += x**2
    return value**0.5

#add two vectors instead using numpy
def VAdd(v0, v1):
    if len(v0) != len(v1): print("ERROR in VAdd: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]+v1[i]
    return v

#subtract two vectors instead using numpy: result = v0-v1
def VSub(v0, v1):
    if len(v0) != len(v1): print("ERROR in VSub: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]-v1[i]
    return v

#scalar multiplication of two vectors instead using numpy: result = v0'*v1
def VMult(v0, v1):
    if len(v0) != len(v1): print("ERROR in VMult: incompatible vectors!")
    r = 0
    for i in range(len(v0)):
        r += v0[i]*v1[i]
    return r

#multiplication vectors with scalar: result = s*v
def ScalarMult(scalar, v):
    res=[0]*len(v)
    for i in range(len(v)):
        res[i] += scalar*v[i]
    return res

#normalize a 3D vector (set length to 1)
def Normalize(vector):
    #v=copy.deepcopy(vector) #copy, such that vector is not changed
    v=[0]*len(vector)

    fact = NormL2(vector)
    fact = 1./fact
    for i in range(len(v)): 
        v[i]=fact*vector[i]
    return v
    
#apply tilde operator (skew) to R3-vector
def Vec2Tilde(v):
    print('Vec2Tilde is deprecated; use Skew(...)')
    return [[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]]

#convert skew symmetric matrix to vector
def Tilde2Vec(m):
    print('Tilde2Vec is deprecated; use Skew(...)')
    return [-m[1][2], m[0][2], -m[0][1]]












