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


#**function: create a diagonal or identity matrix; used for interface.py, avoiding the need for numpy
#**input: 
#       rowsColumns: provides the number of rows and columns
#       value: initialization value for diagonal terms
#**output: list of lists representing a matrix
def DiagonalMatrix(rowsColumns, value=1):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m

eye2D = DiagonalMatrix(rowsColumns=2,value=1.) #2x2 identity matrix
eye3D = DiagonalMatrix(rowsColumns=3,value=1.) #3x3 identity matrix
#eye4D = DiagonalMatrix(rowsColumns=4,value=1.) #4x4 identity matrix

#**function: compute L2 norm for vectors without switching to numpy or math module
#**input: vector as list or in numpy format
#**output: L2-norm of vector
def NormL2(vector):
    value = 0
    for x in vector:
        value += x**2
    return value**0.5

#**function: compute sum of all values of vector
#**input: vector as list or in numpy format
#**output: sum of all components of vector
def VSum(vector):
    value = 0
    for x in vector:
        value += x
    return value

#**function: add two vectors instead using numpy
#**input: vectors v0 and v1 as list or in numpy format
#**output: component-wise sum of v0 and v1
def VAdd(v0, v1):
    if len(v0) != len(v1): print("ERROR in VAdd: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]+v1[i]
    return v

#**function: subtract two vectors instead using numpy: result = v0-v1
#**input: vectors v0 and v1 as list or in numpy format
#**output: component-wise difference of v0 and v1
def VSub(v0, v1):
    if len(v0) != len(v1): print("ERROR in VSub: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]-v1[i]
    return v

#**function: scalar multiplication of two vectors instead using numpy: result = v0'*v1
#**input: vectors v0 and v1 as list or in numpy format
#**output: sum of all component wise products: c0[0]*v1[0] + v0[1]*v1[0] + ...
def VMult(v0, v1):
    if len(v0) != len(v1): print("ERROR in VMult: incompatible vectors!")
    r = 0
    for i in range(len(v0)):
        r += v0[i]*v1[i]
    return r

#**function: multiplication vectors with scalar: result = s*v
#**input: value {\it scalar} and vector {\it v} as list or in numpy format
#**output: scalar multiplication of all components of v: [scalar*v[0], scalar*v[1], ...]
def ScalarMult(scalar, v):
    res=[0]*len(v)
    for i in range(len(v)):
        res[i] += scalar*v[i]
    return res

#**function: take a 3D vector and return a normalized 3D vector (L2Norm=1)
#**input: vector v as list or in numpy format
#**output: vector v multiplied with scalar such that L2-norm of vector is 1
def Normalize(v):
    #v=copy.deepcopy(vector) #copy, such that vector is not changed
    v2=[0]*len(v)

    fact = NormL2(v)
    fact = 1./fact
    for i in range(len(v2)): 
        v2[i]=fact*v[i]
    return v2
    
#**function: apply tilde operator (skew) to 3D-vector and return skew matrix
#**input: 3D vector v as list or in numpy format
#**output: matrix as list of lists containing the skew-symmetric matrix computed from v: $\mr{0}{-v[2]}{v[1]} {v[2]}{0}{-v[0]} {-v[1]}{v[0]}{0}$
def Vec2Tilde(v):
    print('Vec2Tilde is deprecated; use Skew(...)')
    return [[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]]

#**function: take skew symmetric matrix and return vector (inverse of Skew(...))
#**input: list of lists containing a skew-symmetric matrix (3x3)
#**output: list containing the vector v (inverse function of Vec2Tilde(...))
def Tilde2Vec(m):
    print('Tilde2Vec is deprecated; use Skew(...)')
    return [-m[1][2], m[0][2], -m[0][1]]


gaussIntegrationPoints=[[0],
                        [-(1. / 3.)**0.5, (1. / 3.)**0.5],
                        [-(3. / 5.)**0.5, 0., (3. / 5.)**0.5],
                        [-(3. / 7. + (120.)**0.5 / 35.)**0.5, -(3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. + (120.)**0.5 / 35.)**0.5],
                        [-0.906179845938664, -0.5384693101056831, 0., 0.5384693101056831, 0.906179845938664],
                        ]

gaussIntegrationWeights=[[2],
                         [1., 1.],
                         [5. / 9., 8. / 9., 5. / 9.],
                         [1. / 2. - 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. - 5. / (3.*(120.)**0.5)],
                         [0.23692688505618914, 0.47862867049936636, 0.5688888888888889, 0.47862867049936636, 0.23692688505618914],
                         ]

#numerically integrate a scalar function with integration Order in interval [a,b]
def GaussIntegrate(functionOfX, integrationOrder, a, b):
    cnt = 0
    value = 0*functionOfX(0) #initialize value with correct shape
    if integrationOrder > 9:
        raise ValueError("GaussIntegrate: maximum implemented integration order is 9!")
    if integrationOrder%2 != 1 or integrationOrder < 1:
        raise ValueError("GaussIntegrate: integration order must be odd (1,3,5,...) and > 0")
    
    points = gaussIntegrationPoints[int(integrationOrder/2)]
    weights = gaussIntegrationWeights[int(integrationOrder/2)]
    
    for p in points:
        x = 0.5*(b - a)*p + 0.5*(b + a)
        value += 0.5*(b - a)*weights[cnt]*functionOfX(x);
        cnt += 1

    return value







