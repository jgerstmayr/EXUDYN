
.. _testmodels-symbolicmoduletest:

*********************
symbolicModuleTest.py
*********************

You can view and download this file on Github: `symbolicModuleTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/symbolicModuleTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Tests for symbolic scalar, vector and matrix
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-12-14
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import exudyn as exu
   from exudyn.utilities import *
   
   useGraphics = False #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   esym = exu.symbolic
   import numpy as np
   import math
   
   #reset counters:
   esym.Real.__newCount = 0
   esym.Real.__deleteCount = 0
   esym.Vector.__newCount = 0
   esym.Vector.__deleteCount = 0
   esym.Matrix.__newCount = 0
   esym.Matrix.__deleteCount = 0
   
   SymReal = esym.Real 
   
   def Not(value):
       return not value
   
   def PyRealName(name, value):
       return value
   
   def PyNamedVector(name, value):
       return np.array(value)
   
   def Evaluate(value):
       #print(str(value))
       if (type(value) == esym.Real
           or type(value) == esym.Vector
           or type(value) == esym.Matrix
           ):
           return value.Evaluate()
       if type(value) == np.ndarray:
           value = value.copy() #requires copy, to store results!
       return value
   
   def Min(a,b):
       return a if a<b else b
   def Max(a,b):
       return a if a>b else b
   
   math.sign = np.sign #to have function in math
   math.Not = Not
   math.abs = np.abs
   math.mod = math.fmod
   math.min = Min
   math.max = Max
   math.round = np.round
   math.ceil = np.ceil
   math.floor = np.floor
   math.acosh=np.arccosh #numpy agrees to 16 digits with std::acosh, math.acosh not
   
   
   scalarTests = True
   vectorTests = True
   debug = False
   caseSymbolic = 0   #exudyn.symbolic
   casePython = 1     #Python
   
   listFunctions = ['abs', 'sign', 'Not',
                    'round', 'ceil', 'floor', 'exp',
                    'sqrt', 'log',
                    'sin', 'cos', 'tan', 'asin', 'acos', 'atan',
                    'sinh', 'cosh', 'tanh', 'asinh', 'acosh', 'atanh',]
   
   listBinFunctions = ['pow', 'atan2', 'mod', 'min', 'max']
   
   
   # some integers, zeros, Reals, etc.
   listNumbers = [-3,0,2,-0.13157932734543,0.,0.2345473645342,math.pi]
   # listNumbers = [-0.13157932734543,0.,math.pi]
   listResults = []
   currentResult = [] #0=symbolic, 1=math/numpy
   cntWrong = 0
   cntTests = 0
   sumResults = 0.
   if scalarTests:
       #for recording in []:
       for recording in [True,False]:
       # for recording in [True]:
           if debug: exu.Print('***********')
           if recording:
               if debug: exu.Print('WITH RECORDING')
           else:
               if debug: exu.Print('WITHOUT RECORDING')
           for number in listNumbers:
               if debug: exu.Print('NUMBER=',number)
                   
               esym.SetRecording(recording)
       
               result = [[],[]]
               for case in [0,1]:
                   if case == 0:
                       Real = SymReal
                       NamedReal = SymReal
                       lib = esym
                   else:
                       Real = float
                       NamedReal = PyRealName
                       lib = math
                       
                   a = Real(number)
                   b = NamedReal("b",math.pi)
                   c = NamedReal("c",-0.1)
                   result[case] += [Evaluate(a)]
                   result[case] += [Evaluate(b)]
                   result[case] += [Evaluate(c)]
       
                   d = a+lib.sin(a/b)**2+lib.tan(a)*lib.cos(a)*c*3
                   result[case] += [Evaluate(d)]
                   result[case] += [Evaluate(a+b)]
                   result[case] += [Evaluate(b+a)]
                   result[case] += [Evaluate(b-a)]
                   result[case] += [Evaluate(a-b)]
                   result[case] += [Evaluate(b*a)]
                   result[case] += [Evaluate(a*b)]
                   result[case] += [Evaluate(a/b)]
                   result[case] += [Evaluate(a/3)]
                   result[case] += [Evaluate(-a)]
                   result[case] += [Evaluate(+a)]
                   result[case] += [Evaluate(b**a)]
                   # result[case] += [Evaluate(1**a)] #does not WORK!
       
                   # +=,-=
                   d = Real(0)
                   result[case] += [Evaluate(d)]
                   d += a
                   result[case] += [Evaluate(d)]
                   #Real delete count wrong from here:
                   d -= a*b
                   result[case] += [Evaluate(d)]
                   d *= a
                   result[case] += [Evaluate(d)]
                   if (float(a) != 0.):
                       d /= a
                       result[case] += [Evaluate(d)]
                   
                   f = a < b
                   result[case] += [Evaluate(f)]
                   f = a <= b
                   result[case] += [Evaluate(f)]
                   f = a > b
                   result[case] += [Evaluate(f)]
                   f = a >= b
                   result[case] += [Evaluate(f)]
                   f = a == b
                   result[case] += [Evaluate(f)]
                   f = a != b
                   result[case] += [Evaluate(f)]
                   
                   #functions
                   for funcStr in listFunctions:
                       if funcStr=='log':
                           if number <= 0: continue
                       if funcStr=='sqrt':
                           if number < 0: continue
                       if funcStr=='acosh':
                           if number < 1 : continue
                       if funcStr=='atanh':
                           if abs(number) >= 1 : continue
                       if funcStr=='acos' or funcStr=='asin':
                           if abs(number) > 1 : continue
       
                       func = getattr(lib, funcStr)
                       # if debug: exu.Print(funcStr)
                       d = func(a)
                       if debug: exu.Print('  '+funcStr+'('+str(Evaluate(a))+')=',Evaluate(d))
                       result[case] += [Evaluate(d)]
                       
                   #binary functions
                   for funcStr in listBinFunctions:
                       func = getattr(lib, funcStr)
                       #if debug: exu.Print(funcStr)
                       d = func(a,2)
                       if debug: exu.Print('  '+funcStr+'('+str(Evaluate(a))+','+str(Evaluate(b))+')=',Evaluate(d))
                       result[case] += [Evaluate(d)]
                   
                   #ifthenelse
                   if case==0:
                       f = lib.IfThenElse(a, a+1, b+a)
                   else:
                       f = a+1 if a else b+a
                   result[case] += [Evaluate(f)]
                   
               
               result = np.array(result).T.tolist()
               cntTests += len(result)
   
               for res in result:
                   s = 'correct'
                   sumResults += res[0]
                   if res[0] != res[1]: 
                       s = 'WRONG:'
                       s += str(res[0]-res[1])
                       cntWrong+=1
                   if debug: exu.Print('. res sym:',res[0], ', res Py:', res[1], s)
               
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Vector-Matrix
   SymVector = esym.Vector
   SymMatrix = esym.Matrix
   
   if vectorTests:
       for recording in [False,True]:
       # for recording in [True]:
           if debug: exu.Print('***********')
           if recording:
               if debug: exu.Print('WITH RECORDING')
           else:
               if debug: exu.Print('WITHOUT RECORDING')
                   
           esym.SetRecording(recording)
       
           result = [[],[]]
           for case in [1,0]:
               if debug: exu.Print('*********\ncase:',case)
               if case == 0:
                   Real = SymReal
                   NamedReal = SymReal
                   lib = esym
                   Vector = esym.Vector
                   NamedVector = esym.Vector
                   Matrix = esym.Matrix
               else:
                   Real = float
                   NamedReal = PyRealName
                   lib = math
                   Vector = np.array
                   NamedVector = PyNamedVector
                   Matrix = np.array
       
               a = NamedReal("a",0.5)
               b = NamedReal("b",math.pi)
               
               v = Vector([1.3])
               w = Vector([4.2])
               result[case] += [Evaluate(v+w)]
       
               v = NamedVector('vv',[1,2])
               w = Vector([4.2-1,-1.2-1])
               x = Vector([0.,0.])
                   
               if case == 0:
                   w.SetVector([4.2,-1.2])
                   d = 3.3*v+a*w+(v*w)*v
                   x[0] += d.NormL2()
                   x[1] += Evaluate(v == v )
                   x[1] += Evaluate(v == Vector([1,2.1]) )
                   x[1] += Evaluate(v != Vector([1,2.1]) )
                   x[1] += Evaluate(v == Vector([1,2.]) )
                   x[1] += Evaluate(v != Vector([1,2.]) )
               else:
                   w = np.array([4.2,-1.2])
                   d = 3.3*v+a*w+(v@w)*v
                   x[0] += np.linalg.norm(d)
                   x[1] += (v == v ).all()
                   x[1] += (v == Vector([1,2.1]) ).all()
                   x[1] += (v != Vector([1,2.1]) ).any()
                   x[1] += (v == Vector([1,2.]) ).all()
                   x[1] += (v != Vector([1,2.]) ).any()
               
               result[case] += [Evaluate(x)]
   
               result[case] += [Evaluate(d)]
               result[case] += [Evaluate(v+w)]
               result[case] += [Evaluate(v-w)]
               if case==0: 
                   n = w.NumberOfItems()
                   result[case] += [Evaluate(v*w)]
                   # print('v0:',x)
               if case==1: 
                   n = len(w)
                   result[case] += [Evaluate(v@w)]
                   # print('v1:',x)
               x = Vector([n,1.1])
               
               result[case] += [Evaluate(x)]
               result[case] += [Evaluate(v)]
               result[case] += [Evaluate(-v)]
               result[case] += [Evaluate(a*v)]
               result[case] += [Evaluate(v*a)]
               result[case] += [Evaluate(3.3*v)]
               result[case] += [Evaluate(v*3.3)]
   
               v = Vector([-0.33,0.347,1.5])
               w = Vector([4.2,-1.2+10,7.7])
               w[1] = -1.2
               result[case] += [Evaluate(w)]
       
               result[case] += [Evaluate(d)]
               result[case] += [Evaluate(v+w)]
               result[case] += [Evaluate(v-w)]
   
               if case==0: 
                   result[case] += [Evaluate(v*w)]
                   result[case] += [Evaluate(v.MultComponents(w))]
                   
               if case==1: 
                   result[case] += [Evaluate(v@w)]
                   result[case] += [Evaluate(v*w)]
               
               result[case] += [Evaluate(v)]
               result[case] += [Evaluate(-v)]
               result[case] += [Evaluate(a*v)]
               result[case] += [Evaluate(v*a)]
               result[case] += [Evaluate(3.3*v)]
               result[case] += [Evaluate(v*3.3)]
   
               u = Vector([0.,0.,0.])
               u += 2*v
               # print('case'+str(case)+': 2*v=',Evaluate(2*v),', v=',v,', u=',u)
               u = u + 2*v
               result[case] += [Evaluate(u)]
               u -= v
               result[case] += [Evaluate(u)]
               result[case] += [Evaluate(u==v)]
               result[case] += [Evaluate(u!=v)]
               u *= a
               result[case] += [Evaluate(u)]
               u *= 1/3
               result[case] += [Evaluate(u)]
               result[case] += [Evaluate(u==v)]
               result[case] += [Evaluate(u!=v)]
   
               #MATRIX MATRIX MATRIX
               M = Matrix(np.array([[2.,0.1,0.33],
                                    [-0.1,2.3,0.7],
                                    [0,0.34,1.8]]))
               N = Matrix(np.array([[1.,0.3,-0.33],
                                    [-0.9,1.3,-0.7],
                                    [0,0.64,-1.8]]))
               result[case] += [Evaluate(N)]
   
               if case==0: 
                   M02 = M[0,2].Evaluate()
                   M20 = M[2,0].Evaluate()
                   
                   if (M02 != M.Get(0,2).Evaluate()):
                       M02 = -1000
                       #print('*****************ERROR')
                   nr = M.NumberOfRows()
                   nc = M.NumberOfColumns()
                   
                   N.SetMatrix(np.array([[(nr+nc)/6,0.3,-M02],
                                        [-0.9,1.3,-0.7],
                                        [M20,0.64+100,-1.8]]))
                   N[2,1] = 0.64
   
               result[case] += [Evaluate(N)]
               
               result[case] += [Evaluate(M)]
               result[case] += [Evaluate(a*M)]
               result[case] += [Evaluate(0.5*M)]
               result[case] += [Evaluate(M*a)]
               result[case] += [Evaluate(M*0.5)]
               result[case] += [Evaluate(M+N)]
               result[case] += [Evaluate(M-N)]
               if case==0: 
                   result[case] += [Evaluate(M*N)]
                   result[case] += [Evaluate(M*v)]
                   result[case] += [Evaluate(v*M)]
               if case==1: 
                   result[case] += [Evaluate(M@N)]
                   result[case] += [Evaluate(M@v)]
                   result[case] += [Evaluate(v@M)]
                   
               P = 0.*M
               result[case] += [Evaluate(P)]
               
               #matrix delete counts wrong starting here:
               P += N
               result[case] += [Evaluate(P)]
               P -= M
               result[case] += [Evaluate(P)]
               P *= 1.3
               result[case] += [Evaluate(P)]
               
               
           # result = np.array(result).T.tolist() #doesnt work for mixed components
           cntTests += len(result[0])
           for i in range(len(result[0])):
               res = [result[0][i],result[1][i]]
               s = 'correct'
               sumResults += np.linalg.norm(res[0])
               #if (res[0] != res[1]).any():  #problem with 1e-16 errors
               wrong = False
               if np.linalg.norm(res[0] - res[1]) > 1e-15: 
                   s = '\\diff:\n'
                   s += str(res[0]-res[1])
                   cntWrong+=1
                   wrong=True
               if debug or wrong: 
                   exu.Print('. res sym:\n',res[0], ',\n  res Py:\n', res[1], s)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #cleanup and check new/delete
   if True:
       del a,b,c,d,f
       del u,v,w,x
       del M,N,P
   
       #check sum of new/delete
       newDelSum = 0
       newDelSum += esym.Real.__newCount-esym.Real.__deleteCount
       newDelSum += esym.Vector.__newCount-esym.Vector.__deleteCount
       newDelSum += esym.Matrix.__newCount-esym.Matrix.__deleteCount
       if newDelSum != 0:
           exu.Print('*******************')
           exu.Print('*  WARNING        *')
           exu.Print('New-Del=',newDelSum)
           exu.Print('*******************')
       sumResults += newDelSum
   
   exu.Print('Real.newCount=',esym.Real.__newCount)
   exu.Print('Real.deleteCount=',esym.Real.__deleteCount)
   
   exu.Print('Vector.newCount=',esym.Vector.__newCount)
   exu.Print('Vector.deleteCount=',esym.Vector.__deleteCount)
   
   exu.Print('Matrix.newCount=',esym.Matrix.__newCount)
   exu.Print('Matrix.deleteCount=',esym.Matrix.__deleteCount)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exu.Print('\nfinished',cntTests,'tests')
   exu.Print('WRONG results (after vectorTests):',cntWrong,'\n') 
   
   #
   u = sumResults/1000
   exu.Print('u=',u)
   exu.Print('solution of symbolicModuleTest=',u)
   
   # result for 10000 steps; identical for both UF cases
   exudynTestGlobals.testError = u - (0.9480053738744615) 
   exudynTestGlobals.testResult = u
   


