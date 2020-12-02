#using sympy to do symbolic computation and to integrate an expression

from sympy import *
init_printing() #do pretty printing; turn off with (False)

x, a, L, b, v = symbols('x a L b v')

expr=expr=v*((1-b)*cos(a*x)+b)
fx=expr.subs(a,2).subs(b,0.5).subs(v,10)
plotting.plot(fx,(x,0,pi.evalf()))

intX=integrate(expr,x)
fintX=intX.subs(a,2).subs(b,0.5).subs(v,10)
plotting.plot(fintX,(x,0,pi.evalf()))

#subsitute multiple variables
res_exp = expr.subs([(a, 2), (b, 4), (v, 1)])



#+++++++++++++++++++++++++++++++++++++++++++++++++++
#solve problem of moving point (0,L) with init/final velocity v:
from sympy import *
x, L, b, v = symbols('x L b v')

expr=v*((1-b)*cos(2*pi*x/L)+b)
intX=integrate(expr,x)
#==> b=1/v
expr2=v*((1-1/v)*cos(2*pi*x/L)+1/v)
intX2=integrate(expr2,x)
#==> intX2=v*(L*sin(2*pi*x/L)/(2*pi) - L*sin(2*pi*x/L)/(2*pi*v) + x/v)
intX2.subs(x,0) #0
intX2.subs(x,L) #L
expr2.subs(x,0) #v
expr2.subs(x,L) #v

#+++++++++++++++++++++++++++++++++++++++++++++++++++
#user functions with symbolic differentiation:
from sympy import *
t, L = symbols('t L')
f=L*sin(t*4)
#str(diff(f, t)) ==> write into file?

diffF = diff(f,t)

from math import sin, cos, pi
def UFoffset(time, lOffset): 
	return f.subs([(t, time), (L, lOffset)]

def UFoffset_t(t, lOffset): #time derivative of UFoffset
	return diffF.subs([(t, time), (L, lOffset)]


