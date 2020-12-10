import sympy
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix
import numpy as np


sympy.init_printing(use_latex="mathjax", fontsize='16pt')
a, x, y, v, w, theta, time = symbols('a, x, y, v, w, theta, t')
d = v*time
beta = (d/w)*sympy.tan(alpha)
r = w/sympy.tan(alpha)

fxu = Matrix([[x-r*sympy.sin(theta) + r*sympy.sin(theta+beta)],
              [y+r*sympy.cos(theta)- r*sympy.cos(theta+beta)],
              [theta+beta]])
F = fxu.jacobian(Matrix([x, y, theta]))
print(F)
print("\n\n")


subs = {x: 1, y: 1, theta:0}


resultado = np.array(F.evalf(subs=subs)) #.astype(float)
print(resultado)


symbols('')



from sympy import *
x, y, z = symbols("x y z")
expr = cos(x) + 1
expr.subs(x, y)


print(cos(y) + 1)


print(expr.evalf(subs={x: 0}))
