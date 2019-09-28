from timeit import default_timer as timer
from scipy.optimize import fsolve
from math import *

def equations(p):
    j1, j2, j3 = p
    x, y, z = (90, 100, 150)

    f1 = -x + 5*sin(j1)*(20*cos(j2 + j3) - 3*sin(j2 + j3) + 12*cos(j2))
    f2 = -y + 156 - 5*pow(409,0.5)*cos(j2 + j3 - atan(20/3)) - 60*sin(j2)
    f3 = -z + 60*cos(j1)*cos(j2) + 100*cos(j1)*cos(j2)*cos(j3) - 15*cos(j1)*cos(j2)*sin(j3) - 15*cos(j1)*cos(j3)*sin(j2) - 100*cos(j1)*sin(j2)*sin(j3) + 41

    return (f1, f2, f3)

start = timer()
j1, j2, j3 = fsolve(equations, (0,0,0))
end = timer()

print(j1, j2, j3)
print(end - start)
