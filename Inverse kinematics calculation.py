from sympy import symbols, Eq, solve, sin, cos, tan
import math

# ----------------------- Inverse kinematics calculation -----------------------

# Define the variables
x, y, z = 0.2, 30, 14
a2, a3, a6 = 1.3, 12.021, 13
d1, d2, d5 = 6.1, 7.001, 12.171
beta1, beta2, beta3 = symbols('beta1 beta2 beta3')

# Define the equations
eq1 = Eq(12.021*cos(beta2) + 12.171*cos(beta3), (x**2+y**2)**0.5 - (a2 + a6))
eq2 = Eq(12.021*sin(beta2) - 12.171*sin(beta3), z - (d1 + d2))
eq3 = Eq(tan(beta1), y/x)

# Solve the system of equations
solution = solve((eq1, eq2, eq3),(beta1, beta2, beta3))
solution = [(round(math.degrees(i[0]),4), round(math.degrees(i[1]),4), round(math.degrees(i[2]),4)) for i in solution]
filtered_solution = [(beta1, beta2, beta3) for (beta1, beta2, beta3) in solution if 0 <= beta1 <= 165 and 0 <= beta2 <= 120 and -80 <= beta3 <= 85]

print("Solution:", solution)
print("Filtered Solution:", filtered_solution)
