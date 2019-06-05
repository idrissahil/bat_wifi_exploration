#!/usr/bin/env python

import math
from sympy import apart
from sympy.abc import x, y, z
from sympy import solve, symbols, solveset, S
x, y, z = symbols('x,y,z', real=True)
curr_location_x1=2.3321850299835214
curr_location_y1=4.577636718749999
curr_location_z1=1.4980845451354983

curr_location_x2=1.0010116100311284
curr_location_y2=3.3068938255310054
curr_location_z2=1.2934914827346804

curr_location_x3=3.0047345161437993
curr_location_y3=1.7993526458740228
curr_location_z3=1.0497559309005742

f_1=0.6856286193993439
f_2=0.9024800948483291
f_3=0.8693180767942476

x_my=3
y_my=3
z_my=3

x_test= -2.10558177595880
y_test= -1.99215177435568
z_test= -1.90247140211703

test_function = math.log(f_2) + ((curr_location_x2-x_my)**2)/(2*(x_test**2)) + ((curr_location_y2-y_my)**2)/(2*(y_test**2)) + ((curr_location_z2-z_my)**2)/(2*(z_test**2))

print("test function", test_function)
sol4=solve([math.log(f_3) + ((curr_location_x3-x_my)**2)/(2*(x**2)) + ((curr_location_y3-y_my)**2)/(2*(y**2)) + ((curr_location_z3-z_my)**2)/(2*(z**2)),\
            math.log(f_2) + ((curr_location_x2-x_my)**2)/(2*(x**2)) + ((curr_location_y2-y_my)**2)/(2*(y**2)) + ((curr_location_z2-z_my)**2)/(2*(z**2)),\
            math.log(f_1) + ((curr_location_x1-x_my)**2)/(2*(x**2)) + ((curr_location_y1-y_my)**2)/(2*(y**2)) + ((curr_location_z1-z_my)**2)/(2*(z**2)) ], [x, y, z])
#first_set=sol4[0]

print(sol4)

