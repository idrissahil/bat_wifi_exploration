import math
rho=1
phi=3*math.pi/2

x = math.sqrt(rho) * math.cos(phi)
y = math.sqrt(rho) * math.sin(phi)


width=2
height=2

x = x * width/2
y = y * height/2

print(x)
print(y)


for i in range (10):
    print(i)