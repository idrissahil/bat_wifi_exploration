import time


start_new=time.time()
newlist = []
newlist2 = []

append_new = newlist.append
for i in range(1000000):
    b=200*i
    append_new(b)

end_new=time.time()

print("new append", end_new - start_new)

start_old=time.time()
newlist = []
for i in range(1000000):
    b=200*i
    newlist.append(b)
end_old=time.time()

print("old append", end_old - start_old)

start_new2=time.time()
newlist = [0] * 1000000
for i in range(1000000):
    b=200*i
    newlist[i] = b

end_new2=time.time()

print("pre allocate", end_new2 - start_new2)

start_new3=time.time()
for i in range(1000000):
    b=200*i
    newlist[i] = b

end_new3=time.time()

print("no allocate", end_new3 - start_new3)

start_new4=time.time()
i=0
while i<1000000:
    b=200*i
    newlist[i] = b
    newlist2.append(i)

    i=i+1
print("while loop", newlist[10])
end_new4=time.time()

print("while loop", end_new3 - start_new3)





items = [0] * 1000000
start_new5=time.time()
map_test = list(map(lambda x: x*200, newlist2))
end_new5=time.time()
print("map_test", map_test[10])



print(" map time ", end_new5 - start_new5)

print(round (5.7))


def round_of_rating(number):
    """Round a number to the closest half integer.
    >>> round_of_rating(1.3)
    1.5
    >>> round_of_rating(2.6)
    2.5
    >>> round_of_rating(3.0)
    3.0
    >>> round_of_rating(4.1)
    4.0"""

    return round(number * 2) / 2
print(round_of_rating(5.7))

list_0=[]
list_1=[2,2,5]
list_2=[3,3,4]
#list_0.extend(list_1)
#list_0.append(list_2)
print(list_0)

#print(list_0[0][1])

for i in range(10):
    if i==4:
        break
    print(i)