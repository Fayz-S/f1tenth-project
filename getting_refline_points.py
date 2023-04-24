import numpy as np

def getLineCoords(x1,y1,x2,y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0:
        xs = np.array([x1]*10)
        ys = np.linspace(y1,y2, 10)
    elif dy == 0:
        xs = np.linspace(x1,x2, 10)
        ys = np.array([y1]*10)
    else:
        # m = (y2-y1)/(x2-x1)
        # b = y1 - m*x1
        # y = lambda x: m*x + b
        xs = np.linspace(x1,x2, 10)
        ys = np.linspace(y1,y2, 10)

    return xs, ys
        
# print ("Start of Corner:")
def printCList(l):
    print ("{", end="")
    for i in l[0:-1]:
        print (f"{i}, ")    
    print (f"{i}", end="")
    print ("}")
    
    
print ("Apex of Corner:")
x1 = 31.842456
y1 = 36.784211
x2 = 33.409512
y2 = 34.94573

xs, ys = getLineCoords(x1,y1,x2,y2)
printCList(xs)
printCList (ys)

# print ("End of Corner:")