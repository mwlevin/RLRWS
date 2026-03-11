import matplotlib.pyplot as plt

i=0

x=[]
y=[]
while i<1000000:
    plt.clf()
    x.append(i)
    y.append(i**2)
    plt.plot(x,y)
    i+=1
    plt.pause(0.01)
    plt.ioff()