import matplotlib.pyplot as plt
w = -0.75
warning = -0.1
a = 0.4
d = 0.3
c_val = 0.7

c_val_inv = 1/c_val
c_val_inv = round(round(c_val_inv / 0.2) * 0.2, 1)


print(" solution is ", ((a*w - warning) / d)**(c_val_inv)) 