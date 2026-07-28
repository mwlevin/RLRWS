import socket
import sys
import pickle as pkl
from matplotlib import animation
import datetime
import numpy as np
from matplotlib import pyplot as plt
import matplotlib

plt.ion()  # Turn on interactive mode

# Create figure and axis
fig = plt.figure(figsize=(6, 6))
ax = plt.subplot(111)
plt.axis('off')
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.tight_layout()

HOST_REC = ""
PORT_REC = 10890

s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))
s_rec.setblocking(False)




current_time = datetime.datetime.now()
last_time = current_time
yellow_limit = 30  # at warning = 30, we switch from yellow to red

# pos_ego = 1500
# spd_ego = 20
varr = 2



while True:
    current_time = datetime.datetime.now()
    
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData = None

        
        keepReceiving = True
        
        while keepReceiving:
            try:
                data, fromAddr = s_rec.recvfrom(10048)

                
                if data:
                    newestData = data

                    
            except socket.error as why:
                keepReceiving = False
                break
        
        # Check if we have ALL required data
        if newestData:
            # Clear the axis
            ax.clear()
            ax.axis('off')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            
            # Unpack data
            warning_showed = pkl.loads(newestData)
            warning_showed = float(warning_showed)  # Ensure it's a float for plotting

            
            print('warning_showed', warning_showed)

            # Determine color and size
            
            if warning_showed <=0.001:   # green color, normal driving
                plt.plot(0,0,'green',alpha=0.5,marker='o', markersize= 40 * varr )[0]

            elif 0.001<warning_showed <=yellow_limit:  # yellow color, mild braking
                if warning_showed  <= yellow_limit/3:   # warning smaller than 10
                    plt.plot(0,0,'gold',alpha=0.5,marker='o', markersize= (warning_showed *5+40) * varr )[0]
                else:   # warning larger than 10 but still yellow
                    plt.plot(0,0,'gold',alpha=0.5,marker='o', markersize=((warning_showed  - yellow_limit/3)*3 + (5 * yellow_limit/3) + 40) * varr )[0]
            else:   # warning larget than limit, red color, harsh braking
                plt.plot(0,0,'red',alpha=0.5,marker='o', markersize=((min( (warning_showed - yellow_limit )*3  +  (5*yellow_limit/3) + 40 , 350))* varr ))[0]
        # Update the display
            plt.draw()
            plt.pause(0.01)
        else:
            print("Waiting for complete data...")
            plt.pause(0.01)