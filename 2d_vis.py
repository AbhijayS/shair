import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pandas as pd
import math

fig,axs = plt.subplots(4,2)

#definitions
DPI = 1050
RADIUS = 4.5/2.54 # 4.5 cm to in

x = []
y = []

pdf = None

def animate(i):
    global pdf
    df = pd.read_csv('clipper.csv')
    rows = len(df['timestamp'])

    if not df.equals(pdf):
        pdf = df

        axs[0][0].clear()
        axs[0][0].set_title('2D Pose')
        x = []
        y = []
        for i in range(rows):
            e = eval(df['pose2d'][i])
            x.append(e[0])
            y.append(e[1])
        axs[0][0].plot(x, y, '.-b')

        # axs[0][0].spines['left'].set_position('center')
        # axs[0][0].spines['right'].set_color('none')
        # axs[0][0].spines['bottom'].set_position('center')
        # axs[0][0].spines['top'].set_color('none')
        # axs[0][0].xaxis.set_ticks_position('bottom')
        # axs[0][0].yaxis.set_ticks_position('left')
        axs[0][0].xaxis.set_ticks(np.arange(0, 9, 1))
        axs[0][0].yaxis.set_ticks(np.arange(0, 12, 1))
        axs[0][0].grid(True)

        # axs[1][0].clear()
        # axs[1][0].set_title('Pose X vs. Time')
        # axs[1][0].plot(df['timestamp'], df['pose_x'])

        # axs[2][0].clear()
        # axs[2][0].set_title('Optical X vs. Time')
        # x = []
        # sx = 0
        # for i in df['dx']:
        #     sx = sx + i
        #     x.append(sx)
        # # axs[2][0].plot(df['timestamp'], x)
        # axs[2][0].plot(np.arange(len(df['dx'])), x)
        # axs[2][0].grid(True)

        # axs[1][1].clear()
        # axs[1][1].set_title('Pose Y vs. Time')
        # axs[1][1].plot(df['timestamp'], df['pose_y'])

        # axs[2][1].clear()
        # axs[2][1].set_title('Optical Y vs. Time')
        # y = []
        # sy = 0
        # for i in df['dy']:
        #     sy = sy + i
        #     y.append(sy)
        # axs[2][1].plot(df['timestamp'], y)
        axs[2][1].clear()
        axs[2][1].set_title('Loop times')
        axs[2][1].plot(df['timestamp'], df['loop'])


        axs[3][0].clear()
        axs[3][0].set_title('Yaw vs. Time')
        yaws = []
        for p in df['pose2d']:
            yaws.append(eval(p)[2])
        axs[3][0].plot(df['timestamp'], yaws)


        # df = pd.read_csv('mouse.data', names=['dx','dy'])
        # x = []
        # y = []
        # sx = 0
        # for i in df['dx'].to_numpy():
        #     sx = sx + i
        #     angle = sx / DPI / RADIUS
        #     x.append(math.cos(angle) * RADIUS)
        #     y.append(math.sin(angle) * RADIUS)
        

        # axs[0][1].clear()
        # axs[0][1].set_title('Optical X vs. Y')
        # axs[0][1].plot(x, y, '.-b')
        # axs[0][1].spines['left'].set_position('center')
        # axs[0][1].spines['right'].set_color('none')
        # axs[0][1].spines['bottom'].set_position('center')
        # axs[0][1].spines['top'].set_color('none')
        # axs[0][1].xaxis.set_ticks_position('bottom')
        # axs[0][1].yaxis.set_ticks_position('left')
        # axs[0][1].xaxis.set_ticks(np.arange(-4, 5, 1))
        # axs[0][1].yaxis.set_ticks(np.arange(-2, 3, 1))
        # axs[0][1].grid(True)


        # axs[3][1].clear()
        # x = []
        # sx = 0
        # for i in df['dx']:
        #     sx = sx + i
        #     x.append((sx/1050))
        # axs[3][1].plot(np.arange(len(df['dx'])), x)
        # axs[3][1].grid(True)




ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()