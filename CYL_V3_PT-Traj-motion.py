import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib.pyplot as plt

# link lengths in cm
a1 = 25
a2 = 25
a3 = 25

#Link mm to meters
def mm_to_meter(a):
    m = 1000 # 1 meter = 1000m
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)


# link limits converted to meters
lm2 = 10
lm2 = mm_to_meter(lm2)

lm3 = 10
lm3 = mm_to_meter(lm3)


def deg_to_rad(T):
    return (T/180)*np.pi


#Creat links
Cyl_Standard = DHRobot([
    RevoluteDH(a1,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH((270/180)*np.pi,0,(270/180)*np.pi,a2,qlim=[0, lm2]),
    PrismaticDH(0,0,0,a3,qlim=[0, lm3]),
], name='Cylindrical')

print(Cyl_Standard)

#radians to degrees converter
def rad_to_deg(T):
    return (T/180.0)*np.pi

# q Paths
a0 = np.array([0,0,0])
a1 = np.array([deg_to_rad(-45),
                mm_to_meter(25),
                mm_to_meter(25)])



# Trajectory commands
traj1 = rtb.jtraj(a0,a1,50)
print (traj1)
print (traj1.q)

x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = -0.1
z2 = 0.1

#PLot commands

Cyl_Standard.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
rtb.qplot(traj1.q)

#SCARA_V3.teach(jointlabels=1)
