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
q0 = np.array([0,0,0])
q1 = np.array([deg_to_rad(0),
                mm_to_meter(0),
                mm_to_meter(25)])

q2 = np.array([deg_to_rad(0),
                mm_to_meter(0),
                mm_to_meter(0)])

q3 = np.array([deg_to_rad(-90),
                mm_to_meter(0),
                mm_to_meter(0)])
q4 = np.array([deg_to_rad(-90),
                mm_to_meter(25),
                mm_to_meter(0)])
q5 = np.array([deg_to_rad(-90),
                mm_to_meter(25),
                mm_to_meter(25)])
q6 = np.array([deg_to_rad(-90),
                mm_to_meter(25),
                mm_to_meter(0)])
q7 = np.array([deg_to_rad(-270),
                mm_to_meter(25),
                mm_to_meter(0)])
q8 = np.array([deg_to_rad(-270),
                mm_to_meter(0),
                mm_to_meter(0)])
q9 = np.array([deg_to_rad(-270),
                mm_to_meter(0),
                mm_to_meter(25)])
qa = np.array([deg_to_rad(-270),
                mm_to_meter(0),
                mm_to_meter(0)])
qb = np.array([deg_to_rad(-360),
                mm_to_meter(0),
                mm_to_meter(0)])


# Trajectory commands
traj1 = rtb.jtraj(q0,q1,50)
#print (traj1)
#print (traj1.q)
traj2 = rtb.jtraj(q1,q2,10)
#print (traj2)
#print (traj2.q)
traj3 = rtb.jtraj(q2,q3,10)
#print (traj3)
#print (traj3.q)
traj4 = rtb.jtraj(q3,q4,10)
#print (traj4)
#print (traj4.q)
traj5 = rtb.jtraj(q4,q5,10)
#print (traj5)
#print (traj5.q)
traj6 = rtb.jtraj(q5,q6,10)
#print (traj6)
#print (traj6.q)
traj7 = rtb.jtraj(q6,q7,10)
#print (traj7)
#print (traj7.q)
traj8 = rtb.jtraj(q7,q8,10)
#print (traj8)
#print (traj8.q)
traj9 = rtb.jtraj(q8,q9,10)
#print (traj9)
#print (traj9.q)
traja = rtb.jtraj(q9,qa,10)
#print (traja)
#print (traja.q)
trajb = rtb.jtraj(qa,qb,10)
#print (trajb)
#print (trajb.q)

x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = -0.1
z2 = 0.1

#PLot commands

Cyl_Standard.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
#rtb.qplot(traj1.q)
#rtb.qplot(traj2.q)
Cyl_Standard.plot(traj2.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj3.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj4.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj5.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj6.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj7.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj8.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traj9.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(traja.q,limits=[x1, x2, y1, y2, z1, z2])
Cyl_Standard.plot(trajb.q,limits=[x1, x2, y1, y2, z1, z2])
#SCARA_V3.teach(jointlabels=1)
