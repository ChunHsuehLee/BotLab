import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("lcmtypes")
import lcm
from lcmtypes import odometry_t

if len(sys.argv) < 2:
    #sys.stderr.write("usage: plot_slam_true.py <logfile>")
    log = lcm.EventLog('../data/obstacle_slam_error2.log',"r")
else:
    log = lcm.EventLog(sys.argv[1],"r")

#print(str(sys.argv[1]))


trueData = np.empty((0,4), dtype=float)
slamData = np.empty((0,4), dtype=float)
odoData = np.empty((0,4), dtype=float)
initTrue = 0
initSlam = 0
initOdo = 0

for event in log:
    if event.channel == "TRUE_POSE":
        msg = odometry_t.decode(event.data)
        if initTrue==0:
            start_true_utime = msg.utime
            initTrue = 1
        trueData = np.append(trueData, np.array([[ \
            (msg.utime-start_true_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)
    if event.channel == "ODOMETRY":
        msg = odometry_t.decode(event.data)
        if initOdo==0:
            start_odo_utime = msg.utime
            initOdo = 1
        odoData = np.append(odoData, np.array([[ \
            (msg.utime-start_odo_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)
    if event.channel == "SLAM_POSE":
        if initSlam==0:
            start_slam_utime = msg.utime
            initSlam = 1
        msg = odometry_t.decode(event.data)
        slamData = np.append(slamData, np.array([[ \
            (msg.utime-start_slam_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

errorData = np.empty((0,4), dtype=float)
j = 0
for i in range(np.shape(odoData)[0]-1):
    if(j < np.shape(slamData)[0] - 1):
        if trueData[i,0] > slamData[j,0]:
            j+=1
        errorData = np.append(errorData, np.array([[ \
                trueData[i,0], \
                abs(trueData[i,1] - slamData[j,1]), \
                abs(trueData[i,2] - slamData[j,2]), \
                abs(trueData[i,3] - slamData[j,3])
                ]]), axis=0)
        #print(errorData[i,:])

#print(slamData)
#print(trueData)
fig = plt.figure()
#plt.plot(trueData[:,0], trueData[:,1] - slamData[:,1], 'r')

plt.plot(errorData[:,0], errorData[:,1], 'r')
plt.plot(errorData[:,0], errorData[:,2], 'b')
#plt.plot(errorData[:,0], errorData[:,3], 'g')
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
fig.suptitle("Odometry Pose Error versus Time")

plt.show()

print("max error X: " + str(np.max(errorData[:,1])))
print("max error Y: " + str(np.max(errorData[:,2])))
print("max error T: " + str(np.max(errorData[:,3])))

print("final error X: " + str(errorData[np.shape(errorData)[0]-1,1]))
print("final error Y: " + str(np.max(errorData[np.shape(errorData)[0]-1,2])))
print("final error T: " + str(np.max(errorData[np.shape(errorData)[0]-1,3])))

errorData[:,1] = np.square(errorData[:,1])
errorData[:,2] = np.square(errorData[:,2])
errorData[:,3] = np.square(errorData[:,3])

RMSErrorX = np.sqrt(np.sum(errorData[:,1])/np.shape(errorData)[0])
RMSErrorY = np.sqrt(np.sum(errorData[:,2])/np.shape(errorData)[0])
RMSErrorT = np.sqrt(np.sum(errorData[:,3])/np.shape(errorData)[0])

print(RMSErrorX)
print(RMSErrorY)
print(RMSErrorT)


