from aruco import *
from plutodrone import *
import time

height_max= 3
height_min = 0 
height_set=0.75

mini=[1400,1400,1400,1400]
maxi=[1800,1660,1600,1600]

# throttle_min = 1400
# throttle_max = 1800

# roll_min= 1400
# roll_max= 1600

# pitch_min= 1400
# pitch_max= 1600

# yaw_min= 1400
# yaw_max= 1600

throttle_hover = 1500

# previous_error=np.array([0,0,0])
previous_error = np.zeros(4)
previous_error_sum = np.zeros(4)

# yaw_setpoint = 0
# setpoint = np.array([0,0])
# setpoint1 = np.array([0.5,0.5],[0.5,-0.5],[-0.5,-0.5],[-0.5,0.5])
# setpoint2 = np.array([0.5,-0.5])
# setpoint3 = np.array([-0.5,-0.5])


# k = 30
# t = 4

kp=[30,6,6,10]
kd=[0,25,25,30]
ki=[0,0,0,0]


previous_error_height=0
dt=0.1

obj2 = height()
# obj1 = drone() 
obj2.start()
# # obj1.start()
# i = 0
# while True:
# i =0

roll_temp = 0
pitch_temp = 0
throttle_temp = 0
previous_time = 0
error_sum = np.zeros(4)
fxd_tm=time.time()

while 1: 
    while drone.bool_pid:
        # if i == 0:
        #     time.sleep(2)
        # i =1
        # curr_time = time.time()
        # dt = curr_time - previous_time
        # if dt == 0:
        #     dt = 0.1
        # previous_time = curr_time
        # print("dt",dt)

        current_height= height_max-height.distance
        error_height =  height_set - current_height           
        error_roll= - height.y
        # error_pitch= - height.y
        error_pitch= - height.x
        error_yaw= height.yaw
        # print(height.x," ",height.y," ",height.yaw)
        error=[height_set - current_height , error_roll , error_pitch , error_yaw]
        if roll_temp <= 1420 or roll_temp >= 1530:
            error_sum[1] = 0
        if pitch_temp <= 1470 or pitch_temp >= 1530:
            error_sum[2] = 0
        if throttle_temp <= 1400 or throttle_temp >= 1800:
            error_sum[0] = 0

        error_sum = np.add(previous_error_sum, error)
            
        # print("errorsum: ",error_sum)
        # print("prev_sum: ",previous_error_sum)


        # error = np.array(height_set - height.z, setpoint[i][0] - height.x ,setpoint[i][1] - height.y )
        # error_yaw = yaw_setpoint - height.yaw
        throttle_temp= 1500+ 5*(kp[0]*error[0]  +  kd[0]*((error[0]-previous_error[0])/dt)  +  ki[0]* (error_sum[0])*(dt))
        # if (time.time()- fxd_tm)<=3:
        #     roll_temp=1500
        #     pitch_temp=1500
        #     yaw_temp=1500
        # else:
        roll_temp = 1500+ 2*(kp[1]*error[1]  +  kd[1]*((error[1]-previous_error[1])/dt)  +  ki[1]* (error_sum[1])*(dt))
        pitch_temp = 1500- 2*(kp[2]*error[2]  +  kd[2]*((error[2]-previous_error[2])/dt)  +  ki[2]* (error_sum[2]*(dt)))
        yaw_temp = 1500 - 7*(kp[3]*error[3]  +  kd[3]*((error[3]-previous_error[3])/dt)  +  ki[3]* (error_sum[3])*(dt))
        # yaw_temp = 1500+ 7*(kp[3]*error_yaw  +  kd[3]*(error_yaw-previous_error_yaw)/dt)  +  ki[3]* (previous_error_yaw+error_yaw)*dt)
        # print(kd[0]*((error[0]-previous_error[0])/dt))
        # previous_error_height=error_height
        # previous_error_roll=error_roll 
        # previous_error_pitch=error_pitch
        # previous_error_yaw=error_yaw

        previous_error=error
        previous_error_sum = error_sum
        # if np.sqrt(np.sqrt((error[i][0])**2 + (error[i][1])**2)) < 0.2 and i <4:
        #     i = i+1
        #     print("setpoint updated")
        # else:
        #     i = 0
        #     print("setpoint updated")

        throttle=max(mini[0], min(maxi[0],int(throttle_temp) ))
        roll=max(mini[1], min(maxi[1], roll_temp))
        pitch=max(mini[2], min(maxi[2], pitch_temp))
        yaw=max(mini[3], min(maxi[3], yaw_temp))
        # yaw=yaw_temp
        
        # print(throttle,roll,pitch,yaw)
        # 
        
        drone.throttle=throttle
        drone.roll_val=roll
        drone.pitch_val=pitch
        drone.yaw_val=yaw
        
        # print(drone.throttle)

        # print("throttle: ", throttle, "Roll: ",roll,"Pitch: ", pitch,"Yaw :", yaw)
        

        if 0xFF == ord('q'):
            break
        # pass
        time.sleep(0.05)
    # print("not_in_pid")