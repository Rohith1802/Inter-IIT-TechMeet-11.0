from aruco import *
from plutodrone import *
import time

height_max= 3
height_min = 0 
height_set=0.75

mini=[1400,1490,1490,1400]
maxi=[1800,1510,1510,1600]





previous_error=np.array([0,0])
previous_error = np.zeros(4)
previous_error_sum = np.zeros(4)





yaw_setpoint = 0

setpoint = np.array([[0.5,0.5],[0.5,-0.5],[-0.5,-0.5],[0.5,-0.5]])





kp=[30,6,6,10]
kd=[0,25,25,30]
ki=[0,0,0,0]

previous_error_height=0
dt=0.1

obj2 = height()
obj2.start()
i =0

roll_temp = 0
pitch_temp = 0
throttle_temp = 0
previous_time = 0
error_sum = np.zeros(4)



while True:

    while drone.bool_pid:
    
        current_height= height_max-height.distance
        error_height =  height_set - current_height           
        error_roll= setpoint[i][0] - height.y
        error_pitch= setpoint[i][1] - height.x
        error_yaw= height.yaw
        error=[height_set - current_height , error_roll , error_pitch , error_yaw]
        if roll_temp <= 1420 or roll_temp >= 1530:
            error_sum[1] = 0
        if pitch_temp <= 1470 or pitch_temp >= 1530:
            error_sum[2] = 0
        if throttle_temp <= 1400 or throttle_temp >= 1800:
            error_sum[0] = 0

        error_sum = np.add(previous_error_sum, error)

        throttle_temp= 1500+ 5*(kp[0]*error[0]  +  kd[0]*((error[0]-previous_error[0])/dt)  +  ki[0]* (error_sum[0])*(dt))
        roll_temp = 1500+ 2*(kp[1]*error[1]  +  kd[1]*((error[1]-previous_error[1])/dt)  +  ki[1]* (error_sum[1])*(dt))
        pitch_temp = 1500- 2*(kp[2]*error[2]  +  kd[2]*((error[2]-previous_error[2])/dt)  +  ki[2]* (error_sum[2]*(dt)))
        yaw_temp = 1500 - 7*(kp[3]*error[3]  +  kd[3]*((error[3]-previous_error[3])/dt)  +  ki[3]* (error_sum[3])*(dt))



        # if np.sqrt(np.sqrt((error[i][0])**2 + (error[i][1])**2)) < 0.2 and i <4:
        #     i = i+1
        #     print("setpoint updated")
        # else:
        #     i = 0
        #     print("setpoint updated")

        # if np.sqrt(np.sqrt((error[0])**2 + (error[1])**2 + (error[2])**2)) < 0.2 and i <4:
        if np.sqrt(np.sqrt((error[1])**2 + (error[2])**2 )) < 0.2 and i <4:
            
            
            i +=1     
            # drone.throttle=1500
            # drone.roll_val=1500
            # drone.pitch_val=1500
            # drone.yaw_val=1500
            time.sleep(2)
        elif i >= 4:
            i = 0
            # drone.throttle=1500
            # drone.roll_val=1500
            # drone.pitch_val=1500
            # drone.yaw_val=1500
            time.sleep(2)




        throttle=max(mini[0], min(maxi[0],int(throttle_temp) ))
        roll=max(mini[1], min(maxi[1], roll_temp))
        pitch=max(mini[2], min(maxi[2], pitch_temp))
        yaw=max(mini[3], min(maxi[3], yaw_temp))
        
        drone.throttle=throttle
        drone.roll_val=roll
        drone.pitch_val=pitch
        drone.yaw_val=yaw

        # print("throttle: ", drone.throttle, "Roll: ",drone.roll_val,"Pitch: ", drone.pitch_val,"Yaw :", drone.yaw_val)

        if 0xFF == ord('q'):
            break
    pass