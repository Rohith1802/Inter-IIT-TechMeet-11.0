from telnetlib import Telnet
import time
import sys
import struct
from threading import *
from keyboard import *
# from pid0502 import *
# from getheight import *


last_button = None


class drone(Thread):  # creating a class with methods that contain basic drone functions and also to recieve sensory data
    throttle = 1000
    yaw_val = 1500
    pitch_val = 1500
    roll_val = 1500
    armed = None
    bool_pid = False
    
    
    
    def __init__(self, last_button=None):
        super(drone, self).__init__()
        pass

    def make_in(self, command: int, byte_arr: bytes):  # method to create an "IN" and "OUT" communication packets

        cmd = struct.pack(f"<cBB{len(byte_arr)}s", b'<',
                          len(byte_arr), command, byte_arr)

        crc = 0
        for c in cmd[1:]:
            crc ^= c
        crcb = bytes([crc])
        return b"$M" + cmd + crcb

  

    def msp_set_raw_rc(self, roll=1500, pitch=1500, throttle=1000, yaw=1500, aux1=2100, aux2=900, aux3=1500, aux4=1500):# This is a generalized method with equilibrium parameters which are altered to create      payload of individual packets
        payload = struct.pack("<8H", roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)
        return drone.make_in(self, 0xc8, payload)

    def arm(self): # method to arm the vehicle
        return drone.msp_set_raw_rc(self, throttle=1000, aux4=1500)

   #  def box_arm(self):
   #      return drone.msp_set_raw_rc(self, throttle=1000, aux4=1500)

    def disarm(self): # method to disarm the vehicle
        return drone.msp_set_raw_rc(self, aux4=900)

    def takeoff(self):
        return drone.make_in(self, 0xd9, struct.pack("<H", 1))

    def land(self): 
        return drone.make_in(self, 0xd9, struct.pack("<H", 2))

    def roll(self): # method to send a positive roll command
        return drone.msp_set_raw_rc(self, throttle=1500, roll=1700)

    def croll(self): #method to send a counter(negative) roll command
        return drone.msp_set_raw_rc(self, roll=1300,)

    def pitch(self): 
        return drone.msp_set_raw_rc(self, throttle=1500, pitch=1600) 

    def cpitch(self): 
        return drone.msp_set_raw_rc(self, throttle=1500, pitch=1400)

    def yaw(self):
        return drone.msp_set_raw_rc(self, throttle=1500, yaw=2000)

    def cyaw(self):
        return drone.msp_set_raw_rc(self, throttle=1500, yaw=1000)

    def msp_attitude(self):# method to recieve a packet that contains attitude information
        payload = bytearray(6) # creating an empty 6 byte array (empty payload)
        return drone.make_in(0x6c, payload)

    def raw_imu(self): # method to recieve a packet that contains raw imu information
        payload = bytearray(18) #creating an empty 18 byte array(empty payload) 
        return drone.make_in(0x66, payload)

    def msp_altitude(self): # method to recieve a packet that contains altitude information
        payload = bytearray(6) # creating an empty 6 byte array (empty payload)
        return drone.make_in(0x6d, payload)

    def run(self): # the run method is overridden to define the thread body
        with Telnet('192.168.4.1', 23) as tn: # establishing a connection using telnet protocol
            

            print("ready to take commands")
            # last_button = input("Enter command: ")
            try:
                while True:
                  # each keyboard input is assosiated with corresponding drone action
                    last_button = keyboard.run(self)
                    if last_button == "p":
                        drone.armed = False
                        tn.write(self.disarm())


                    elif last_button == "o":
                        tn.write(self.arm())
                        drone.armed = True

                    elif last_button == "t" and drone.armed:
                        print('TAKEOFF START', end='\n', flush=True)

                        tn.write(self.takeoff())
                        drone.armed = True
                        drone.throttle = 1700
                        drone.bool_pid = True

                    elif last_button == "l" and drone.armed:
                        print('LAND START', end='\n', flush=True)
                        tn.write(self.land())
                        time.sleep(2)
                        drone.throttle = 1400
                        drone.armed = True
                        drone.bool_pid = True


                    elif last_button == "d" and drone.armed:
                        print('ROLL START', end='\n', flush=True)
                        tn.write(self.roll())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.roll_val=1700
                        drone.armed = True
                        drone.bool_pid = True


                    elif last_button == "a" and drone.armed:
                        print('Counter roll started', end='\n', flush=True)
                        tn.write(self.croll())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.roll_val=1300
                        drone.armed = True

                    elif last_button == "w" and drone.armed:
                        print('PITCH STARTED', end='\n', flush=True)
                        tn.write(self.pitch())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.pitch_val=1600
                        drone.armed = True

                    elif last_button == "s" and drone.armed:
                        print('counter pitch started', end='\n', flush=True)
                        tn.write(self.cpitch())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.pitch_val=1400
                        drone.armed = True

                    elif last_button == "." and drone.armed:
                        print('YAW STARTED', end='\n', flush=True)
                        tn.write(self.yaw())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.yaw_val=2000
                        drone.armed = True

                    elif last_button == "," and drone.armed:
                        print('counter yaw started', end='\n', flush=True)
                        tn.write(self.cyaw())
                        time.sleep(2)
                        drone.throttle = 1500
                        drone.yaw_val=1000
                        drone.armed = True

                    elif last_button == "b" and drone.armed:
                        drone.throttle = 1500
                        drone.roll_val = 1500
                        drone.pitch_val = 1500
                        drone.yaw_val = 1500
                        
                    # elif last_button==None :
                    #     tvec=height.run(height)
                    #     [x,y,z]=tvec[0][0]
                    #     drone.throttle,drone.roll_val,drone.pitch_val=Quadcopter.run(Quadcopter,[0,0,1],[x,y,z],0.01)
                
                        
                        
                        

                    if not drone.armed: 
                        tn.write(self.disarm())
                        # print('DISARMED', end='\n', flush=True)
                        continue
                    tn.write(self.msp_set_raw_rc(roll=int(drone.roll_val), pitch=int(drone.pitch_val), throttle=int(drone.throttle), yaw=int(drone.yaw_val)))
                    time.sleep(0.2)
                  
                    print('throttle: {:4.2f}    roll: {:4}    pitch {:4}    yaw {:4}'.format(drone.throttle, drone.roll_val, drone.pitch_val, drone.yaw_val), end='\n', flush=True)
                    last_button = None

            except KeyboardInterrupt:
                pass
                tn.write(self.disarm())


d1 = drone() # creating an instance of class drone
obj2 = keyboard() # creating an instance of class keyboard
obj2.start() #initiating  thread 
d1.start() #initiating thread