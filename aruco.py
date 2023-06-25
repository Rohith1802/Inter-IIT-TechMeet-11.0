import cv2
import numpy as np
from threading import *
import pyrealsense2 as rs
import time

class height(Thread):

    distance = 0
    
    x=0
    y=0
    z=0
    yaw = 0

    def __init__(self):
        
        super(height,self).__init__()
        height.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        height.parameters = cv2.aruco.DetectorParameters()
        height.parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        
        height.markerLength = 0.07

        height.focal_length=3.6
        height.width,height.length=(1080,720)    

        height.cameraMatrix =np.array(  [[619.4827104748699,0.0,330.86654279772415],
                                         [0.0,618.5748144541332,276.5243455233969],
                                         [0.0,0.0,1.0]])
        height.distCoeffs = np.array( [[0.02800177528602659, 0.5453132261116592,0.013942705654592895,0.00796046432761273,-2.073190153728887]])
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        pipeline.start(config)
        
    def run(self,mirror=False):

    
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        pipeline.start(config)
        print("sleeping for 1")
        time.sleep(2)

        while True:

            frames = pipeline.wait_for_frames()
            # time.sleep(2)

            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())


            corners, ids, _ = cv2.aruco.detectMarkers(color_image, height.aruco_dict, parameters=height.parameters)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, height.markerLength, height.cameraMatrix, height.distCoeffs)
                # print("tvecs: ",tvecs)
                height.y,height.x,height.distance = tvecs[0][0]
                # print("x y z :", height.x,height.y,height.distance)

                # print("norm: ", np.linalg.norm(tvecs))

                rotation_matrix,_= cv2.Rodrigues(rvecs)

                sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
                singular = sy < 1e-6

                if not singular:
                #     height.x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                #     height.y = np.arctan2(-rotation_matrix[2, 0], sy)
                    height.yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                else:
                    # height.x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                    # height.y = np.arctan2(-rotation_matrix[2, 0], sy)
                    height.yaw = 0

                # print("yaw: ", height.yaw)

                for i in range(len(ids)):
                    cv2.aruco.drawDetectedMarkers(color_image, corners)
                #     height.distance = np.linalg.norm(tvecs[i])

                # print("Distance: ", height.distance,"roll : ",height.x ,"pitch :", height.y,"yaw", height.z)

            cv2.imshow("Webcam", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        pipeline.stop()
        cv2.destroyAllWindows()



# obj = height()
# obj.start()