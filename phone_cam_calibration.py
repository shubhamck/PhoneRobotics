import urllib
import cv2
import numpy as np
import time
import argparse
import json

parser  = argparse.ArgumentParser(description='Calibrate any IP Webcam')

parser.add_argument('-ip', '--ip_address', type=str, metavar='', required=True, help='IP Address of the Webcam')
parser.add_argument('-port', '--port_number', type=int, metavar='', default=8080, help='Port Number')
parser.add_argument('-V', '--vertical', type=int, metavar='', default=6, help='Chessboard Number of corners in longest side')
parser.add_argument('-H', '--horizontal', type=int, metavar='', default=5, help='Chessboard Number of corners in shortest side')
parser.add_argument('-F', '--frames', type=int, metavar='', default=8, help='Number of frames to capture')
parser.add_argument('-phone', '--phone_name', type=str, metavar='', default='sample', help='Name of your phone')



args = parser.parse_args()



# Replace the URL with your own IPwebcam shot.jpg IP:port
ip = args.ip_address
port = args.port_number
url = 'http://'+ ip + ':' + str(port) + '/shot.jpg'
filename = args.phone_name

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

NUM_VERTICAL_CORNERS = args.vertical
NUM_HORIZONTAL_CORNERS = args.horizontal
TOTAL_NUM_FRAME_TO_CAPTURE = args.frames

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((NUM_HORIZONTAL_CORNERS*NUM_VERTICAL_CORNERS,3), np.float32)
objp[:,:2] = np.mgrid[0:NUM_VERTICAL_CORNERS,0:NUM_HORIZONTAL_CORNERS].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

num_valid_frames = 0


while True:
    # Use urllib to get the image from the IP camera
    imgResp = urllib.urlopen(url)

    # Numpy to convert into a array
    imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)

    # Finally decode the array to OpenCV usable format ;)
    img = cv2.imdecode(imgNp, -1)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (NUM_VERTICAL_CORNERS,NUM_HORIZONTAL_CORNERS),None)

    # put the image on screen
    # cv2.imshow('IPWebcam', img)

    # To give the processor some less stress
    # time.sleep(0.1)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print "Captured : ", num_valid_frames
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

        num_valid_frames = num_valid_frames + 1

    if num_valid_frames >= TOTAL_NUM_FRAME_TO_CAPTURE:
        break

    # Quit if q is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
params = {
    "calibration_matrix": mtx.tolist(),
    "distortion_params" : dist.tolist()
}

with open("config/" + filename + "_cam_calibration.json", "w") as outfile:  
    json.dump(params, outfile) 

print "Calibration Matrix : ", mtx
print "distortion : ", dist




