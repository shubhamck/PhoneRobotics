import urllib
import cv2
import numpy as np
import time
import json
import tf


with open("../config/moto.json") as f:
    calibration_params = json.load(f)

mtx = np.array(calibration_params["calibration_matrix"])
dist = np.array(calibration_params["distortion_params"])

WHITEBORAD_HEIGHT = 720
WHITEBORAD_WIDTH = 640

whiteboard = np.zeros((WHITEBORAD_HEIGHT, WHITEBORAD_WIDTH))

whiteboard_corners = np.float32(
    [
        [0, 0],
        [WHITEBORAD_WIDTH - 1, 0],
        [0, WHITEBORAD_HEIGHT - 1],
        [WHITEBORAD_WIDTH - 1, WHITEBORAD_HEIGHT - 1],
    ]
)

tag_id_to_index = {0: 0, 20: 1, 15: 2, 5: 3}
pen_x, pen_y = 0, 0

WINDOW_SIZE = 5
p_x_window = []
p_y_window = []


print("calib : ", mtx)
print("dist : ", dist)
# Replace the URL with your own IPwebcam shot.jpg IP:port
url = "http://192.168.0.25:8080/shot.jpg"
# url = 'http://192.168.0.27:4747'
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

parameters = cv2.aruco.DetectorParameters_create()

while True:

    # Use urllib to get the image from the IP camera
    imgResp = urllib.urlopen(url)

    # Numpy to convert into a array
    imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)

    # Finally decode the array to OpenCV usable format ;)
    img = cv2.imdecode(imgNp, -1)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
        img, dictionary, parameters=parameters
    )

    if len(markerCorners) == 4:

        surface_markers = 4 * [[0, 0]]
        surface_markers[tag_id_to_index[markerIds[0][0]]] = [
            markerCorners[0][0][0][0],
            markerCorners[0][0][0][1],
        ]
        surface_markers[tag_id_to_index[markerIds[1][0]]] = [
            markerCorners[1][0][0][0],
            markerCorners[1][0][0][1],
        ]
        surface_markers[tag_id_to_index[markerIds[2][0]]] = [
            markerCorners[2][0][0][0],
            markerCorners[2][0][0][1],
        ]
        surface_markers[tag_id_to_index[markerIds[3][0]]] = [
            markerCorners[3][0][0][0],
            markerCorners[3][0][0][1],
        ]

        surface_markers = np.float32(surface_markers)

        H, mask = cv2.findHomography(
            whiteboard_corners, surface_markers, cv2.RANSAC, 5.0
        )

        H_inv = np.linalg.inv(H)

        hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        # This is your pen
        red_lower = np.array([25, 100, 100], np.uint8)
        red_upper = np.array([30, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        res = cv2.bitwise_and(img, img, mask=red_mask)

        im2, contours, he = cv2.findContours(
            red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        areas = []

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            areas.append(area)

        if len(areas) > 0 and max(areas) > 100:

            max_idx = areas.index(max(areas))
            x, y, w, h = cv2.boundingRect(contours[max_idx])

            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 5)

            points_on_whiteboard = cv2.perspectiveTransform(
                np.float32([[[x, y]]]), H_inv
            )

            p_x = int(points_on_whiteboard[0][0][0])
            p_y = int(points_on_whiteboard[0][0][1])

            if p_x < WHITEBORAD_WIDTH - 50 and p_y < WHITEBORAD_HEIGHT - 50:
                # whiteboard[p_y, p_x] = 255

                if len(p_x_window) > WINDOW_SIZE:
                    p_x_window.pop(0)
                    p_y_window.pop(0)

                p_x_window.append(p_x)
                p_y_window.append(p_y)

                p_x_avg = sum(p_x_window) / len(p_x_window)
                p_y_avg = sum(p_y_window) / len(p_y_window)

                if pen_x == 0 and pen_y == 0:
                    pen_x = p_x_avg
                    pen_y = p_y_avg
                else:
                    whiteboard = cv2.line(
                        whiteboard, (pen_x, pen_y), (p_x_avg, p_y_avg), 255, 4
                    )
                pen_x = p_x_avg
                pen_y = p_y_avg

        else:
            pen_x, pen_y = 0, 0

        cv2.imshow("board", whiteboard)

        if np.all(markerIds != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(
                markerCorners, 0.05, mtx, dist
            )

            for i in range(0, markerIds.size):
                # draw axis for the aruco markers
                cv2.aruco.drawAxis(img, mtx, dist, rvec[i], tvec[i], 0.1)

            # draw a square around the markers
            cv2.aruco.drawDetectedMarkers(img, markerCorners)

        else:
            print("No markers")
    # put the image on screen
    cv2.imshow("IPWebcam", img)

    # To give the processor some less stress
    # time.sleep(0.1)

    # Quit if q is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()
