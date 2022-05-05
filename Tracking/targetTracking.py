import numpy as np
import cv2

cap = cv2.VideoCapture("TargetVideo.MP4")
font = cv2.FONT_HERSHEY_COMPLEX

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([165, 50, 20])
    upper_blue = np.array([179, 110, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    bluecnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(bluecnts) > 0:
        blue_area = max(bluecnts, key = cv2.contourArea)
        (xg, yg, wg, hg) = cv2.boundingRect(blue_area)
        cv2.rectangle(frame, (xg, yg), (xg + wg, yg + hg), (0, 255, 2), 2)
        cv2.putText(frame, "TARGET HERE", (xg, yg - 15), font, 1, (220, 0, 0), 2, cv2.LINE_AA)

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    k = cv2.waitKey(5)
    if k == 27:
        break