import numpy as np
import cv2

# Set up video capture with video path, or 0 for the device's default camera
cap = cv2.VideoCapture("TargetVideo.MP4")
font = cv2.FONT_HERSHEY_COMPLEX

# Infinite loop while system is running
while True:
    # Read the current frame and make it an image, then convert it's colour from RGB to HSV to be read
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Set the lower and upper bounds of the colour you're looking for
    # Preset options for comp target colours need to be added
    lower_blue = np.array([165, 50, 20])
    upper_blue = np.array([179, 110, 255])

    # Make a mask that displays only the pixels wihin the range, then create a contour around it
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    bluecnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # If a contour is found, then run this code that will:
    # Display the bounding box of where the target is along with some text
    # Determine the midpoints of the contour and then set that as the target's coordinates
    # Display the target's coordinates at the top left
    # The coordinates will be sent to the automated flight system
    if len(bluecnts) > 0:
        blue_area = max(bluecnts, key = cv2.contourArea)
        (xg, yg, wg, hg) = cv2.boundingRect(blue_area)
        cv2.rectangle(frame, (xg, yg), (xg + wg, yg + hg), (0, 255, 2), 2)
        cv2.putText(frame, "TARGET HERE", (xg, yg - 15), font, 1, (220, 0, 0), 2, cv2.LINE_AA)
        targetX = (xg + wg) / 2
        targetY = (yg + hg) / 2
        cv2.putText(frame, "Target Coords: " + str(targetX) + ", " + str(targetY), (0, 30), font, 1, (220, 0, 0), 2, cv2.LINE_AA)

    # Display 2 windows. One with the raw video, and one for the mask
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    k = cv2.waitKey(5)
    if k == 27:
        break