from jetcam.csi_camera import CSICamera
import cv2
import numpy as np

def nothing(x):
    pass

cap = CSICamera(capture_device=0, width=640, height=480)
#cap = cv2.VideoCapture(0)
cv2.namedWindow("RikibotHSVBars")

cv2.createTrackbar("LH", "RikibotHSVBars", 0, 179, nothing)
cv2.createTrackbar("LS", "RikibotHSVBars", 0, 255, nothing)
cv2.createTrackbar("LV", "RikibotHSVBars", 0, 255, nothing)
cv2.createTrackbar("UH", "RikibotHSVBars", 179, 179, nothing)
cv2.createTrackbar("US", "RikibotHSVBars", 255, 255, nothing)
cv2.createTrackbar("UV", "RikibotHSVBars", 255, 255, nothing)

while True:
    image = cap.read()
    frame = cv2.flip(image, 0 )
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("LH", "RikibotHSVBars")
    l_s = cv2.getTrackbarPos("LS", "RikibotHSVBars")
    l_v = cv2.getTrackbarPos("LV", "RikibotHSVBars")
    u_h = cv2.getTrackbarPos("UH", "RikibotHSVBars")
    u_s = cv2.getTrackbarPos("US", "RikibotHSVBars")
    u_v = cv2.getTrackbarPos("UV", "RikibotHSVBars")

    lower_blue = np.array([l_h, l_s, l_v])
    upper_blue = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("RikibotOrgin", frame)
    cv2.imshow("RikibotMask", mask)
    cv2.imshow("RikibotResult", result)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
