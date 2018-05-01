# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import imutils
import cv2
import numpy as np
from Servo.ServoController import servoController
from ControlSystems import PID
#from matplotlib import pyplot as plt

####################################
# Setup Serial Servo Communication #
####################################
myServo = servoController("/dev/serial0", 4800)

##################
# Servo Commands #
##################
right = "r"
left = "l"
up = "u"
down = "d"

##################
# Setup PID Loop #
##################
Kp, Ki, Kd = 0.065, 0.005, 0.00005
xPID = PID(Kp, Ki, Kd)
yPID = PID(Kp, Ki, Kd)


# Puts string into propper format for the microcontroller to receive
def formatTheta(theta):
    if(theta < 10):
        addString = "00"
    elif(theta < 100):
        addString = "0"
    else:
        addString = ""
    return addString + str(theta)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Create a window for our Trackbar
TrackbarHSV = np.zeros((300, 512, 3), np.uint8)
cv2.namedWindow('TrackbarHSV')

# Create a Trackbar to allow user modify HSV Bitmask
h, s, v, = 20, 100, 65# initial values
def printHSV(x):
    print("h: " + str(h))
    print("s: " + str(s))
    print("v: " + str(v))
cv2.createTrackbar('h', 'TrackbarHSV', 60, 179, printHSV)
cv2.createTrackbar('s', 'TrackbarHSV', 130, 255, printHSV)
cv2.createTrackbar('v', 'TrackbarHSV', 225, 255, printHSV)

# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	TrackbarHSV[:] = [h, s, v]
	TrackbarHSV = cv2.cvtColor(TrackbarHSV, cv2.COLOR_HSV2BGR)
	cv2.imshow('TrackbarHSV', TrackbarHSV)
	start = time.time()# Start the timer
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	image = imutils.rotate(image, 180)# the cammera is upside down, flip image
	
	key = cv2.waitKey(1) & 0xFF
        
        # load the image and resize it to a smaller factor so that
        # the shapes can be approximated better
	resized = imutils.resize(image, width=300)
	ratio = image.shape[0] / float(resized.shape[0])
         
	# Convert BGR to HSV
	hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
	
	# Get info from track bar and apply to HSV Bitmask
	h = cv2.getTrackbarPos('h', 'TrackbarHSV')
	s = cv2.getTrackbarPos('s', 'TrackbarHSV')
	v = cv2.getTrackbarPos('v', 'TrackbarHSV')

	# Create the bitmask
	lowerRange = np.array([h, s, v])
	upperRange = np.array([180, 255, 255])
	mask = cv2.inRange(hsv, lowerRange, upperRange)
	
	# blur image to smoothe contours
	blurred = cv2.GaussianBlur(mask, (5, 5), 0)
			    
	# Bitwise-AND mask and original image
	masked = cv2.bitwise_and(blurred, blurred, mask= mask)
        
        # find contours in the thresholded image and initialize the
        # shape detector
	cnts = cv2.findContours(masked.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        
        # Display the masked image
	cv2.imshow("Masked Image", masked)
        
        #########################
	# User Control Pan Tilt #
        #########################
	controlKey = cv2.waitKey(1)
	if(controlKey == ord('u')):# Up
            myServo.updateAngles(0, -5)# Up 5 deg
	elif(controlKey == ord('d')):# Down
            myServo.updateAngles(0, 5)# Down 5 deg
	elif(controlKey == ord('l')):# Left
            myServo.updateAngles(5, 0)# Left 5 deg
	elif(controlKey == ord('r')):# Right
            myServo.updateAngles(-5, 0)# Right 5 deg
        # A spacebar was pressed, Initiate Bitmap Tracking
	elif(controlKey == ord(' ')):
            TrackImage = True
        	
	try:
            c = max(cnts, key = cv2.contourArea)# Select the largest area
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
     
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            #locationString = "(" + str(cX) + ", " + str(cY) + ")"
            #cv2.putText(image, locationString, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,	0.5, (255, 0, 0), 2)
        
            # Get Dimensions of the Image
            width = 1.5*resized.shape[0]#*ratio
            height = resized.shape[1]#*ratio
	    
            # Stop timer and display frame rate
            stop = time.time()# Start the timer
            deltaT = stop - start
            print("deltaT (s): ", (stop - start))
	    
	    # Update the pixel error
            xError, yError = -(cX - width), (height - cY)
            errorString = "(" + str(xError) + ", " + str(yError) + ")"
            cv2.putText(image, errorString, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,	0.5, (255, 0, 0), 2)
	    
	    # Update the PID loops
            xGain = xPID.updateGain(xError, deltaT)
            yGain = yPID.updateGain(yError, deltaT)
	    
            # Tell Servo to center itself on the centroid of the bitmask
            myServo.updateAngles(xGain, yGain)	
	
	except:            
            cv2.putText(image, "Error", (0, 0), cv2.FONT_HERSHEY_SIMPLEX,	0.5, (0, 0, 255), 2)
            print("No blobs found, don't move")
            
            # Stop timer and display frame rate
            stop = time.time()# Start the timer
            deltaT = stop - start
            print("deltaT (s): " + str(deltaT))
        
	# show the output image
	cv2.imshow("Blobs", image)
        

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
            cv2.destroyAllWindows()
            break
