#ref: https://www.pyimagesearch.com/2015/03/09/capturing-mouse-click-events-with-python-and-opencv/

#!/usr/bin/env python2
import argparse
import cv2
 
refPt=[]
 
def click(event,x,y,flags,param):
	global refPt
 
	if event==cv2.EVENT_LBUTTONUP:
		print "mouse click at",x,y
		refPt.append((x,y))

	
parser=argparse.ArgumentParser()
parser.add_argument("-i","--image",required=True,help="Path to the image")
args=vars(parser.parse_args())
 
image=cv2.imread(args["image"])
window_name="image to grab 2D correspondence points"
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name,click)
 
while True:
	cv2.imshow(window_name,image)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'c' key is pressed, break from the loop
	if key == ord("c"):
		break
 
print refPt
cv2.destroyAllWindows()