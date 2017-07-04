import cv2
vc=cv2.VideoCapture(0)
if vc.isOpened():
	rval, frame=vc.read()
else:
	rval=False
	print "Fail"
	
while rval == True:
	cv2.imshow('Video', frame)
	rval, fram = vc.read()
	key = cv2.waitKey(20)
	if key == 27:
		break
cv2.destroyWindow('Video')
