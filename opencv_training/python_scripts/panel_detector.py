import numpy as np
import cv2

# Button panel cascade file
panel_cascade = cv2.CascadeClassifier('button_panel_classifier.xml')

cap = cv2.VideoCapture(0)

while 1:
	ret, img = cap.read()
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	panels = panel_cascade.detectMultiScale(img, 45, 80)

	# Mark the panel with a rectangle
	for (x,y,w,h) in panels:
	    cv2.rectangle(img, (x,y), (x+w,y+h), (255,255,0), 2)

	cv2.imshow('img', img)

	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break

cap.release()
cv2.destroyAllWindows()
