 #The following program shows the procedure to make a mosiac collage of the image input

import numpy as np
import cv2
#Taking the image from the webcam of my computer

cap = cv2.VideoCapture(0)
#Inputting and resizing the images so that they can fit into the desired collage
while True:
	_,img = cap.read()
	img_1 = cv2.resize(img,(50,50))

# applying the laplacian transformation
	laplacian = cv2.Laplacian(img,cv2.CV_64F)

	laplacian_1 = cv2.resize(laplacian,(225,230))
	laplacian_2 = cv2.resize(laplacian,(200,230))
	laplacian_3 = cv2.resize(laplacian,(225,210))
	laplacian_4 = cv2.resize(laplacian,(200,210))

# applying the sobel filter for image derivative 
	sobel_x = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=5)
	sobel_x_1 = cv2.resize(sobel_x,(45,50))
	sobel_x_2 = cv2.resize(sobel_x,(40,50))
	sobel_x_3 = cv2.resize(sobel_x,(40,40))

	sobel_y = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=5)
	sobel_y_1 = cv2.resize(sobel_y,(45,40))

#applying the canny edge detection algorithm
	canny = cv2.Canny(img,50,100)
	canny_iso = cv2.cvtColor(canny,cv2.COLOR_GRAY2RGB)
	canny_1 = cv2.resize(canny_iso,(225,280))
	canny_2 = cv2.resize(canny_iso,(200,280))
	canny_3 = cv2.resize(canny_iso,(200,260))
	canny_4 = cv2.resize(canny_iso,(225,260))

#Let's create a pure black image of dimension 1000 X 1000
	img_black = np.zeros((1000,1000,3),np.uint8)

#organising the re-sized images of the collage
#ARRANGING THE LEFT-MOST COLUMN OF THE COLLAGE

	img_black[0:230,0:225] = laplacian_1

	for i in range(0,225,45):
		img_black[230:270,i:i+45] = sobel_y_1

	img_black[270:480,0:225] = laplacian_3

	for i in range(0,225,45):
		img_black[480:520,i:i+45] = sobel_y_1

	img_black[520:730,0:225] = laplacian_3

	for i in range(0,225,45):
		img_black[730:770,i:i+45] = sobel_y_1

	img_black[770:1000,0:225] = laplacian_1

#ARRANGING THE SECOND COLUMN OF THE COLLAGE
	for i in range(0,1000,50):
		img_black[i:i+50,225:275] = img_1

#ARRANGING THE THIRD COLUMN OF THE COLLAGE
	for i in range(275,475,40):
		img_black[0:50,i:i+40] = sobel_x_2

	img_black[50:330,275:475] = canny_2

	for i in range(275,475,40):
		img_black[330:370,i:i+40] = sobel_x_3

	img_black[370:630,275:475] = canny_3

	for i in range(275,475,40):
		img_black[630:670,i:i+40] = sobel_x_3

	img_black[670:950,275:475] = canny_2

	for i in range(275,475,40):
		img_black[950:1000,i:i+40] = sobel_x_2

#ARRANGING THE FOURTH COLUMN WHICH IS THE SAME AS THE SECOND COLUMN
	for i in range(0,1000,50):
		img_black[i:i+50,475:525] = img_1

#ARRANGING THE FIFTH COLUMN WHICH RESEMBLES THE FIRST COLUMN
	img_black[0:230,525:725] = laplacian_2

	for i in range(525,725,40):
		img_black[230:270,i:i+40] = sobel_x_3

	img_black[270:480,525:725] = laplacian_4

	for i in range(525,725,40):
		img_black[480:520,i:i+40] = sobel_x_3

	img_black[520:730,525:725] = laplacian_4

	for i in range(525,725,40):
		img_black[730:770,i:i+40] = sobel_x_3

	img_black[770:1000,525:725] = laplacian_2

#ARRANGING THE SIXTH COLUMN WHICH RESEMBLES THE SECOND AND FOURTH COLUMNS
	for i in range(0,1000,50):
		img_black[i:i+50,725:775] = img_1

#ARRANGING THE SEVENTH COLUMN WHICH RESEMBLES THE THIRD COLUMN
	for i in range(775,1000,45):
		img_black[0:50,i:i+45] = sobel_x_1

	img_black[50:330,775:1000] = canny_1

	for i in range(775,1000,45):
		img_black[330:370,i:i+45] = sobel_y_1

	img_black[370:630,775:1000] = canny_4

	for i in range(775,1000,45):
		img_black[630:670,i:i+45] = sobel_y_1

	img_black[670:950,775:1000] = canny_1

	for i in range(775,1000,45):
		img_black[950:1000,i:i+45] = sobel_x_1

	img_black_1 = cv2.resize(img_black,(640,480))

	cv2.imshow('collage',img_black_1)
#press esc to close the window
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break	
cap.release()
cv2.destroyAllWindows()