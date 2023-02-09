import cv2
import numpy as np

# Load the images
image1 = cv2.imread("./Images/Zv4.jpg")
image2 = cv2.imread("./Images/Zv3.jpg")
image3 = cv2.imread("./Images/Zv2.jpg")

# Stack the images vertically
stacked_image = np.vstack((image1, image2,image3))

# Display the combined image
cv2.imshow("Stitched Image", stacked_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
