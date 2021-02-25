import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy import ndimage as ndi
from scipy.ndimage.measurements import label
from skimage import img_as_bool
from skimage.io import imread
from skimage.color import rgb2gray
from skimage.morphology import skeletonize, binary_closing
from skimage.morphology import skeletonize
import numpy as np


def largest_component(indices):
    # this function takes a list of indices denoting
    # the white regions of the image and returns the largest
    # white object of connected indices

    return_arr = np.zeros((512, 512), dtype=np.uint8)
    for index in indices:
        return_arr[index[0]][index[1]] = 255

    return return_arr


image = cv2.imread('/Users/valeriy/Documents/sMedX/MarkerBot/camera_vision/test_image.png', 0)
image_gaussian = ndi.gaussian_filter(image, 4)
image_gaussian_inv = cv2.bitwise_not(image_gaussian)
kernel = np.ones((3, 3), np.uint8)

# double thresholding extracting the sides of the rectangle
ret1, img1 = cv2.threshold(image_gaussian_inv, 120, 255, cv2.THRESH_BINARY)
ret2, img2 = cv2.threshold(image_gaussian_inv, 150, 255, cv2.THRESH_BINARY)

double_threshold = img1 - img2
closing = cv2.morphologyEx(double_threshold, cv2.MORPH_CLOSE, kernel)

labeled, ncomponents = label(closing, kernel)
indices = np.indices(closing.shape).T[:, :, [1, 0]]
twos = indices[labeled == 2]
area = [np.sum(labeled == val) for val in range(ncomponents + 1)]

rectangle = largest_component(twos)

cv2.imshow('rectangle', rectangle)
cv2.waitKey(0)




# Step #1 - Skeletonize
im = img_as_bool(rgb2gray(imread('test_image.JPG')))
out = binary_closing(skeletonize(im))

# Convert to uint8
out = 255 * (out.astype(np.uint8))
f, (ax0, ax1) = plt.subplots(1, 2)
ax0.imshow(im, cmap='gray', interpolation='nearest')
ax1.imshow(out, cmap='gray', interpolation='nearest')
plt.show()

img = cv2.imread(R'/Users/valeriy/Documents/sMedX/MarkerBot/camera_vision/test_image.png')
print(img.shape)
img_moments = cv2.moments(
    img[:, :, 0])  # use only one channel here (cv2.moments operates only on single channels images)
print(img_moments)
# print(dir(img_moments))

# calculate centroid (center of mass of image)
x = img_moments['m10'] / img_moments['m00']
y = img_moments['m01'] / img_moments['m00']

# calculate orientation of image intensity (it corresponds to the image intensity axis)
u00 = img_moments['m00']
u20 = img_moments['m20'] - x * img_moments['m10']
u02 = img_moments['m02'] - y * img_moments['m01']
u11 = img_moments['m11'] - x * img_moments['m01']

u20_prim = u20 / u00
u02_prim = u02 / u00
u11_prim = u11 / u00

angle = 0.5 * math.atan(2 * u11_prim / (u20_prim - u02_prim))
print('The image should be rotated by: ', math.degrees(angle) / 2.0, ' degrees')

cols, rows = img.shape[:2]
# rotate the image by half of this angle
rotation_matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), math.degrees(angle / 2.0), 1)
img_rotated = cv2.warpAffine(img, rotation_matrix, (cols, rows))
# print(img_rotated.shape, img_rotated.dtype)

cv2.imwrite(R'/Users/valeriy/Documents/sMedX/MarkerBot/camera_vision/img1_rotated.png', img_rotated)

img_rotated_clone = np.copy(img_rotated)
img_rotated_clone2 = np.copy(img_rotated)

# first method - just calculate bounding rect
bounding_rect = cv2.boundingRect(img_rotated[:, :, 0])
cv2.rectangle(img_rotated_clone, (bounding_rect[0], bounding_rect[1]),
              (bounding_rect[0] + bounding_rect[2], bounding_rect[1] + bounding_rect[3]), (255, 0, 0), 2)


# second method - find columns and rows with biggest sums
def nlargest_cols(a, n):
    col_sums = [(np.sum(col), idx) for idx, col in enumerate(a.T)]
    return sorted(col_sums, key=lambda a: a[0])[-n:]


def nlargest_rows(a, n):
    col_sums = [(np.sum(col), idx) for idx, col in enumerate(a[:, ])]
    return sorted(col_sums, key=lambda a: a[0])[-n:]


top15_cols_indices = nlargest_cols(img_rotated[:, :, 0], 15)
top15_rows_indices = nlargest_rows(img_rotated[:, :, 0], 15)

for a in top15_cols_indices:
    cv2.line(img_rotated_clone, (a[1], 0), (a[1], rows), (0, 255, 0), 1)

for a in top15_rows_indices:
    cv2.line(img_rotated_clone, (0, a[1]), (cols, a[1]), (0, 0, 255), 1)

cv2.imwrite(R'/Users/valeriy/Documents/sMedX/MarkerBot/camera_vision/img2.png', img_rotated_clone)