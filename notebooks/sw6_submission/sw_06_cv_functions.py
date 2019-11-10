import cv2
import numpy as np
import rospy
import math

## Software Exercise 6: Choose your category (1 or 2) and replace the cv2 code by your own!

## CATEGORY 1
def inRange(hsv_image, low_range, high_range):
	dist = np.ones((hsv_image.shape[0],hsv_image.shape[1]), dtype='uint8')
	for ch in range(hsv_image.shape[-1]):
		mask_low = hsv_image[:,:,ch] > low_range[ch]
		dist = dist * mask_low
		mask_high = hsv_image[:,:,ch] < high_range[ch]
		dist = dist * mask_high
	return dist
	#return cv2.inRange(hsv_image, low_range, high_range)

def bitwise_or(bitwise1, bitwise2):
	dist = np.bitwise_or(bitwise1,bitwise2)
	### OR:
	# dist = np.clip(bitwise1 + bitwise2,0,1)
	return dist
	# return cv2.bitwise_or(bitwise1, bitwise2)

def bitwise_and(bitwise1, bitwise2):
	dist = np.bitwise_and(bitwise1,bitwise2)
	### OR:
	# dist = bitwise1 * bitwise2
	return dist
	# return cv2.bitwise_and(bitwise1, bitwise2)

def getStructuringElement(shape, size):
	if shape==0:
		dist = np.ones(size[::-1], dtype='uint8')
	elif shape==1:
		cx, cy = int(np.floor((size[1])/2)),int(np.floor((size[0])/2)) 
		dist = np.zeros(size[::-1],dtype='uint8')
		dist[cx,:] = 1
		dist[:,cy] = 1
	elif shape==2:
		dist = np.zeros(size[::-1],dtype='uint8')
		a, b = int(np.floor((size[1])/2)),int(np.floor((size[0])/2))
		for r in range(size[1]):
			diff_y = r - a
			diff_x = int(round(((1 - (diff_y**2)/(a**2))*(b**2))**0.5))
			left_margin = int(b) - diff_x
			right_margin = int(b) + diff_x + 1
			dist[r,left_margin:right_margin] = 1
	else:
		return getStructuringElement(2, size)
	return dist
	# return cv2.getStructuringElement(shape, size)
	

def dilate(bitwise, kernel):
	top_margin = int(np.floor((kernel.shape[1])/2))
	left_margin = int(np.floor((kernel.shape[0])/2))
	dist = np.zeros(bitwise.shape, dtype='uint8')
	zero_padded_dist = np.pad(bitwise,((top_margin,top_margin),(left_margin,left_margin)),mode='constant')
	for row in range(top_margin, bitwise.shape[0]+top_margin):
		for col in range(left_margin, bitwise.shape[1]+left_margin):
			top_r = int(np.floor((kernel.shape[1])/2))
			bot_r = kernel.shape[1] - top_r - 1
			left_r = int(np.floor((kernel.shape[0])/2))
			right_r = kernel.shape[0] - left_r - 1
			window = zero_padded_dist[row-top_r:row+bot_r+1, col-left_r:col+right_r+1] * kernel
			dist[row-top_margin,col-left_margin] = np.amax(window)
	return dist
	# return cv2.dilate(bitwise,kernel)


## CATEGORY 2
def Canny(image, threshold1, threshold2, apertureSize=3):
	return cv2.Canny(image, threshold1, threshold2, apertureSize=3)


## CATEGORY 3 (This is a bonus!)
def HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap):
	return cv2.HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap)