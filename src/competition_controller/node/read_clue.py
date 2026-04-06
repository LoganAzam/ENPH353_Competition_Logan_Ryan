#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
import string
import tensorflow as tf
import  os

# Needs to be global
pub_score = None

bridge = CvBridge()

# Import CNN Model
model_path = os.path.expanduser("~/ros_ws/src/competition_controller/models/conv_model.tflite")
interpreter = tf.lite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# One hot encoding function for characters
characters = characters = string.ascii_uppercase + string.digits + ' '

def convert_to_one_hot(Y_char):
  Y_ord = ord(Y_char)
  if Y_ord > 64:
    Y = Y_ord - 65
  elif Y_ord > 47:
    Y = Y_ord - 48 + 26
  else:
    Y = 36
  ohe = np.zeros(37)
  ohe[Y] = 1.0
  return ohe

def convert_from_one_hot(ohe):
  Y = np.argmax(ohe)
  return characters[Y]

# Parameters
# Add black border to get full clueboard
border = 20
# HSV Filter to find clueboard
lowerHSV_bound = np.array([100, 125, 30])
upperHSV_bound = np.array([140, 255, 255])
# Find clueboard Border
# Find Inner Clueboard Border Points
# Compute Homography
boardWidth = 130
boardHeight = 76
templatePoints = np.array([[[0, 0]],
                               [[boardWidth, 0]],
                               [[boardWidth-1, boardHeight-1]],
                               [[0, boardHeight-1]]]
                              , dtype=np.float32)
# Cut out Clue Type
clue_type_x_min = 48 # 50 for colab
clue_type_x_max = 118 # 120 for colab
clue_type_y_min = 5
clue_type_y_max = 25
lowerHSV_clue_type_bound = np.array([100, 60, 0])
upperHSV_clue_type_bound = np.array([140, 255, 255])
# Get the start of the clue type
# Convolve with Clue Type Kernels
kernelW = 70
kernelH = 20
textX = 6
textY = 1
fontSize = 16
type_subtract_value = 50
# Cut out Clue Value
clue_val_x_min = 6
clue_val_x_max = 123
clue_val_y_min = 49
clue_val_y_max = 63
# Get the start of the message
# Break into Characters
charH = 70
charW = 50
# Put characters through CNN

# Create Kernels for Clue Type
monospace = ImageFont.truetype(font="/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf", size=fontSize)
kernelArray = []
clueTypeArray = ["SIZE", "VICTIM", "CRIME", "TIME", "PLACE", "MOTIVE", "WEAPON", "BANDIT"]
for clueType in clueTypeArray:
  kernel_PIL = PILImage.fromarray(np.zeros([kernelH, kernelW], dtype=np.uint8))
  draw = ImageDraw.Draw(kernel_PIL)
  draw.text((textX, textY), clueType, fill=255, font=monospace, stroke_width=1)
  typeKernel = np.array(kernel_PIL).astype(np.float32) / 255.0
  typeKernel[typeKernel == 0] = -1.0
  kernelArray.append(typeKernel)

# Reorder the points of a rectangle
def reshapeRectangle(rectangle):
  rectanglePoints = rectangle.reshape(-1, 2)
  corners = np.zeros((4,2), dtype=rectanglePoints.dtype)
  s = rectanglePoints.sum(axis=1)
  diff = np.diff(rectanglePoints, axis=1)
  corners[0] = rectanglePoints[np.argmin(s)]
  corners[2] = rectanglePoints[np.argmax(s)]
  corners[1] = rectanglePoints[np.argmin(diff)]
  corners[3] = rectanglePoints[np.argmax(diff)]
  return corners.reshape(4, 1, 2)

def readClue(cv2Image):
  # Add black border to get full clueboard
  cv2Image = cv2.copyMakeBorder(cv2Image, border, border, border, border, cv2.BORDER_CONSTANT, value=(0, 0, 0))

  # HSV Filter to find clueboard
  hsv_image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2HSV)
  hsv_threshold_img = cv2.inRange(hsv_image, lowerHSV_bound, upperHSV_bound)

  # Find clueboard Border
  num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(hsv_threshold_img)
  max_area = 0
  max_area_label = 0
  if num_labels <= 1:
    return None, None
  for i in range(1, num_labels):
    if stats[i, 4] > max_area:
      max_area = stats[i, 4]
      max_area_label = i
  largest_object_mask = np.uint8(labels == max_area_label) * 255

  # Find Inner Clueboard Border Points
  contours, hierarchy = cv2.findContours(largest_object_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
  if len(contours) < 2:
    return None, None
  sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
  inside_contour = sorted_contours[1]
  perimeter = cv2.arcLength(inside_contour, True)
  rectangle = cv2.approxPolyDP(inside_contour, 0.02 * perimeter, True)
  if len(rectangle) != 4:
    return None, None
  rectangleCorners = reshapeRectangle(rectangle)

  # Compute Homography
  homographyMatrix, homographyMask = cv2.findHomography(rectangleCorners, templatePoints, cv2.RANSAC)
  clueboard_img = cv2.warpPerspective(cv2Image, homographyMatrix, (boardWidth, boardHeight))

  # Cut out Clue Type
  clue_type_img = clueboard_img[clue_type_y_min:clue_type_y_max, clue_type_x_min:clue_type_x_max]
  clue_type_hsv = cv2.cvtColor(clue_type_img, cv2.COLOR_BGR2HSV)
  clue_type_gray = cv2.inRange(clue_type_hsv, lowerHSV_clue_type_bound, upperHSV_clue_type_bound)

  # Convolve with Clue Type Kernels
  best_clue_type_match = 0
  best_clue_type_index = 0
  for i, kernel in enumerate(kernelArray):
    conv_val = np.sum((np.array(clue_type_gray, dtype=np.float32) - type_subtract_value) * np.array(kernel, dtype=np.float32))
    if conv_val > best_clue_type_match:
      best_clue_type_match = conv_val
      best_clue_type_index = i
  clueType = clueTypeArray[best_clue_type_index]

  # Cut out Clue Value
  clue_val_full_img = clueboard_img[clue_val_y_min:clue_val_y_max, clue_val_x_min:clue_val_x_max]
  clue_val_gray = cv2.cvtColor(clue_val_full_img, cv2.COLOR_BGR2GRAY)
  clue_val_normalized = 255 - cv2.normalize(clue_val_gray, None, 0, 255, cv2.NORM_MINMAX)
  clue_val_resized = cv2.resize(clue_val_normalized, (charW*12, charH))
  _, clue_val_img = cv2.threshold(clue_val_resized, 90, 255, cv2.THRESH_BINARY)

  # Get the start of the message
  coords = np.argwhere(clue_val_img != 0)
  if coords.size != 0:
    row, col = coords[np.argmin(coords[:, 1])]
    shift_distance = col - clue_val_x_min + 1
    if shift_distance > 0:
      clue_val_img[:, :-shift_distance] = clue_val_img[:, shift_distance:]
      clue_val_img[:, -shift_distance:] = 0

  # Break into Characters
  messageArray = []
  for i in range(12):
    nextCharacter = np.array(clue_val_img[:, charW*i:charW*(i+1)], dtype=np.uint8)
    messageArray.append(nextCharacter)

  # Put characters through CNN
  clueVal = ''
  for char_img in messageArray:
    char_img = char_img / 255.0
    char_img = np.expand_dims(char_img, -1)
    char_img = np.expand_dims( char_img, 0)

    # Use the model
    interpreter.set_tensor(input_details[0]['index'], char_img.astype(np.float32))
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])

    clueVal += convert_from_one_hot(output_data)

  return (best_clue_type_index + 1), clueVal

def callback(data):
    cv2Image = bridge.imgmsg_to_cv2(data, "bgr8")
    clueType, clueVal = readClue(cv2Image)
    if (clueType is not None) and (clueVal is not None):
      submitMessage = "TeamID,Password," + str(clueType) + "," + clueVal
      pub_score.publish(submitMessage)

    # if not os.path.exists("saved_images"):
    #     os.makedirs("saved_images")
    # global count
    # count += 1
    # filename = os.path.join("saved_images", f"image_{count:03d}.png")
    # cv2.imwrite(filename, cv2Image.copy())

def main():
    global pub_score
    # global pub_view
    rospy.init_node('read_clue_node', anonymous=True)

    # Initialize publishers and subscribers
    pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
    # pub_view = rospy.Publisher('/image_viewer', Image, queue_size=1)
    rospy.Subscriber('/clue_images', Image, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()