import cv2

rgb_img = raw_input('input the image name ')
rgb_data = cv2.imread(rgb_img, cv2.IMREAD_UNCHANGED)

b_channel, g_channel, r_channel = cv2.split(rgb_data)

alpha_channel = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2GRAY)
img_BGRA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
cv2.imwrite('new_'+rgb_img, img_BGRA)
