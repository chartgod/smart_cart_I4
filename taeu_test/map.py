import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
img = mpimg.imread('/home/user1/catkin_ws/src/smart_cart_I4/Smartcart_ROS/rosbook_kobuki/kobuki_navigation/maps/map_1/map.pgm')
# x 180-350 170 /8 y 220-50 170 /8  170/8 => 1m=21.25픽셀

#ros x,y좌표
x = 1.0
y = 1.0
#원 포인트
x1 = 180+(x * 170 / 8)
y1 = 220-(y * 170 / 8)
red = (0,0,255)
blue = (0,255,0)
image_circle_1 = cv2.circle(img, (int(x1),int(y1)), 3, red, -1)
print('This image is:', type(image_circle_1), 'with dimensions:', image_circle_1.shape)
plt.imshow(image_circle_1)
plt.show()