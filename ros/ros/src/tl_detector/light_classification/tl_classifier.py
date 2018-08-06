from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        result = TrafficLight.UNKNOWN
        hsvimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #hsv red has 2 range
        lower_red1 = np.array([0,43,46])
        upper_red1 = np.array([10,255,255])
        red1 = cv2.inRange(hsvimage, lower_red1 , upper_red1)


        lower_red2 = np.array([156,43,46])
        upper_red2 = np.array([180,255,255])
        red2 = cv2.inRange(hsvimage, lower_red2 , upper_red2)

        converted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)

        blur_img = cv2.GaussianBlur(converted_img,(15,15),0)
        circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,1,30, param1=100,param2=30,minRadius=5,maxRadius=300)

        if circles is not None:
            result = TrafficLight.RED

        return result