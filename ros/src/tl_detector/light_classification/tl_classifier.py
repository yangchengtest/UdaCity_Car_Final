from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import rospy
import tensorflow as tf
import os
import yaml
from distutils.version import LooseVersion

class TLClassifier(object):
    def __init__(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        config_string = rospy.get_param("/traffic_light_config")
        config = yaml.load(config_string)
        PATH_TO_FROZEN_GRAPH = None
        if config['is_site']:
            rospy.loginfo("is site")
            PATH_TO_FROZEN_GRAPH = os.path.join(base_path, "model", "frozen_inference_graph_real.pb")
        else:
            PATH_TO_FROZEN_GRAPH = os.path.join(base_path, "model", "frozen_inference_graph.pb")
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)
        self.threshold = .5

    def get_classification_tf(self, img):
        # Bounding Box Detection.
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(img, axis=0)
            (boxes, scores, classes, num) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            num = np.squeeze(num)
        return boxes,scores,classes,num

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        result = TrafficLight.UNKNOWN
        boxes, scores, classes, num = self.get_classification_tf(image)
        if scores[0] > self.threshold:
            if classes[0] == 1:
                result = TrafficLight.GREEN
            elif classes[0] == 2:
                result = TrafficLight.RED
            elif classes[0] == 3:
                pass
        rospy.loginfo("traffic lights %d", result)
        return result
