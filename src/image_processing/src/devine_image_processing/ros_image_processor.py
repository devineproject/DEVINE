# -*- coding: utf-8 -*-
"""Generic ROS Wrapper around image processing classes"""
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from io import BytesIO
try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty
import time
import signal
import inspect
from PIL import Image
from sensor_msgs.msg import CompressedImage
import rospy
import numpy as np


class ImageProcessor(object):
    """ Base interface for an image processor"""

    def processor_name(self):
        """ Return the processor's name """
        return self.__class__.__name__

    def process(self, image, image_payload):
        """ Callback when a new image is received and ready to be processed """
        raise NotImplementedError()


class ROSImageProcessingWrapper(object):
    """ Generic ROS Wrapper around an image processing class """
    image_queue = Queue(2)  # Processing must be on the main thread for TensorFlow compatible processors
    image_processor = None

    def __init__(self, image_processor, receiving_topic):
        if inspect.isclass(image_processor) and issubclass(image_processor, ImageProcessor):
            image_processor = image_processor()
        if not isinstance(image_processor, ImageProcessor):
            raise Exception('The image processor is not an instance of ImageProcessor')
        if not receiving_topic:
            raise Exception('Receiving topic must be set to an image topic')

        self.image_processor = image_processor
        rospy.init_node(image_processor.processor_name())
        rospy.Subscriber(receiving_topic, CompressedImage,
                         self.image_received_callback, queue_size=1)

    def image_received_callback(self, data):
        """ Callback when a new image is received from the topic """
        if self.image_queue.full():
            rospy.logwarn_throttle(30, rospy.get_name() + ' : image receiving rate is too high.')
            self.image_queue.get()
        self.image_queue.put(data)

    def loop(self, process_callback=None):
        """ Looping method to process every image """
        killable_loop = GracefulKiller()
        while True:
            try:
                img_payload = self.image_queue.get(False)
                img = np.array(Image.open(BytesIO(img_payload.data))).astype(np.uint8)
                output = self.image_processor.process(img, img_payload)
                if process_callback:
                    process_callback(output)
            except Empty:
                time.sleep(0.1)
            finally:
                if rospy.is_shutdown() or killable_loop.kill_now:
                    break


class GracefulKiller:
    # Thanks Mayank Jaiswal, https://stackoverflow.com/a/31464349
    kill_now = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill_now = True
