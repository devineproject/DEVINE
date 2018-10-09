'''Generic ROS Wrapper around image processing classes'''
try:
    from queue import Queue, Empty
except:
    from Queue import Queue, Empty

import time
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import signal
import inspect

class ImageProcessor(object):
    ''' Base interface for an image processor'''
    payload = None
    def update_payload(self, payload):
        '''Private func to update the payload'''
        self.payload = payload

    def processor_name(self):
        '''Return the processor's name'''
        return self.__class__.__name__

    def process(self, image):
        '''Callback when a new image is received and ready to be processed'''
        raise NotImplementedError()

class ROSImageProcessingWrapper(object):
    '''Generic ROS Wrapper around an image processing class'''
    image_queue = Queue(2) # Processing must be on the main thread for TensorFlow compatible processors
    image_processor = None

    def __init__(self, image_processor, receiving_topic):
        if inspect.isclass(image_processor) and issubclass(image_processor, ImageProcessor):
            image_processor = image_processor()
        if not isinstance(image_processor, ImageProcessor):
            raise Exception("The image processor is not an instance of ImageProcessor")
        if not receiving_topic:
            raise Exception("Receiving topic must be set to an image topic")
        self.image_processor = image_processor
        rospy.init_node(image_processor.processor_name())        
        rospy.Subscriber(receiving_topic, CompressedImage,
                         self.image_received_callback, queue_size=1)

    def image_received_callback(self, data):
        '''Callback when a new image is received from the topic'''
        if self.image_queue.full():
            rospy.logwarn(rospy.get_name() + " : image receiving rate is too high.")
            self.image_queue.get()
        self.image_queue.put(data)

    def loop(self, process_callback=None):
        '''Looping method to process every image'''
        killable_loop = GracefulKiller()
        while True:
            try:
                img = self.image_queue.get(False)
                self.image_processor.update_payload(img)
                output = self.image_processor.process(np.asarray(bytearray(img.data),dtype=np.uint8))
                if process_callback:
                    process_callback(output)
            except Empty:
                time.sleep(0.5)
            finally:
                if rospy.is_shutdown() or killable_loop.kill_now:
                    break

# Thanks Mayank Jaiswal, https://stackoverflow.com/a/31464349
class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self,signum, frame):
        self.kill_now = True
