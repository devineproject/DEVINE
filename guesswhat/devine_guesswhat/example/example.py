import rospy
from std_msgs.msg import String, Float64MultiArray
import h5py
import gzip
import json

SEGMENTATION_TOPIC = '/rcnn_segmentation'
FEATURES_TOPIC = '/vgg16_features'

rospy.init_node('guesswhat_example', anonymous=True)
f = gzip.open('guesswhat.image.jsonl.gz')
seg = json.loads(next(f).decode())
f.close()
h5file = h5py.File('image_features.h5', 'r')
feats = h5file['features'][0]
rospy.loginfo(seg)
rospy.loginfo(feats)

seg_pub = rospy.Publisher(SEGMENTATION_TOPIC, String, queue_size=1, latch=True)
seg_pub.publish(json.dumps(seg))

feats_pub = rospy.Publisher(FEATURES_TOPIC, Float64MultiArray, queue_size=1, latch=True)
feats_pub.publish(Float64MultiArray(data=feats))

rospy.spin()
