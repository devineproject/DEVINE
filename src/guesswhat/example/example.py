# -*- coding: utf-8 -*-
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
from std_msgs.msg import Header
import h5py
import gzip
import json
from devine_config import topicname
from devine_image_processing.msg import SegmentedImage, SceneObject, VGG16Features

SEGMENTATION_TOPIC = topicname('objects')
FEATURES_TOPIC = topicname('image_features')

rospy.init_node('guesswhat_example', anonymous=True)
f = gzip.open('guesswhat.image.jsonl.gz')
seg = json.loads(next(f).decode())
f.close()
h5file = h5py.File('image_features.h5', 'r')
feats = h5file['features'][0]

head = Header(stamp=rospy.Time.now())
seg_msg = SegmentedImage(header=head, objects=[])
for obj in seg['objects']:
    obj_t = SceneObject(
      category_id=obj['category_id'],
      category_name=obj['category']
    )
    obj_t.bounding_box.x_offset = int(obj['bbox'][0])
    obj_t.bounding_box.y_offset = int(obj['bbox'][1])
    obj_t.bounding_box.width = int(obj['bbox'][2])
    obj_t.bounding_box.height = int(obj['bbox'][3])
    seg_msg.objects.append(obj_t)

seg_pub = rospy.Publisher(SEGMENTATION_TOPIC, SegmentedImage, queue_size=1, latch=True)
seg_pub.publish(seg_msg)

feats_pub = rospy.Publisher(FEATURES_TOPIC, VGG16Features, queue_size=1, latch=True)
feats_pub.publish(VGG16Features(header=head, data=feats))

rospy.spin()
