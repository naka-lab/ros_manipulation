
from __future__ import print_function, unicode_literals
import rospy
from std_msgs.msg import String
import tf
import yaml
import os
import time

##### 送信する情報 ###### 
label = 0
pos = (0.4, 0, 0)
topic_name = "/ar_marker_rec/object_info"
#######################


def ilist(lst): return [ int(i) for i in lst ]
def flist(lst): return [ float(i) for i in lst ]

def send_objects_info(rects, positions, labels):
    br = tf.TransformBroadcaster()
    object_info = []
    for i, p in enumerate(positions):
        br.sendTransform(p,
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(), "ar:%d"%(labels[i]), "camera_link")
        object_info.append(
            { 
                "lefttop" : ilist( rects[i][0] ),
                "rightbottom" : ilist( rects[i][1] ),
                "position" : flist( positions[i] ),
                "label" : int(labels[i])
             }
        )
    pub_objinfo.publish( yaml.dump( object_info )  )

rospy.init_node('object_dymmt', anonymous=True)
pub_objinfo = rospy.Publisher(topic_name, String, queue_size=1)

while not rospy.is_shutdown():
    print("send", "lebel:%d"%label, "pos:%.2f, %.2f, %.2f"%pos )
    send_objects_info( [[(0, 0), (0, 0)]], [pos], [label] )
    time.sleep(1)