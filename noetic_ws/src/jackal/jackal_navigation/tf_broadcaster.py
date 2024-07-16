#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class TFBroadcast():
    
    def __init__(self):
        rospy.init_node('tf_broadcaster')
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.br = tf.TransformBroadcaster()
        self.odom_republisher = rospy.Publisher('/odometry', Odometry, queue_size=1)

    def odom_callback(self, msg):
        self.br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                              (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                              rospy.Time.now(),
                              "base_link",
                              "odom")
        self.odom_republisher.publish(msg)

    def broadcast_static_transforms(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # Publish /front_mount to /Lidar transform
            self.br.sendTransform((0.0, 0.0, 0.06),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "/laser",
                                  "/front_mount")
            rate.sleep()

if __name__ == '__main__':
    try:
        tf_br = TFBroadcast()
        tf_br.broadcast_static_transforms()
    except rospy.ROSInterruptException:
        pass
