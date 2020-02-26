#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import intera_interface
from intera_interface import CHECK_VERSION



class ObjectLocalization:


    def __init__(self):
        rospy.init_node('live_item_pose_node')  # Used to add new items into the scene
        self.has_finished = False
        self.done_pub = rospy.Publisher('/live_pose_node/object/final_pose', Pose, queue_size=10)  # Publishes final position of item for ui to add into xml
        self.pose_sub = rospy.Subscriber('/ik/new_object/live/pose', Pose, callback=self.send_final_to_ui, queue_size=10)  # Get current pose of arm
        self.signal_done_sub = rospy.Subscriber('/ui/new_object/state/done', Bool, callback=self.is_done_callback, queue_size=10)  # UI tells us that it has finished succesfully finding the obj & wants the position
        
        rospy.spin()



    ##  Trivial callback that assigns the received pose to a member variable
    def is_done_callback(self, state):
        print('IS DONE')
        self.has_finished = state.data
        
        # self.done_pub.publish(self.current_pose)



    ##  This callback will send the final pose to the UI
    def send_final_to_ui(self, data):
        print('send final to ui %s' % data)
        
        if self.has_finished:
            self.done_pub.publish(data)




if __name__ == "__main__":
    ol = ObjectLocalization()