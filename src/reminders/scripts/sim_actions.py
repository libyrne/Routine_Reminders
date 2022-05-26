#!/usr/bin/env python3 

# ROS imports 
import rospy 
from std_msgs.msg import String

# Python imports
from random import randint


class Action():
    def __init__(self):

        # Subscriber for labels (TO DO: make a temporary fake action publisher)
        self.label_pub = rospy.Publisher('/VBPR/label', String, queue_size=10)

        # List of tracked actions
        self.actions = ["walking", "drinking water", "eating meal", "brushing teeth", "brushing hair", "dropping", "picking up", "throwing", "siting down", "standing up",
                        "clapping", "reading", "writing", "tearing up paper", "putting on jacket", "taking off jacket", "putting on a shoe", 
                        "taking off a shoe", "putting on glasses", "taking off glasses", "putting on a hat/cap", "taking off a hat/cap", "cheering up",
                        "hand waving", "kicking something", "reaching into pocket", "hopping", "jumping up", "phone call", "playing with phone/tablet",
                        "typing on a keyboard", "pointing to something", "taking a selfie", "checking time (from watch)", "rubbing two hands", 
                        "noding head/bow", "shaking head", "wiping face", "saluting", "putting palms together", "crossing hands in front", "sneezing/coughing", 
                        "staggering", "falling down", "headache", "chest pain", "back pain", "neck pain", "nausea/vomiting", "punching/slapping"
                        "kicking", "pushing", "patting on back", "pointing finger", "hugging", "giving object", "touching pocket", "shaking hands",
                        "walking towards", "walking apart"]
        
    def talker(self):
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            index = randint(0,(len(self.actions)-1))
            label = self.actions[index]
            self.label_pub.publish(label)
            rate.sleep()

if __name__ == '__main__': 
    try: 
        rospy.init_node('Action', anonymous=True) 
        processor = Action()
        processor.talker() 

    except rospy.ROSInterruptException: 
        print('Action node failed!') 
        pass