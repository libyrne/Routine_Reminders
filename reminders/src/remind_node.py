#!/usr/bin/env python3 

# ROS imports 
import rospy 
from std_msgs.msg import String 

# Python imports 
from datetime import datetime


class Remind(): 

    def __init__(self): 
        # Subscribe to label topic
        label_sub = rospy.Subscriber('/VBPR/label', String, self.label_cb, queue_size=10)

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

        # Schedule of tracked person's action and corresponding time
        self.schedule = [[action1, action2, action3, action4], [time1, time2, time3, time4]]


    def label_cb(self, msg_data):
        # Chop off preceeding A to get index of corresponding action
        index = msg_data.replace("A", "")
        index = int(index)
        action = self.actions[index] # Get action

        # Check if action corresponds to needed action
        self.check_action(action)

    def check_action(self, action):
        now = datetime.now()
        day = now.weekday() # Monday(0) - Sunday(6)
        hour = now.hour # In military time
        minute = now.minute

        




if __name__ == '__main__': 
    try: 
        rospy.init_node('Remind', anonymous=True) 
        processor = Remind() 
        port_sub = rospy.Subscriber('sonar_ain0', Float64, processor.port_cb) # tilt left 
        starboard_sub = rospy.Subscriber('sonar_ain1', Float64, processor.starboard_cb) # tilt right  
        trim_fwd_sub = rospy.Subscriber('sonar_ain2', Float64, processor.trim_fwd_cb) # tilt forward 
        trim_bwd_sub = rospy.Subscriber('sonar_ain3', Float64, processor.trim_bwd_cb) # tilt backward 
        rospy.spin() 

    except rospy.ROSInterruptException: 
        print('List and trim node failed!') 
        pass 