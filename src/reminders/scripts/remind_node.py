#!/usr/bin/env python3 

# ROS imports 
import rospy 
from std_msgs.msg import String

# Python imports 
from datetime import datetime
from gtts import gTTS
from io import BytesIO
import pygame


class Remind(): 

    def __init__(self): 
        # Subscribe to label topic
        label_sub = rospy.Subscriber('/VBPR/label', String, self.label_cb, queue_size=10)

        # List of tracked actions
        self.actions = ["walking", "drinking water", "eating meal", "brushing teeth", "brushing hair", "dropping", "picking up", "throwing", "sitting down", "standing up",
                        "clapping", "reading", "writing", "tearing up paper", "putting on jacket", "taking off jacket", "putting on a shoe", 
                        "taking off a shoe", "putting on glasses", "taking off glasses", "putting on a hat/cap", "taking off a hat/cap", "cheering up",
                        "hand waving", "kicking something", "reaching into pocket", "hopping", "jumping up", "phone call", "playing with phone/tablet",
                        "typing on a keyboard", "pointing to something", "taking a selfie", "checking time (from watch)", "rubbing two hands", 
                        "noding head/bow", "shaking head", "wiping face", "saluting", "putting palms together", "crossing hands in front", "sneezing/coughing", 
                        "staggering", "falling down", "headache", "chest pain", "back pain", "neck pain", "nausea/vomiting", "punching/slapping"
                        "kicking", "pushing", "patting on back", "pointing finger", "hugging", "giving object", "touching pocket", "shaking hands",
                        "walking towards", "walking apart"]

        # Schedule of tracked person's action and corresponding time
        # Need to figure out what the schedule look like (in order based on time?)
        done1, done2, done3, done4 = False
        rem1, rem2, rem3, rem4 = False
        self.schedule = [["action1", "action2", "action3", "action4"], ["time1", "time2", "time3", "time4"], [done1, done2, done3, done4], [rem1, rem2, rem3, rem4]]
        self.action_completed = False

        # For the speech
        self.engine = pyttsx3.init()


    def label_cb(self, msg_data):
        # Chop off preceeding A to get index of corresponding action
        index = msg_data.replace("A", "")
        index = int(index)
        action = self.actions[index] # Get action

        # Check if action corresponds to needed action
        self.check_action(action)

    def check_action(self, rtn_action):
        now = datetime.now()
        day = now.weekday() # Monday(0) - Sunday(6)
        hour = now.hour # In military time
        minute = now.minute

        for index in range(len(self.schedule[1])):
            # Need to get in terms of hour and min
            time_hour = self.schedule[1][index]
            time_min = self.schedule[1][index]
            action = self.schedule[0][index] # Will need to match to one of the actions in the actions list
            done = self.schedule[2][index]
            reminded = self.schedule[3][index]

            # Check if the action has been sent
            if(not done):
                # Check if a reminder for the action has been sent
                if(not reminded):
                    # Check if the action has been done within the last hour 
                    if(time_hour == hour or time_hour == (hour-1)):
                        if(action == rtn_action):
                            self.schedule[2][index] = True # Mark action as done
                        # Check if you are within five minutes of the reminder time
                        elif(time_min >= (min - 5) or time_min <= (min +5)):
                            #Add reminder here!!!
                            text = "Time to ..."
                            self.engine.say(text)
                            self.engine.runAndWait()
                            # Set reminded bool to done
                            self.schedule[3][index] = True

    def speak(action, person_obj):
        mp3_fp = BytesIO()
        # tts = gTTS('γεια', lang='el', tld='com')
        tts_en = gTTS('Good morning Mary, have you ' +  action + 'yet today?', lang='en', tld='com')
        tts_en.write_to_fp(mp3_fp)
        return mp3_fp   





if __name__ == '__main__': 
    try: 
        rospy.init_node('Remind', anonymous=True) 
        processor = Remind()  
        rospy.spin() 

    except rospy.ROSInterruptException: 
        print('Reminder node failed!') 
        pass 