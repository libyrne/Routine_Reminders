#!/usr/bin/env python3 

# ROS imports 
import rospy 
from std_msgs.msg import String


# Python imports
from gtts import gTTS
from io import BytesIO
import pygame
from datetime import datetime

# Custom imports
from parse_cal import main

class Remind():
    def __init__(self):
        # Initialize pygame for sound 
        pygame.init()
        pygame.mixer.init()

        # Subscriber for labels (TO DO: make a temporary fake action publisher)
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

        # Initialize array for calendar events
        date_list = []
        time_list = []
        action_list = []

        # Get schedule from Google Calendar from parse_cal.py script (returns date, start time, and event name)
        events = main()
        for event in events:
            # Takes in date and start time of event
            timing = event['start'].get('dateTime', event['start'].get('time'))
            # Get just the date and append to date list
            date = timing[0:10]
            date_list.append(date)
            print(type(date))
            # Get just the time of dat and append to time list
            time = timing[11:19]
            time_list.append(time)
            print("time")
            print(type(time))
            # Append the event title to the action list
            action_list.append(event['summary'])

        # Get number of events in calendar
        num_events = len(action_list)
        
        # Initialize lists for tasks done and tasks reminded
        completed = [False for i in range(num_events)]
        reminded = [False for i in range(num_events)]

        # Make an array of all the events' details
        self.master_list = [date_list, time_list, action_list, completed, reminded]

        # Add person object for more personal responses (like name?)
        


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
            # Get the separate lists from list (may end up ditching master list)
            day = self.master_list[0][index]
            time = self.master_list[1][index]
            action = self.master_list[2][index] # Will need to match to one of the actions in the actions list
            done = self.master_list[3][index]
            reminded = self.master_list[4][index]

            # Check if event day is today (may change depending on how many events we take i.e. week vs. daily sched)
            # Check if the action has been completed
            # Check if a reminder has already been sent
            if(str(now.date()) == day and not done and not reminded):
                # Check if the action has been done within the last hour 
                if(time[:2] == str(hour) or time[:2] == str(hour-1)):


                    # NEEDS EDITING PAST HERE
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

    def speak(self, action):
        mp3_fp = BytesIO()
        # tts = gTTS('γεια', lang='el', tld='com')
        tts_en = gTTS('Good morning Mary, have you ' +  action + 'yet today?', lang='en', tld='com')
        tts_en.write_to_fp(mp3_fp)
        sound = mp3_fp
        sound.seek(0)
        pygame.mixer.music.load(sound, "mp3")
        pygame.mixer.music.play()
        # while pygame.mixer.music.get_busy():
        #     pygame.time.Clock().tick(10)
        # return mp3_fp


if __name__ == '__main__': 
    try: 
        rospy.init_node('Remind', anonymous=True) 
        processor = Remind()
        processor.speak("taken a walk")  
        rospy.spin() 

    except rospy.ROSInterruptException: 
        print('Reminder node failed!') 
        pass 