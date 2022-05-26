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
            # Get just the time of dat and append to time list
            time = timing[11:19]
            time_list.append(time)
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
        # Subscriber for labels (TO DO: make a temporary fake action publisher)
        label_sub = rospy.Subscriber('/VBPR/label', String, self.label_cb, queue_size=10)
        


    def label_cb(self, msg_data):
        # Chop off preceeding A to get index of corresponding action
        # index = msg_data.replace("A", "")
        # index = int(index)
        # action = self.actions[index] # Get action
        action = msg_data.data

        # Check if action corresponds to needed action
        self.check_action(action)


    def check_action(self, action):
        now = datetime.now()
        day = now.weekday() # Monday(0) - Sunday(6)
        hour = now.hour # In military time
        minute = now.minute
        length = len(self.master_list[1])
        print(action)

        for index in range(len(self.master_list[1])):
            # Get the separate lists from list (may end up ditching master list)
            day = self.master_list[0][index]
            time = self.master_list[1][index]
            rtn_action = self.master_list[2][index] # Will need to match to one of the actions in the actions list
            done = self.master_list[3][index]
            reminded = self.master_list[4][index]

            # Check if event day is today (may change depending on how many events we take i.e. week vs. daily sched)
            # Check if the action has been completed
            # Check if a reminder has already been sent
            if(str(now.date()) == day and not done and not reminded):
                # Check if within an hour of a scheduled task
                if((time[:2] == str(hour) or time[:2] == str(hour-1)) and not done):
                    # Check if the scheduled task has been done
                    if(rtn_action == action):
                        self.master_list[3][index] = True
                        print("Good job!")
                    # If the task has not been completed, and it's time to remind
                    elif(time[:2] == str(hour) and time[3:5] <= str(minute) and not reminded):
                        # Send a reminder to the person to complete the task
                        self.speak(rtn_action)
                        self.master_list[4][index] = True
        print(self.master_list[3])


                    # # NEEDS EDITING PAST HERE
                    # if(action == rtn_action):
                    #     self.schedule[2][index] = True # Mark action as done
                    # # Check if you are within five minutes of the reminder time
                    # elif(time_min >= (min - 5) or time_min <= (min +5)):
                    #     #Add reminder here!!!
                    #     text = "Time to ..."
                    #     self.engine.say(text)
                    #     self.engine.runAndWait()
                    #     # Set reminded bool to done
                    #     self.schedule[3][index] = True

    def speak(self, action):
        mp3_fp = BytesIO()
        # tts_en = gTTS('Καλημέρα, Mark. Ωραία γυαλιά!', lang='el', tld='com')
        tts_en = gTTS('Hello, I noticed you have not been ' +  action, lang='en', tld='com')
        tts_en.write_to_fp(mp3_fp)
        sound = mp3_fp
        sound.seek(0)
        pygame.mixer.music.load(sound, "mp3")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        # return mp3_fp


if __name__ == '__main__': 
    try: 
        rospy.init_node('Remind', anonymous=True) 
        processor = Remind()
        rospy.spin() 

    except rospy.ROSInterruptException: 
        print('Reminder node failed!') 
        pass 