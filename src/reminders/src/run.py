#!/usr/bin/env python3 

# ROS imports 
import rospy 
from std_msgs.msg import String


# Python Imports
from gtts import gTTS
from io import BytesIO
import pygame
import time

class Remind():
    def __init__(self):
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