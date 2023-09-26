#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from geometry_msgs.msg import Twist
import re

motion_command = Twist()
motion_publisher = None  
text_publisher = None  

def speech_to_text_callback(event):
    global text_publisher  # Declare text_publisher as global

    r = sr.Recognizer()
    r.grammar = text_grammar

    try:
        with sr.Microphone() as source:
	    print("Listening now: ")
            audio = r.listen(source, timeout = 5)
            print("Stopped Listening")
            text = r.recognize_google(audio)
            print("Recognized text: %s", text)
            text_publisher.publish(text)  # Publishing the recognized text

    except sr.UnknownValueError:
        rospy.logwarn("Could not recognize speech")

    except sr.RequestError as e:
        rospy.logerr("Speech recognition error: %s", e)
        
    # rospy.sleep(2)  

def process_voice_command(text_msg):
    global motion_command, motion_publisher

    text = text_msg.data
    digit_match = re.search(r'\b(1|2|3|4|5|6|7|8|9|10)\b', text)
    digit_text = digit_match.group(0)

        # Map the extracted text to numeric values
    digit_mapping = {
            "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
            "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
            "1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6, "7": 7, "8": 8, "9": 9, "10": 10
        }

    if digit_text in digit_mapping:
        digit = digit_mapping[digit_text]
        rospy.loginfo("Recognized digit: %s", digit)
            

    if "left" in text:
        rospy.loginfo("Command: Left")
        motion_command.angular.z = 0.2
	if digit_text in digit_mapping:  
		motion_command.linear.x = float(digit) / 10.0
	else:
		motion_command.linear.x = 0.0
		
        # Publishing movement commands
        motion_publisher.publish(motion_command)

    elif "right" in text:
        rospy.loginfo("Command: Right")
        motion_command.angular.z = -0.2
	if digit_text in digit_mapping:  
		motion_command.linear.x = float(digit) / 10.0
	else:
		motion_command.linear.x = 0.0
        # Publishing movement commands
        motion_publisher.publish(motion_command)

    elif "straight" in text:
        rospy.loginfo("Command: Straight") 
        motion_command.angular.z = 0.0
	if digit_text in digit_mapping:  
		motion_command.linear.x = float(digit) / 10.0
	else:
		motion_command.linear.x = 0.0
        # Publishing movement commands
        motion_publisher.publish(motion_command)

    else:
        rospy.loginfo("Unrecognized command")
        stop_robot()  # Stop the robot for unrecognized commands

def stop_robot():
    global motion_command, motion_publisher
    motion_command.linear.x = 0.0
    motion_command.angular.z = 0.0
    motion_publisher.publish(motion_command)  # Publishing stop command

if __name__ == '__main__':
    text_grammar = """
	    # Command format: "Move <direction> by <distance> units"
	    # e.g., "Move left by ten units", "Move right by five units"
	    direction = "left" | "right"
	    distance = "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9" | "10"
	    units = "units"
	    command = "move" (direction "by" distance units | direction distance units)
	"""
    rospy.init_node('voice_commands2', anonymous=True)
    text_publisher = rospy.Publisher(
        '/recognized_text', String, queue_size=1)  # Publishing text
    
    # Creating a timer 
    timer = rospy.Timer(rospy.Duration(2), speech_to_text_callback)
    
    text_subscriber = rospy.Subscriber('/recognized_text',String, process_voice_command)
    motion_publisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=1)  # Publishing movement commands

    rospy.spin()
