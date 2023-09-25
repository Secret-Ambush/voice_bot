#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from geometry_msgs.msg import Twist

motion_command = Twist()
motion_publisher = None  
text_publisher = None  

def speech_to_text_callback(event):
    global text_publisher  # Declare text_publisher as global

    r = sr.Recognizer()

    try:
        with sr.Microphone() as source:
	    print("Listening now: ")
            audio = r.listen(source, timeout = 1) #better timeout
            print("Stopped Listening")
            text = r.recognize_google(audio, grammar=text_grammar)
            rospy.loginfo("Recognized text: %s", text)
            text_publisher.publish(text)  # Publishing the recognized text

    except sr.UnknownValueError:
        rospy.logwarn("Could not recognize speech")

    except sr.RequestError as e:
        rospy.logerr("Speech recognition error: %s", e)
        
    # rospy.sleep(2)  

def process_voice_command(text_msg):
    global motion_command, motion_publisher

    text = text_msg.data

    if "left" in text:
        rospy.loginfo("Command: Left")
        
        motion_command.linear.x = 0.0
        motion_command.angular.z = 0.2  
        # Publishing movement commands
        motion_publisher.publish(motion_command)

    elif "right" in text:
        rospy.loginfo("Command: Right")
       
        motion_command.angular.z = -0.2  
        motion_command.linear.x = 0.0
        # Publishing movement commands
        motion_publisher.publish(motion_command)

    elif "straight" in text:
        rospy.loginfo("Command: Straight")
        
        motion_command.linear.x = 0.2  
        motion_command.angular.z = 0.0
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
    rospy.init_node('voice_commands2', anonymous=True)
    text_publisher = rospy.Publisher(
        '/recognized_text', String, queue_size=1)  # Publishing text
    
    # Creating a timer 
    timer = rospy.Timer(rospy.Duration(2), speech_to_text_callback)
    
    text_subscriber = rospy.Subscriber('/recognized_text',String, process_voice_command)
    motion_publisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=1)  # Publishing movement commands

    text_grammar = """
    # Command format: "Move <direction> by <distance> units"
    # e.g., "Move left by ten units", "Move right by five units"
    direction = "left" | "right"
    distance = "one" | "two" | "three" | "four" | "five" | "six" | "seven" | "eight" | "nine" | "ten" | "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9" | "10"
    units = "units"
    command = "move" (direction "by" distance units | direction distance units)
	"""

    rospy.spin()
