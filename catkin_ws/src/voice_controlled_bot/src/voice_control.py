#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from geometry_msgs.msg import Twist

motion_command = Twist()
motion_publisher = None  # Initialize the motion_publisher
text_publisher = None  # Initialize the text_publisher

def speech_to_text_callback(event):
    global text_publisher  # Declare text_publisher as global

    r = sr.Recognizer()

    try:
        with sr.Microphone() as source:
            audio = r.listen(source)
            text = r.recognize_google(audio)
            rospy.loginfo("Recognized text: %s", text)
            text_publisher.publish(text)  # Publish the recognized text

    except sr.UnknownValueError:
        rospy.logwarn("Could not recognize speech")

    except sr.RequestError as e:
        rospy.logerr("Speech recognition error: %s", e)
        
    # rospy.sleep(2)  # Sleep for 2 seconds

def process_voice_command(text_msg):
    global motion_command, motion_publisher
    text = text_msg.data

    if "left" in text:
        rospy.loginfo("Command: Left")
        # Generate motion command for turning left
        motion_command.linear.x = 0.0
        motion_command.angular.z = 0.2  # Adjust the angular velocity as needed
        # Publish the motion command
        motion_publisher.publish(motion_command)

    elif "right" in text:
        rospy.loginfo("Command: Right")
        # Generate motion command for turning right
        motion_command.linear.x = 0.0
        motion_command.angular.z = -0.2  # Adjust the angular velocity as needed
        # Publish the motion command
        motion_publisher.publish(motion_command)

    elif "straight" in text:
        rospy.loginfo("Command: Straight")
        # Generate motion command for moving straight
        motion_command.linear.x = 0.2  # Adjust the linear velocity as needed
        motion_command.angular.z = 0.0
        # Publish the motion command
        motion_publisher.publish(motion_command)

    else:
        rospy.loginfo("Unrecognized command")
        stop_robot()  # Stop the robot for unrecognized commands

def stop_robot():
    global motion_command, motion_publisher
    motion_command.linear.x = 0.0
    motion_command.angular.z = 0.0
    motion_publisher.publish(motion_command)  # Publish the stop command

if __name__ == '__main__':
    rospy.init_node('voice_commands', anonymous=True)
    text_publisher = rospy.Publisher( '/recognized_text', String, queue_size=10)  # Publish recognized text
    
    # Create a timer that calls speech_to_text_callback every 2 seconds
    timer = rospy.Timer(rospy.Duration(2), speech_to_text_callback)
    
    text_subscriber = rospy.Subscriber('/recognized_text',String, process_voice_command)
    motion_publisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=10)  # Publish robot motion commands
    rospy.spin()
