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
			audio = r.listen(source, timeout=2)
			print("Stopped Listening")
			text = r.recognize_google(audio, show_all=True)
			print(text)
		
		alternative_list = text.get('alternative', [])
		
		# Iterating to find text with numeric digits
		selected_text = ""
		for item in alternative_list:
		    transcript = item.get('transcript', '')
		    if any(char.isdigit() for char in transcript):
		        selected_text = transcript
		        break
		
		# If no text with numeric digits found, select the first one
		if not selected_text and alternative_list:
		    selected_text = alternative_list[0].get('transcript', '')
		
		print("Selected Text:", selected_text)
		
		text_publisher.publish(selected_text)  # Publishing the recognized text
	
	except sr.UnknownValueError:
		rospy.logwarn("Could not recognize speech")
		stop_robot()
	
	except Exception as e:
		rospy.logerr(f"Speech recognition error: {str(e)}")
		stop_robot()
	
	# rospy.sleep(2)  

def process_voice_command(text_msg):
	global motion_command, motion_publisher
	
	text = text_msg.data
	digit_match = re.search(r'\b([1-9]|10|1[1-9]|20|30)\b', text)
	
	#digit mapping approach (tedious)
	'''
	if digit_match:
	digit_text = digit_match.group(0)
	
	# Map the extracted text to numeric values
	digit_mapping = {
	    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
	    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
	    "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15,
	    "sixteen": 16, "seventeen": 17, "eighteen": 18, "nineteen": 19, "twenty": 20,
	    "twenty-one": 21, "twenty-two": 22, "twenty-three": 23, "twenty-four": 24, "twenty-five": 25,
	    "twenty-six": 26, "twenty-seven": 27, "twenty-eight": 28, "twenty-nine": 29, "thirty": 30
	}
 
	'''
	
	if digit_match:
		digit = int(digit_match.group(0))
		linear_value = float(digit) / 10.0
		rospy.loginfo("Recognized digit: ", digit)
	else:
		linear_value = 0.0

	print("Linear Value: ", linear_value)
	
	
	if "left" in text:
		rospy.loginfo("Command: Left")
		motion_command.angular.z = 0.2
		motion_command.linear.x = linear_value
		
		# Publishing movement commands
		motion_publisher.publish(motion_command)
	
	elif "right" in text:
		rospy.loginfo("Command: Right")
		motion_command.angular.z = -0.2
		motion_command.linear.x = linear_value
		
		# Publishing movement commands
		motion_publisher.publish(motion_command)
	
	elif "straight" in text:
		rospy.loginfo("Command: Straight") 
		motion_command.angular.z = 0.0
		motion_command.linear.x = linear_value
		
		# Publishing movement commands
		motion_publisher.publish(motion_command)
	
	elif "stop" in text:
		rospy.loginfo("Command: Stop") 
		stop_robot()
	
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
	# e.g., "Move left by ten units", "Move right by five units", "Turn left", "Turn right", "Stop", "Move straight"
	direction = "left" | "right" | "straight"
	distance = /([1-9]|10|1[1-9]|20|30)/  # Matches numbers from 1 to 30
	units = "units"
	command = ("move" direction "by" distance units | "turn" direction | "stop" | "move" direction )
	"""
	
	rospy.init_node('voice_commands2', anonymous=True)
	text_publisher = rospy.Publisher(
	'/recognized_text', String, queue_size=1)  # Publishing text
	
	# Creating a timer 
	timer = rospy.Timer(rospy.Duration(2), speech_to_text_callback)
	
	text_subscriber = rospy.Subscriber('/recognized_text',String, process_voice_command)
	motion_publisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=1)  # Publishing movement commands
	
	rospy.spin()
