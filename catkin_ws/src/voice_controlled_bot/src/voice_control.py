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
		
		alternative_list = text.get('alternative', [])
		
		# Iterating to find text with numeric digits
		selected_text = ""
		for item in alternative_list:
			transcript = item.get('transcript', '')
			if any(char.isdigit() for char in transcript):
				selected_text = transcript
				break
		
		# If no text with numeric digits found, select the first one
		if selected_text is None and alternative_list:
			selected_text = alternative_list[0].get('transcript', '')
		
		print("Selected Text:", selected_text)
		
		text_publisher.publish(selected_text)  # Publishing the recognized text
	
	except sr.UnknownValueError:
	rospy.logwarn("Could not recognize speech")
	
	except Exception as e:
	rospy.logerr("Speech recognition error: ", e)
	
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
	"1": 1, "2": 2, "3": 3, "4": 4, "5": 5,
	"6": 6, "7": 7, "8": 8, "9": 9, "10": 10,
	"11": 11, "12": 12, "13": 13, "14": 14, "15": 15,
	"16": 16, "17": 17, "18": 18, "19": 19, "20": 20,
	"21": 21, "22": 22, "23": 23, "24": 24, "25": 25,
	"26": 26, "27": 27, "28": 28, "29": 29, "30": 30
	}
	
	'''
	
	digit = (int)digit_match
	if digit:
		linear_value = float(digit) / 10.0
		rospy.loginfo("Recognized digit: %s", digit)
	
	else:
		linear_value = 0.0
	
	
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
