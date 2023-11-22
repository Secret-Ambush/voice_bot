#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from geometry_msgs.msg import Twist
import re
import pygame

motion_command = Twist()
motion_publisher = None  
text_publisher = None  

def playsound(file_path):
    pygame.init()
    pygame.mixer.init()
    try:
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    except pygame.error as e:
        print("Cannot load or play the file: "+ file_path)
    finally:
        pygame.mixer.music.stop()
        pygame.mixer.quit()
        pygame.quit()


def speech_to_text_callback(event):
	global text_publisher  # Declare text_publisher as global
	if flag:
		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio.mp3")
	else:
		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio5.mp3")
	flag = False
	r = sr.Recognizer()
	r.grammar = text_grammar
	
	try:
		with sr.Microphone() as source:
			print("Listening now: ")
			audio = r.listen(source, timeout=3)
			print("Stopped Listening")
			playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio2.mp3")
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
		if selected_text == '' and alternative_list:
			selected_text = alternative_list[0].get('transcript', '')
	
		print("Selected Text:", selected_text)
	
		text_publisher.publish(selected_text)  # Publishing the recognized text

	
	except sr.UnknownValueError:
		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio3.mp3")
		rospy.logwarn("Could not recognize speech")
		stop_robot()

	except sr.RequestError as e:
		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio3.mp3")
		print("Could not request results: "+ str(e))	
		stop_robot()

	except Exception as e:
		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio3.mp3")
		rospy.logerr("Speech recognition error: "+ str(e))
		stop_robot()
	

def process_voice_command(text_msg):
	global motion_command, motion_publisher
	
	text = text_msg.data
	digit_match = re.search(r'\b([1-9]|10|1[1-9]|20|30)\b', text)
	
	
	if digit_match:
		distance_to_travel = int(digit_match.group(0))

	else:
		distance_to_travel = 0

	print("Linear Value: ", distance_to_travel)
	linear_velocity = 0.1
	
	
	if "left" in text:
		rospy.loginfo("Command: Left")
		angular_velocity = 0.5
		desired_angle = 18

		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio4.mp3")

		while desired_angle > 0 and not rospy.is_shutdown():
			motion_command.angular.z = angular_velocity
			motion_publisher.publish(motion_command)
			rospy.sleep(0.1)
			desired_angle -= abs(angular_velocity) 

			motion_command.angular.z = 0.0  
			motion_publisher.publish(motion_command)

		while distance_to_travel > 0 and not rospy.is_shutdown():
			motion_command.linear.x = linear_velocity
			motion_publisher.publish(motion_command)
			rospy.sleep(0.1)
			distance_to_travel -= linear_velocity

		motion_command.linear.x = 0.0  
		motion_publisher.publish(motion_command)
		speech_to_text_callback()
	
	elif "right" in text:
		rospy.loginfo("Command: Right")
		angular_velocity = -0.5
		desired_angle = 18

		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio4.mp3")

		while desired_angle > 0 and not rospy.is_shutdown():
			motion_command.angular.z = angular_velocity
			motion_publisher.publish(motion_command)
			rospy.sleep(0.1)
			desired_angle -= abs(angular_velocity) 

			motion_command.angular.z = 0.0  # Stop the angular motion
			motion_publisher.publish(motion_command)


		while distance_to_travel > 0 and not rospy.is_shutdown():
			motion_command.linear.x = linear_velocity
			motion_publisher.publish(motion_command)
			rospy.sleep(0.1)
			distance_to_travel -= linear_velocity

		motion_command.linear.x = 0.0
		motion_publisher.publish(motion_command)
		speech_to_text_callback()

	
	elif "straight" in text:
		rospy.loginfo("Command: Straight") 
		motion_command.angular.z = 0.0

		playsound("catkin_ws/src/voice_controlled_bot/src/assets/generated_audio4.mp3")

		while distance_to_travel > 0 and not rospy.is_shutdown():
				motion_command.linear.x = linear_velocity
				motion_publisher.publish(motion_command)
				rospy.sleep(0.1)
				distance_to_travel -= linear_velocity
		
		motion_command.linear.x = 0.0
		motion_publisher.publish(motion_command)
		speech_to_text_callback()
	
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
	motion_publisher.publish(motion_command)
	speech_to_text_callback()

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
	flag = True
	speech_to_text_callback()
	
	text_subscriber = rospy.Subscriber('/recognized_text',String, process_voice_command)
	motion_publisher = rospy.Publisher( '/cmd_vel_mux/input/switch', Twist, queue_size=1)  # Publishing movement commands
	
	rospy.spin()
