#!/usr/bin/env python
import openai
import speech_recognition as sr
import re
import pygame
from elevenlabs import generate
import io
import os
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("API_KEY")
openai.api_key = api_key


def playsound(text):
    audio = generate(text, voice="Bella")
    pygame.init()
    pygame.mixer.init()
    try:
        sound = pygame.mixer.Sound(io.BytesIO(audio))
        sound.play()
        while pygame.mixer.get_busy():
            pygame.time.Clock().tick(10)
    except pygame.error as e:
        print("Cannot play the audio")
    finally:
        pygame.mixer.quit()
        pygame.quit()
        
def createtext(prompting):
	completion = openai.chat.completions.create(
	model="gpt-3.5-turbo",
	messages=[
		{"role": "system", "content": "You are a bot."},
		{"role": "user", "content": prompting}
		]
	)
	
	response = completion.choices[0].message.content
	print(response)
	playsound(response)

def process_voice_command(text_msg):
    global motion_command, motion_publisher

    text = text_msg
    digit_match = re.search(r'\b([1-9]|10|1[1-9]|20|30)\b', text)

    if digit_match:
        distance_to_travel = int(digit_match.group(0))

    else:
        distance_to_travel = 0

    print("Linear Value: ", distance_to_travel)
    linear_velocity = 0.1
	
    if "left" in text:
        createtext(f"Generate a simple sentence to say that you are moving left {distance_to_travel} units")
        speech_to_text_callback()

    elif "right" in text:
        createtext(f"Generate a simple sentence to say that you are moving right {distance_to_travel} units")
        speech_to_text_callback()

    elif "straight" in text:
        createtext(f"Generate a simple sentence to say that you are moving straight {distance_to_travel} units")
        speech_to_text_callback()

    elif "stop" in text:
        createtext("Generate a simple sentence to say that you stopped")
        print("Command: Stop") 
        stop_robot()

    else:
        print("Unrecognized command")
        stop_robot()  # Stop the robot for unrecognized commands

def speech_to_text_callback():
    global text_publisher  # Declare text_publisher as global
    if flag:
        createtext("Generate a short simple salutation eager for the human to direct you")
    else:
        createtext("Generate one very short informal sentence to show that you are ready to listen")


    r = sr.Recognizer()
    # r.grammar = text_grammar

    try:
        with sr.Microphone() as source:
            print("Listening now: ")
            audio = r.listen(source, timeout=3)
            print("Stopped Listening")
            createtext("Generate a very short informal sentence to say that you got the command")
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
        process_voice_command(selected_text)

    except sr.UnknownValueError:
        createtext("Generate a simple sentence to say that you did not understand")
        print("Could not recognize speech")
        stop_robot()

    except sr.RequestError as e:
        createtext("Generate a simple sentence to say that you did not understand")
        print("Could not request results: "+ str(e))	
        stop_robot()

    except Exception as e:
        createtext("Generate a simple sentence to say that you did not understand")
        print("Speech recognition error: "+ str(e))
        stop_robot()
	
def stop_robot():
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
    flag = False
    speech_to_text_callback()
