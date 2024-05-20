import openai
from openai import OpenAI
import pygame
from elevenlabs import voices, generate
import io
import os
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("API_KEY")
print(api_key)


def speak(text):
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
        
completion = openai.chat.completions.create(
  model="gpt-3.5-turbo",
  messages=[
    {"role": "system", "content": "You are a bot."},
    {"role": "user", "content": "Generate a simple sentence to say that you are moving left"}
  ]
)

response = completion.choices[0].message.content
print(response)
speak(response)