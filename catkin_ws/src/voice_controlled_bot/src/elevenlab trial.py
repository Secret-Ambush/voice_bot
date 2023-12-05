from elevenlabs import voices, generate
import pygame
import io

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

playsound("Hello there!")  # Play the generated audio
