import numpy as np
import sounddevice as sd
from whisper import load_model
import time

# Load the Whisper model
model = load_model("base")

def recognize_speech(audio_data):
    # Converts the audio data to text using Whisper
    result = model.transcribe(audio_data)
    return result['text']

def callback(indata, frames, time, status):
    global flag, silence_start_time, max_silence_duration
    if status:
        print(status)

    if np.abs(indata).mean() < 0.01:  # Adjust the threshold based on your microphone sensitivity
        if silence_start_time is None:
            silence_start_time = time.time()
        elif time.time() - silence_start_time > max_silence_duration:
            flag = False  # Stop listening if max silence duration exceeded
    else:
        silence_start_time = None  # Reset silence timer

    if flag:
        text = recognize_speech(np.float32(indata))
        if text:
            print("Recognized text:", text)


def speech_to_text_callback(max_silence=3):
    global flag, silence_start_time, max_silence_duration
    flag = True
    silence_start_time = None
    max_silence_duration = max_silence  # Maximum silence duration in seconds

    try:
        # Stream audio and transcribe in real time
        with sd.InputStream(callback=callback):
            while flag:
                sd.sleep(1000)  # Keep the stream open until flag is False
            print("Stopped listening")
    except Exception as e:
        print("An error occurred:", e)

# Initialize and use the above functions as necessary.
print("Listening")
speech_to_text_callback()