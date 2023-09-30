# Voice Controlled Robot with ROS 🤖

This documentation provides an overview of a Python script for controlling a robot using voice commands with updated grammar for a specific syntax of commands through ROS (Robot Operating System). The script utilizes the `speech_recognition` library to recognize voice input and ROS for robot control.

## Prerequisites

Before running the script, make sure you have the following installed:

- ROS (Robot Operating System)
- `speech_recognition` Python library
- Appropriate ROS packages (e.g., `std_msgs`, `geometry_msgs`)

## Script Overview

The Python script (`voice_control.py`) listens to voice commands from a microphone and translates them into robot motion commands. Here's a breakdown of the script:

### Importing Dependencies

```python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
from geometry_msgs.msg import Twist
```

- The script starts by importing necessary libraries and ROS message types.

### Initializing Global Variables

```python
motion_command = Twist()
motion_publisher = None  # Initialize the motion_publisher
text_publisher = None  # Initialize the text_publisher
```

- `motion_command`: Stores the robot's motion commands.
- `motion_publisher`: Initializes the motion command publisher.
- `text_publisher`: Initializes the text publisher.

### `speech_to_text_callback` Function

```python
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
```

- `speech_to_text_callback` listens to voice input from the microphone and uses the `speech_recognition` library to recognize the speech.
- Recognized text is published to the ROS topic `/recognized_text`.

### `process_voice_command` Function

```python
def process_voice_command(text):
    global motion_command, motion_publisher

    if "left" in text:
        # Handle left turn command
        # ...

    elif "right" in text:
        # Handle right turn command
        # ...

    elif "straight" in text:
        # Handle moving straight command
        # ...

    else:
        # Handle unrecognized command
        # ...
```

- `process_voice_command` processes the recognized text and generates appropriate motion commands based on the recognized voice commands.

### `stop_robot` Function

```python
def stop_robot():
    global motion_command, motion_publisher
    # Stop the robot by setting linear and angular velocities to zero
    motion_command.linear.x = 0.0
    motion_command.angular.z = 0.0
    motion_publisher.publish(motion_command)  # Publish the stop command
```

- `stop_robot` is used to stop the robot by setting its linear and angular velocities to zero.

### Main Script

```python
if __name__ == '__main__':
    rospy.init_node('voice_commands', anonymous=True)
    text_publisher = rospy.Publisher('/recognized_text', String, queue_size=10)  # Publish recognized text
    # ...
```

- The main script initializes the ROS node, sets up publishers and subscribers, and starts the ROS spin loop.

## Usage

1. Ensure that ROS is properly set up on your system.

2. Install the required Python libraries, including `speech_recognition`.

3. Run the script using `rosrun`.

4. The script will listen to your voice commands and control the robot accordingly.

## Note

- Customize the motion commands within the `process_voice_command` function to match your robot's control interface.

- Adjust the timers and sleep durations as needed for your application.

- Ensure that the appropriate ROS topics (`/recognized_text` and `/cmd_vel`) are available and match the topics used in the script.

That's it! You now have a voice-controlled robot using ROS and Python.


Please note that you should customize the handling of specific voice commands within the `process_voice_command` function to match the behaviour of your robot. Additionally, make sure that the ROS topics used in the script (`/recognized_text` and `/cmd_vel`) are correctly configured for your robot's setup.
