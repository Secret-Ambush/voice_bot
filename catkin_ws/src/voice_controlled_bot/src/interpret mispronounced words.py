import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

client = OpenAI(
  api_key=os.getenv("API_KEY"),
)


def interpret_command_with_chatgpt(command):
    try:
        prompt_text = f"""Perform the following operations on the provided command, which is: '{command}'. The command involves direction and distance:
        - Determine the direction as straight, left, right, or stop.
        - Present your response ONLY in the format (direction by distance).
        - If no distance is specified, indicate the distance as 0cm.
        - If the command is 'turn around' or some synonym, your output should be 'turn left 360 degrees'.
        - If the direction is backward, specify the direction as 'straight' with a negative distance.
        - In the case of diagonal movement, the response should be 'turn left 15 degrees and move straight specified units'.
        Please rectify any potential spelling errors or homophone mistakes.
        """

        response = client.completions.create(
            model="gpt-4o",  # Specify the model engine here
            prompt=prompt_text,
            max_tokens=150
        )

        return response.choices[0].text.strip()

    except Exception as e:
        print(f"An error occurred: {e}")
        return "Error processing command"

sample_command = "Move straight ahead by ten centimeters"
print(interpret_command_with_chatgpt(sample_command))




'''
POSSIBLE PROMPT (Needs some modification to include into code)-

Objective: Develop a language model capable of interpreting informal or formal voice commands, potentially containing mispronunciations or spelling errors, intended for navigation. The model should accurately discern the intended direction (e.g., 'go straight', 'move forward', 'turn right', 'turn left') and distance (when specified) from these commands. The output should be in a standardized format, highlighting both direction and magnitude (e.g., 'straight 10', 'left 20', 'right 5'), to facilitate easy interpretation and subsequent action.

Input Format: The input will be a transcribed text of voice commands. These commands may contain informal language, formal requests, or common mispronunciations/spelling errors that could alter the expected vocabulary (e.g., 'strait' instead of 'straight').

Output Format: Regardless of the input's formality or accuracy, output the interpreted command as a direction followed by a distance (if specified) in centimeters. The output should strictly adhere to this format: 'direction distance'. Use 'straight', 'left', and 'right' for direction, and ensure distances are rounded to the nearest whole number.

Vocabulary Consideration: The model should be familiar with a broad range of synonyms and common mispronunciations for directional commands. This includes, but is not limited to:

Straight: 'straight', 'strait', 'forward', 'ahead'
Left: 'left', 'lef', 'lift'
Right: 'right', 'rite', 'wright'
Distances: Recognize numerical values, possibly followed by units like 'cm', 'centimeters', or informal representations ('a bit', 'a few steps').
Special Instructions:

If the command lacks a clear distance, assume the action implies a standard distance of 10cm.
In cases of ambiguous direction due to severe misspelling, request clarification without executing or suggesting a command.
Ensure the model is primed to prioritize the recognition of navigation-related vocabulary, even when faced with uncommon or creatively misspelled words.
Sample Input: "Move strait ahead by ten centimeters"
Expected Output: "straight 10"

'''
