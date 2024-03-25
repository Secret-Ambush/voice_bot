from openai import OpenAI
import os
api_key = os.getenv("API_KEY")

client = OpenAI(api_key)

def interpret_command_with_chatgpt(command, openai_api_key):
    try:
        prompt_text = f"Interpret this command into a standardized navigation format: '{command}'. Include direction and distance (default to 0cm if unspecified)."

        response = client.completions.create(engine="text-davinci-003",
        prompt=prompt_text,
        temperature=0.5, 
        max_tokens=50,
        top_p=1,
        frequency_penalty=0,
        presence_penalty=0)

        # Process and return the response text
        return response.choices[0].text.strip()
    except Exception as e:
        print(f"An error occurred: {e}")
        return "Error processing command"

sample_command = "Move strait ahead by ten centimeters"
print(interpret_command_with_chatgpt(sample_command, api_key))


'''
POSSIBLE PROMPT -

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
