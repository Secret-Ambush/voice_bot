from openai import OpenAI

client = OpenAI(api_key = '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')

completion = client.chat.completions.create(
  model="gpt-3.5-turbo",
  messages=[
    {"role": "system", "content": "You are a bot."},
    {"role": "user", "content": "Generate a short simple salutation eager for the human to direct you"}
  ]
)

response = completion.choices[0].message.content
print(response)