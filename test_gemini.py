# test_gemini.py
import os
import google.generativeai as genai

# API Key laden
api_key = os.environ.get("GOOGLE_API_KEY")
if not api_key:
    raise ValueError("GOOGLE_API_KEY ist nicht gesetzt!")

# Konfiguration
genai.configure(api_key=api_key)

# Modell initialisieren
model = genai.GenerativeModel("gemini-1.5-flash")

# Prompt testen
response = model.generate_content("Hallo, kannst du mir in einem Satz erklären, was ROS2 ist?")

print("Antwort vom Modell:")
print(response.text)



