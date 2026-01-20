import streamlit as st
import random
from Chatbot import chat_step

st.set_page_config(
    page_title="Robot Chatbot",
    page_icon="",
    layout="centered"
)

st.title("Control Interface")
st.caption("MRK Project")

# --- Fun/Serious Toggle ---
st.markdown("### 😐 Oder soll der Roboter heute *lustig* sein?")
fun_mode = st.toggle("🤖 Fun Mode aktivieren", value=False)

if fun_mode:
    st.info("🎭 Fun Mode ist AN — der Roboter darf jetzt frech sein.")
else:
    st.success("🧑‍💼 Serious Mode ist AN — Business only.")

# Session State für UI-Chat
if "messages" not in st.session_state:
    st.session_state.messages = []

# Bisherige Nachrichten anzeigen
for msg in st.session_state.messages:
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])

# User Input
if prompt := st.chat_input("Wohin soll der Roboter fahren?"):
    # User anzeigen
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    # Bot Antwort
    with st.chat_message("assistant"):
        response = chat_step(prompt)
        st.markdown(response)

        # --- Fun Mode Extras (nur wenn aktiv) ---
        if fun_mode:
            ros_jokes = [
                "🤖 ROS sagt: Ziel erreicht. Realität sagt: Wand.",
                "🤖 Ich nutze ROS2. Deshalb dauert alles minimal länger.",
                "🤖 Navigation läuft… hoffentlich.",
                "🤖 Ich bin nicht lost — ich mache nur Mapping.",
                "🤖 TF-Tree stabil… (haha, guter Witz).",
            ]

            # 15% Chance auf einen Joke
            if random.random() < 0.5:
                st.markdown(f"*{random.choice(ros_jokes)}*")

            # Pseudo-Confidence-Bar
            confidence = min(100, len(prompt) * 8)
            st.progress(confidence)
            st.caption(f"Roboter-Confidence: {confidence}%")

    st.session_state.messages.append({"role": "assistant", "content": response})
