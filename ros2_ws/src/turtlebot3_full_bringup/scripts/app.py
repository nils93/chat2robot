import streamlit as st
from Chatbot import chat_main_fct

st.set_page_config(
    page_title="Robot Chatbot",
    page_icon="",
    layout="centered"
)

st.title("Control Interface")
st.caption("MRK Project")

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
    st.session_state.messages.append(
        {"role": "user", "content": prompt}
    )
    with st.chat_message("user"):
        st.markdown(prompt)

    # Bot Antwort
    with st.chat_message("assistant"):
        response = chat_main_fct(prompt)
        st.markdown(response)

    st.session_state.messages.append(
        {"role": "assistant", "content": response}
    )

