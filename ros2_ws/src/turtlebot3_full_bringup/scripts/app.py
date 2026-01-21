"""Streamlit-based Chatbot Interface for Robot Control.

This script creates a web-based chat interface that allows users to send
commands to the robot through natural language. The chatbot processes
user input and returns navigation commands to control the TurtleBot3.
"""

# Streamlit library for building web UI
import streamlit as st
# Import chatbot function that processes user input
from chatbot import chat_main_fct

# Configure the Streamlit page settings
st.set_page_config(
    page_title="Robot Chatbot",                     # Browser tab title
    page_icon="",                                   # Page icon (emoji)
    layout="centered"                               # Centered layout for better UX
)

# Display main title and subtitle
st.title("Control Interface")                       # Main heading
st.caption("MRK Project")                           # Project subtitle

# Initialize session state for chat message history
# Session state persists data across Streamlit reruns
if "messages" not in st.session_state:
    st.session_state.messages = []                  # Store conversation history

# Display all previous messages in chat history
for msg in st.session_state.messages:
    with st.chat_message(msg["role"]):              # Create chat bubble with user/assistant role
        st.markdown(msg["content"])                 # Display message content

# Accept user input through chat input widget
if prompt := st.chat_input("Wohin soll der Roboter fahren?"):  
    # Append user message to session state
    st.session_state.messages.append(
        {"role": "user", "content": prompt}         # Store user message
    )
    # Display user message in chat
    with st.chat_message("user"):
        st.markdown(prompt)

    # Get chatbot response and display it
    with st.chat_message("assistant"):
        response = chat_main_fct(prompt)            # Process user input through chatbot
        st.markdown(response)                       # Display bot response

    # Store bot response in session state for persistence
    st.session_state.messages.append(
        {"role": "assistant", "content": response}  # Store assistant response
    )

