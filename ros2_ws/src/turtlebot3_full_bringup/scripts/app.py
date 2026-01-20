import time
import streamlit as st
from Chatbot import chat_step
from ROS_tools import ROS_get_navigation_status

st.set_page_config(
    page_title="Robot Chatbot",
    page_icon="",
    layout="centered"
)

st.title("Control Interface")
st.caption("MRK Project")

COMMENT_START_TURN1 = "Ich fahre los — wenn /cmd_vel heute gute Laune hat."
COMMENT_REACHED_TURN1 = "Position erreicht… also zumindest im TF-Tree. 😄 … spaẞ!!"
COMMENT_TURN3 = "Plan ist solide. Realität ist optional."

if "messages" not in st.session_state:
    st.session_state.messages = []

if "user_turns" not in st.session_state:
    st.session_state.user_turns = 0

for msg in st.session_state.messages:
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])

if prompt := st.chat_input("Wohin soll der Roboter fahren?"):
    st.session_state.user_turns += 1

    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    with st.chat_message("assistant"):
        placeholder = st.empty()

        response = chat_step(prompt)
        response_ui = response

        if st.session_state.user_turns == 1:
            response_ui += "\n\n💬 _" + COMMENT_START_TURN1 + "_"

        if st.session_state.user_turns == 3:
            response_ui += "\n\n💬 _" + COMMENT_TURN3 + "_"

        placeholder.markdown(response_ui)

        if st.session_state.user_turns == 1:
            timeout_s = 90
            start = time.time()

            while True:
                try:
                    status_code = int(ROS_get_navigation_status())
                except Exception:
                    status_code = 0

                if status_code == 4:  # SUCCEEDED
                    response_ui += "\n\n✅ _" + COMMENT_REACHED_TURN1 + "_"
                    placeholder.markdown(response_ui)
                    break

                if time.time() - start > timeout_s:
                    break

                time.sleep(0.2)

    st.session_state.messages.append({"role": "assistant", "content": response_ui})
