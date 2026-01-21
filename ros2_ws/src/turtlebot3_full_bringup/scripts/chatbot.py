import os
import getpass
import time

from langchain.chat_models import init_chat_model  #hier wird langchain importiert
from langchain_classic.agents import create_tool_calling_agent, AgentExecutor      #neue Version des "langchain-agents import create_tool_calling_agent in zeile 20"
from langchain_core.messages import HumanMessage #datentyp Input der anwendenden Person->UserPrompt
from langchain_core.messages import AIMessage #datentyp Antwort der AI
from langchain_core.messages import SystemMessage #DatenTyp des Systemsprompts
from langchain_core.prompts import ChatPromptTemplate #notwendig, da dem Agent keine normale Liste mehr übergeben werden kann, sonder es muss ein "ChatPromptTemplate" sein
from langchain_mistralai import ChatMistralAI

#eigene Module
from RAG import RAG_Functions
import ROS_tools

#ROS
import rclpy
import threading


def select_llm():

    print("====  Choose LLM  ====")
    print("  1) MISTRAL (mistral-small-latest)")
    print("  2) Google Gemini (gemini-2.5-flash)")
    choice_LLM = input("Choose: [1/2]: ").strip()

    if choice_LLM == "1":
        print("MISTRAL ausgewählt")

        if not os.environ.get("MISTRAL_API_KEY"):
            os.environ["MISTRAL_API_KEY"] = getpass.getpass("Enter API key for MISTRAL: ")

        return ChatMistralAI(
            model="mistral-small-latest",
            temperature=0.5,
            top_p=0.8
        )
    
    elif choice_LLM == "2":
        print("GOOGLE GEMINI ausgewählt")

        if not os.environ.get("GOOGLE_API_KEY"):
            os.environ["GOOGLE_API_KEY"] = getpass.getpass("Enter API key for Google Gemini: ")

        return init_chat_model("gemini-2.5-flash",
            model_provider="google_genai",
            temperature=0.5,
            top_p=0.8
        )
    
    else:
        print("Invalid input. Accepting only [1] or [2]")


# Abfragen und einlesen des LangChai API-Keys
os.environ["LANGCHAIN_TRACING"] = "false"  #wenn false: Anwendung läuft lokal, nichts wird an Langsmith gesendet

# Initialisieren des LLM
LLM = select_llm()
# Liste für Historie anlegen
chat_history=[]


# Systemprompt
system_prompt= "Du steuerst einen mobilen Roboter per ROS2.\n" \
    "Deine Wissensbasis ist der Inhalt der per RAG hinterlegten Dokumente.\n\n" \
    "VORGEHEN:\n" \
    "1) Identifiziere aus dem User-Prompt ein oder mehrere Ziele. Stelle Rückfragen, bis du mindestens ein Ziel eindeutig identifizieren kannst.\n" \
    "2) Extrahiere zu jedem Ziel die x, y und theta Angaben aus der Wissensbasis, oder aus der Chathistorie.\n" \
    "3) Falls Du nach dem aktuellen Navigationsstatus gefragt wird, melde eine kurze Info zurück. Rufe dazu das geeignete tool auf. " \
    "4) Eine Navigation ist erst beendet, wenn den Navigationsstatus SUCCEDED, CANCELED oder ABORTED ist. Rufe dazu regelmäßig das geeignete tool auf, bis der entsprechende Navigationsstatus von ROS rueckgemeldet wird. Frage die status solange ab, bis eine entsprechende Rueckmeldung vorliegt. Bis dahin nehmen keine neuen anfragen an.\n" \
    "5) WICHTIG: Rufe für jede Pose die du anfahren sollst das Tool ROS_send_goal(x, y, theta) auf, wenn du die Koordinaten in der Wissensbasis findest. Bei zwei Zielen: Rufe das Tool 'ROS_send_tow_goals'\n\n" \
    "Nur Koordinaten aus der Wissensbasis verwenden."

#aufbauen des vollständigen Prompts mit allen Inhalten
full_prompt = ChatPromptTemplate.from_messages([
    ("system", system_prompt),
    ("placeholder", "{chat_history_LLM}"),  # Hier kommt die History rein
    ("human", "{input}"),
    ("placeholder", "{agent_scratchpad}"),
])

#RAG initialisieren
RAG_1=RAG_Functions()   #Konstruktor
RAG_1.text_input() #.tex einlesen
RAG_1.chunking() #ausführen der Chunking Funktion
RAG_1.Embeddings() #embebddings erstellen
RAG_1.VectorStorage() #VectorStore aufbauen ->Funktion nochmal selbst machen, hat Wöber extra in Angabe geschrieben


#Tools einbinden
tools=[ROS_tools.ROS_send_goal,ROS_tools.ROS_send_two_goals, ROS_tools.ROS_get_navigation_status]

# Agent anlegen (gehirn/grundfunktionen)
LLM_agent= create_tool_calling_agent(LLM, tools, full_prompt) 

#executor anlegen (prozessmanager)
LLM_agent_executor = AgentExecutor(
    agent=LLM_agent,
    tools=tools,
    verbose=True,   # True: Zeigt den Denkprozess des Agenten
    handle_parsing_errors=True
    )

rclpy.init() #initialisieren ROS2 
ROS_tools.ros_object = ROS_tools.ROSGoalPublisher()
threading.Thread(
    target=rclpy.spin,
    args=(ROS_tools.ros_object,),
    daemon=True
).start()
time.sleep(1)


if ROS_tools.ros_object is None:
    print("FEHLER: ROS2 noch nicht initialisiert")
else:
    print("\n+ + + ROS2 initialisiert + + +\n")

def chat_main_fct(user_input_via_app: str) -> str:
    global chat_history

    # User Anfrage in history
    chat_history.append(HumanMessage(content=user_input_via_app))

    # RAG
    RAG_1.query(user_input_via_app)
    similar_content = [doc.page_content for doc in RAG_1.similar_vectors]

    LLM_input = (
        f"User Anfrage: {user_input_via_app}\n\n"
        f"Wissensbasis-chunks:\n{similar_content}"
    )

    #Agenten mit "create_tool_calling_agent" global angelegt (Zeile 98)
    #Agent_Executor mit "AgentExecutor(...)" angelget (Zeile 100)

    # Agent_executor ausführen 
    LLM_Answer = LLM_agent_executor.invoke({
        "input": LLM_input,
        "chat_history_LLM": chat_history
    })

    chat_history.append(AIMessage(content=LLM_Answer["output"]))

    return RAG_1.LLM_OutputAsListToStr(LLM_Answer["output"])
