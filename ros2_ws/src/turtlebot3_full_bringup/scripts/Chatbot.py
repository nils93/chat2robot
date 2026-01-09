import os
import getpass

from langchain.chat_models import init_chat_model  #hier wird langchain importiert
from langchain_classic.agents import create_tool_calling_agent, AgentExecutor      #neue Version des "langchain-agents import create_tool_calling_agent in zeile 20"
from langchain_core.messages import HumanMessage #datentyp Input der anwendenden Person->UserPrompt
from langchain_core.messages import AIMessage #datentyp Antwort der AI
from langchain_core.messages import SystemMessage #DatenTyp des Systemsprompts
from langchain_core.prompts import ChatPromptTemplate #notwendig, da dem Agent keine normale Liste mehr übergeben werden kann, sonder es muss ein "ChatPromptTemplate" sein
from langchain_mistralai import ChatMistralAI

#eigene Module
from Chatbot_header_RAG import RAG_Functions
from ROS_tools import ROS_send_goal, rechner, ROS_get_navigation_status


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
    "2) Extrahiere zu jedem Ziel die x, y und theta Angaben aus der Wissensbasis.\n" \
    "3) WICHTIG: Rufe für jede Pose die du anfahren sollst das Tool ROS_send_goal(x, y, theta) auf, wenn du die Koordinaten in der Wissensbasis findest. Bei mehreren Zielen: warte auf Bestätigung, dann nächstes Ziel.\n\n" \
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
tools=[rechner, ROS_send_goal, ROS_get_navigation_status]

# Agent anlegen (gehirn/grundfunktionen)
LLM_agent= create_tool_calling_agent(LLM, tools, full_prompt) 

#executor anlegen (prozessmanager)
LLM_agent_executor = AgentExecutor(
    agent=LLM_agent,
    tools=tools,
    verbose=True,   # True: Zeigt den Denkprozess des Agenten
    handle_parsing_errors=True
    )


def main():

    while True:
        #Einlesen von Informationen-> UserPrompt
        InputMessage=input("Was möchtest du der AI sagen?")

        #Abbrunchbedingung
        if InputMessage=="exit":
            break

        # Ähnliche Vektoren vie RAG finden
        RAG_1.query(InputMessage) 
        
        #zu history ergänzen 
        similar_content = [doc.page_content for doc in RAG_1.similar_vectors]
        chat_history.append(HumanMessage(content="RAG-Content: "+str(similar_content)))
        
        #LLM-agent Antwort erstellen
        LLM_answer=LLM_agent_executor.invoke({"input": InputMessage, "chat_history_LLM": chat_history})
        
        #Abspeichern der Antwort und anhängen an die Historie
        chat_history.append(AIMessage(content=LLM_answer["output"]))

        output_str = RAG_1.LLM_OutputAsListToStr(LLM_answer["output"])
        print(f"Chatbot: {output_str}")
        
        print(f"\nAnzahl Nachrichten in Historie: {len(chat_history)}")

if __name__ == "__main__":
    main()
