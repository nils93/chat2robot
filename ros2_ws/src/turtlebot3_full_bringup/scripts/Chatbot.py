import os
import getpass

from langchain.chat_models import init_chat_model  #hier wird langchain importiert
from langchain_classic.agents import create_tool_calling_agent, AgentExecutor      #neue Version des "langchain-agents import create_tool_calling_agent in zeile 20"
from langchain_core.messages import HumanMessage #datentyp Input der anwendenden Person->UserPrompt
from langchain_core.messages import AIMessage #datentyp Antwort der AI
from langchain_core.messages import SystemMessage #DatenTyp des Systemsprompts
from langchain_core.prompts import ChatPromptTemplate #notwendig, da dem Agent keine normale Liste mehr übergeben werden kann, sonder es muss ein "ChatPromptTemplate" sein

#eigene Module
from Chatbot_header_RAG import RAG_Functions
from Chatbot_header_tools import rechner, ROS_cmd_vel

# Abfragen und einlesen des LangChai API-Keys
os.environ["LANGCHAIN_TRACING"] = "false"  #wenn false: Anwendung läuft lokal, nichts wird an Langsmith gesendet

# Abfragen und einlesen des GooglE API-Keys 
if not os.environ.get("GOOGLE_API_KEY"):
    os.environ["GOOGLE_API_KEY"] = getpass.getpass("Enter API key for Google Gemini: ")

# Initialisieren des LLM
LLM = init_chat_model("gemini-2.5-flash", model_provider="google_genai",temperature=0.7,top_p=0.8)

# Liste für Historie anlegen
chat_history=[]


# Systemprompt
system_prompt= "Du bist des Gehirn eines Mobilen Roboters. Du erkennst verschiendene Sprachen (unter anderem auch den Österreichischen Dialekt) und du sollst aus einem Tooling Pool das Richtige Tool auswählen, um den Roboter an eine in x und y angegeben Position zu fahren. Du sollst so lange beim User nachfragen, bis dir ein Position gegeben wird, die du anfahren kannst."


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
tools=[rechner, ROS_cmd_vel]

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
