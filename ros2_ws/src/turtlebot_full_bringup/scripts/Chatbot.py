import os
import getpass

from Chatbot_header import*

#_____________Initialisieren des LLM______________________________________________________________________
#abfragen und einlesen des LangChai API-Keys
os.environ["LANGCHAIN_TRACING"] = "false"  #wenn false: Anwendung läuft lokal, nichts wird an Langsmith gesendet
#os.environ["LANGCHAIN_TRACING_V2"] = "false"
#os.environ["LANGSMITH_API_KEY"] = getpass.getpass("Enter API Key for LangChain")

#abfragen und einlesen des GooglE API-Keys 
if not os.environ.get("GOOGLE_API_KEY"):
    #os.environ["GOOGLE_API_KEY"] = "Hier API-Key eintragen!!!!!"
    os.environ["GOOGLE_API_KEY"] = getpass.getpass("Enter API key for Google Gemini: ")

#initialisieren des chatModels
from langchain.chat_models import init_chat_model  #hier wird langchain importiert

#jetzt: gemini initialisieren, hyperparameter "temperature" (Kreativität [0;1]) und "top_p" (Filter der Wortwahl [0;1]) setzen
model = init_chat_model("gemini-2.5-flash", model_provider="google_genai",temperature=0.7,top_p=0.8)  

#->Hier das OrchestrierungsFramework einbinden:  ->Koordinieren Ablauf zwischen LLM und Tools, Lässt das LLM-Entscheiden welches Tool es verwenden muss
# from langchain.agents import AgentExecutor, create_tool_calling_agent
# from langchain_google_genai import ChatGoogleGenerativeAI


#_______________Setup__________________________________________________________________________
#LLM direk verwenden, ohne anpassungen
#Übergeben der kompletten Chatbot Historie (kompletter Prompt)

from langchain_core.messages import HumanMessage #datentyp Input der anwendenden Person->UserPrompt
from langchain_core.messages import AIMessage #datentyp Antwort der AI
from langchain_core.messages import SystemMessage #DatenTyp des Systemsprompts

#from langchain_core.messages import BaseDataContentBlock #DatenTyp des Systemsprompts

#Liste für Historie anlegen
chat_history=[]

#Systemprompt definieren:
#system_prompt= "Du bist ein sehr sakastischer Roboter wie Tars aus Interstellar und antwortest immer kurz und schnippisch."
system_prompt= "Du bist ein Bot zum programmieren, speziell für python. Du überdenkst immer alle Lösungen und gibst sie mit einer ganz kurzen Erklärung aus."
#system_prompt= "Du bist des Gehirn eines Mobilen Roboters. Du erkennst verschiendene Sprachen (unter anderem auch den Österreichischen Dialekt) und du sollst aus einem Tooling Pool das Richtige Tool auswählen, um Aktivitäten des Roboters durchzuführen. Du sollst so lange beim User nachfragen, bis dir ein Position gegeben wird, die du anfahren kannst"

#Einlesen des Systemprompts
chat_history.append(SystemMessage(content=system_prompt))



#Instanzieren und einlesen der RAG-Informationen
RAG_1=RAG_Functions()   #hier Konstruktorfunktion aufrufen, daher die Klammer

#Hier: Einbinden des RetrievalAugumentedGeneration-Modells
#1. Dokumente laden (Zuordnen von Festen Positionen (HomeBase, ...) zu Koordinaten)
    #->Siehe oberhalb der Main, da dies nicht in einem Loop gemacht werden muss
RAG_1.text_input()

#2. Chunking (Zerscheiden der Dokumente in Sinnvolle Teile, die dann zugeteilt werden)
    #->WIe macht es Sinn zu Zerteilen? ->Hier: in Sätzen!
RAG_1.chunking() #ausführen der Chunking Funktion

#2. Embeddings erstellen: Text aus UserPrompt und zusätzlichen Daten zu Vektoren
RAG_1.Embeddings()

#3. VectorStore aufbauen ->Funktion nochmal selbst machen, hat Wöber extra in Angabe geschrieben
RAG_1.VectorStorage()

#______________________________________Main_____________________________________


while True:
    #Einlesen von Informationen-> UserPrompt
    InputMessage=input("Was möchtest du der AI sagen?")

    #Abbrunchbedingung
    if InputMessage=="exit":
        break
    
    #Hinzufügren vom UserPrompt in die History
    chat_history.append(HumanMessage(content="Input Anwender: "+ InputMessage))

    #Teil von RAG: Vergleichen von Vektor des UserPromts und Vektoren im VectorStorage
    #4. Schauen, welche Vektoren im Vektorstore ähnlich sind (Ähnliche Richtung und Länge, Bestimmung über cosine similarity)
    RAG_1.query(InputMessage) #Hier UserPrompt oder Historie? Userpromt sinnvoller
    
    #5. Gefundene Textteile werden dem UserPrompt angehängt (In Historie laden)
    #Hinzufügren vom UserPrompt in die History
    #chat_history.append(BaseDataContentBlock(content=RAG_1.similar_vectors))
    similar_content = [doc.page_content for doc in RAG_1.similar_vectors]
    chat_history.append(HumanMessage(content="RAG-Content: "+str(similar_content)))

    #aufrufen des LLMs->model.invoke heißt soviel wie model.aufrufen(...) 
    LLM_answer=model.invoke(chat_history)
    
    #Abspeichern der Antwort und anhängen an die Historie
    chat_history.append(AIMessage(content=LLM_answer.content))
    
    #Ausgabe
    print(f"\nLLM: {LLM_answer.content}")
    print(f"\nAnzahl Nachrichten in Historie: {len(chat_history)}")


    #Tooling ()
    #Was für ein Tool notwendig? Berechnung der Trajektorie? Oder Verarbeiten der Koordinaten und Publishen an Ros?
    #Ausgeben der Koordinaten in einem ROS-Format!
    #->Aufrufen einer Funktion durch das LLM (Mit Stichwort?) und übergeben der Koordinaten  im ROS-Format (ANgabe: kein hartgecodetes Tooling!)
    #->Dann die geometry_msgs publishen
    #from langchain_core.tools import tool und from pydantic import BaseModel, Field einbinden, um eigene Tools zu erstellen


