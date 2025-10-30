# ROS2 TurtleBot3 – More Notes from Felix

## Notes

- **Jupyter Notebook:**  
  Zelle ausführen mit `Shift + Enter`

- **Google AI Studio (Gemini):**  
  [https://aistudio.google.com/app/api-keys](https://aistudio.google.com/app/api-keys)

---

## LangChain

- **Beschreibung:** Werkzeugkasten, um Chatbots zu bauen (beinhaltet Chains, Memory, Tools, RAG)
- **LangChain Website:**  
  Account anlegen und API-Key erzeugen:  
  [https://docs.langchain.com/langsmith/create-account-api-key](https://docs.langchain.com/langsmith/create-account-api-key)
- **Tutorial:**  
  [https://python.langchain.com/docs/tutorials/chatbot/](https://python.langchain.com/docs/tutorials/chatbot/)

---

## Grundlagen

- **Kernel:**  
  Schnittstelle zwischen Hardware und Anwendersoftware  
  → verwaltet Prozessor, Arbeitsspeicher, Ein- und Ausgabegeräte

- **LLM und AI:**  
  Ein *Large Language Model (LLM)* ist eine spezielle Untervariante der *Artificial Intelligence (AI)*,  
  die auf Sprache optimiert ist.  
  → Jedes LLM ist eine AI, aber nicht jede AI ist ein LLM.

- **Prompt:**  
  Eingabeaufforderung oder Anweisung an die KI

- **API (Application Programming Interface):**  
  Schnittstelle zwischen User, Software und dem LLM

- **Graph:**  
  Datenstruktur aus Knoten und Verbindungen  
  → Beispiel: *User Input → LLM → Tool (z. B. Suche) → LLM → Antwort*

---

## RAG – Retrieval-Augmented Generation

### Ziel
Einbindung externer, spezifischer Datenquellen in das LLM-Modell, um präzisere Antworten zu erzeugen.

### Schritte
1. **Chunking:** Text in kleinere Abschnitte zerschneiden  
2. **Word Embedding:** Text in Vektoren umwandeln  
3. **Vektorstore:** Lokale Datenbank für Vektoren mit Metadaten

### Grundlagen
- **Embedding:** Das Sprachmodell „embeded“ Text → Vektor  
- **Ähnlichkeit:** Vektoren mit ähnlichem Inhalt liegen im Raum nah beieinander  
  → gemessen mit der *Cosine Similarity*

---

## Warum RAG notwendig ist

Ein LLM arbeitet grundsätzlich nur mit allgemeinem, oberflächlichem Wissen.  
RAG bringt **kontextspezifische oder private Datenquellen** in das Modell ein,  
damit die Antworten relevanter und domänenspezifischer werden.

**Ablauf:**
1. User gibt Prompt ein  
2. RAG sucht in privaten Quellen nach passenden Informationen  
3. Diese Daten werden dem LLM als Kontext übergeben  
4. LLM generiert daraufhin die endgültige Antwort
