import numpy as np

from langchain.text_splitter import RecursiveCharacterTextSplitter #Für das Chunking
from langchain.embeddings import HuggingFaceEmbeddings #Für Embeddings
from langchain.vectorstores import FAISS #Für Speichern der Vektoren

class RAG_Functions:
    def __init__(self):
        pass
    
    def text_input(self):
        with open("Angabe.tex", "r", encoding="utf-8") as f:
          self.imported_text = f.read()     #einlesen des .tex files

          #print(imported_text[:500])

    
    def chunking(self):
        #1. Chunker definieren
        splitter = RecursiveCharacterTextSplitter(
            chunk_size=200,     # wie groß ein Chunk maximal sein darf
            chunk_overlap=20,   # wie viel sich zwei Chunks überschneiden
            separators=["\\item","\n", ".", " "]  # wo vorzugsweise getrennt werden soll
        )
        
        #2. Chunking ausführen
        self.chunks = splitter.split_text(self.imported_text)
        
        #3. Ausgabe
        #for i, c in enumerate(chunks):
        #    print(f"Chunk {i+1}:\n{c}\n---")
        #return chunks
        print("-> 1.Chunking durchgefuehrt")


    def Embeddings(self): #Quelle für folgende Zeilen: ChatGPT
        # Verwende ein leichtes deutsches Modell
        self.embeddings = HuggingFaceEmbeddings(model_name="sentence-transformers/all-MiniLM-L6-v2")
        print("-> 2. Embeddings durchgefuehrt")

    def VectorStorage(self):        
        # Vektordatenbank erstellen-> Nochmal selbst machen, hat Wöber extra in Angabe geschrieben
        self.VectorDatenbank = FAISS.from_texts(self.chunks, embedding=self.embeddings) #chunks aus chunking FUnktion in FAISS Vektordatenbank laden"""
        print("-> 3. VectorStorage durchgefuehrt")

    def query(self, user_prompt):
        # 1️ Benutzertext in Embedding umwandeln
        user_vector = self.embeddings.embed_query(user_prompt)

        # 2 Ähnliche Chunks aus FAISS-Datenbank suchen
        self.similar_vectors = self.VectorDatenbank.similarity_search_by_vector(user_vector, k=5)      #k=anzahl der naheliegensten Vektoren (höchste Kosinus-Similarity zum User-Prompt   )

        # 3 Ausgabe oder Weitergabe ans LLM
        for i, doc in enumerate(self.similar_vectors):
            print(f"Treffer {i+1}: {doc.page_content[:200]}...")
        
        print("-> 4. Vergleich UserPromt mit VectorStorage durchgefuehrt")
