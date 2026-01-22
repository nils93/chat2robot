from langchain_text_splitters import RecursiveCharacterTextSplitter #Für das Chunking
from langchain_huggingface import HuggingFaceEmbeddings #Für Embeddings
from langchain_community.vectorstores import FAISS #Für Speichern der Vektoren


class RAG_Functions:
    
    def text_input(self):
        with open("Angabe.tex", "r", encoding="utf-8") as f:
          self.imported_text = f.read()     #einlesen des .tex files
    
    def chunking(self):
        #1. Chunker definieren
        splitter = RecursiveCharacterTextSplitter(
            chunk_size=400,     # wie groß ein Chunk maximal sein darf
            chunk_overlap=50,   # wie viel sich zwei Chunks überschneiden
            separators=["\n", "."]  # wo vorzugsweise getrennt werden soll
        )
        
        #2. Chunking ausführen
        self.chunks = splitter.split_text(self.imported_text)
        
        print("-> 1.Chunking durchgefuehrt")


    def Embeddings(self): #Quelle für folgende Zeilen: ChatGPT
        # Verwende ein leichtes deutsches Modell
        self.embeddings = HuggingFaceEmbeddings(model_name="sentence-transformers/all-MiniLM-L6-v2")
        print("-> 2. Embeddings durchgefuehrt")

    def VectorStorage(self):        
        # Vektordatenbank 
        self.VectorDatenbank = FAISS.from_texts(self.chunks, embedding=self.embeddings) #chunks aus chunking FUnktion in FAISS Vektordatenbank laden"""
        print("-> 3. VectorStorage durchgefuehrt")

    def query(self, user_prompt):
        # 1️ Benutzertext in Embedding umwandeln
        user_vector = self.embeddings.embed_query(user_prompt)

        # 2 Ähnliche Chunks aus FAISS-Datenbank suchen
        self.similar_vectors = self.VectorDatenbank.similarity_search_by_vector(user_vector, k=6)      #k=anzahl der naheliegensten Vektoren (höchste Kosinus-Similarity zum User-Prompt   )
        
        print("-> 4. Vergleich UserPromt mit VectorStorage durchgefuehrt")
    
    def LLM_OutputAsListToStr(self, output):
        if isinstance(output, list):
            return output[0]["text"]
        return output
    
