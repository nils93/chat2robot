## chat2robot
# Project: LLM-Based Robot Control Center

This project involves the development of a chatbot that serves as a control hub for a robot application. 
Instead of using coordinates, it allows users to command the robot to specific destinations using natural language.

# Getting Started
## Installation
Step-by-step instructions to set up your environment.

## 1. Clone the Repository
 ```bash
   git clone https://github.com/nils93/chat2robot.git
   cd chat2robot
 ```
## 2. Docker set up
**Set a new environment variable:**
```bash
   cd docker
   export GOOGLE_API_KEY="ABCDFEGH12345678"
   source ~/.bashrc
```
**Container build:**
   ```bash
   docker compose build
   ```
**Container start:**
   *with GPU:*
   ```bash
   GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach --gpus all
   ```
   *without GPU:*
   ```bash
   GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach
   ```
**Auf Container zugreifen:**
   ```bash
   docker exec -it ros2_turtlebot3 bash
   ```
## 3. Build the ROS2 workspace
```bash
colcon build
source install/setup.bash
```
Alternativ:
```bash
colcon build --cmake-clean-cache
```
## 4. Launch the ROS2 simulation 
This will take few moments and throw errors while loading, you can ignor them
```bash
ros2 launch turtlebot3_full_bringup full_bringup.launch.py
```
## 5. Start the Chatbot
In a new terminal:
```bash
docker exec -it ros2_turtlebot3 bash
```
Change folder (TODO: change the path in chatbot.py for Angabe.tex from relative to absolute so we don't need to change folder)
```bash
source install/setup.bash
cd src/turtlebot3_full_bringup/scripts
python3 -m streamlit run app.py --server.address 0.0.0.0 --server.port 8501
```
Choose your Model.
Wait...

```bash
In Terminal: Login with yout email-adress

Open Browser and open http://0.0.0.0:8501
```
Tell the AI where you want you robot to drive!

---

## Dokumentation und Erkenntnisse

### Auswirkung unterschiedlicher Sprache (Deutsch vs. Englisch)

- Das Systemprompt ist auf Deutsch formuliert und die Interaktion ist primär deutschsprachig, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/chatbot.py](ros2_ws/src/turtlebot3_full_bringup/scripts/chatbot.py#L67-L76). Die Angabe.tex und wird via RAG eingebunden.
- Das gewählte Embedding-Modell ist `sentence-transformers/all-MiniLM-L6-v2`, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py](ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py#L28). Dieses Modell ist primär für englisch ausgelegt. Anders sprachige Anfragen funktionieren, können aber von einem multilingualen Modell wie `paraphrase-multilingual-MiniLM-L12-v2` profitieren.
- Die Chunk-Trennung nutzt Zeilenumbruch und Punkt als Separatoren, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py](ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py#L13-L19). Dadurch ist die Segmentierung sowohl für Deutsch als auch Englisch robust.
- Empfehlung: Bei gemischten Sprachen ein multilingualer Embedder und ggf. Übersetzungspipeline einsetzen.

### Auswirkung unterschiedlicher Chunking-Hyperparameter bzw. Chunking-Ansätze

- Implementierte Parameter: `chunk_size=400`, `chunk_overlap=50`, `separators=["\n", "."]`, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py](ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py#L13-L19). Der Vectorstore wird mit FAISS aufgebaut, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py](ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py#L31-L34), und die Abfrage holt die Top-6 ähnlichen Chunks, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py](ros2_ws/src/turtlebot3_full_bringup/scripts/RAG.py#L41).
- Trade-offs:
   - Größere `chunk_size` erhöht Recall (mehr Kontext pro Chunk), kann aber irrelevante Passagen mitziehen und die Präzision senken.
   - Kleinere `chunk_size` verbessert Präzision, riskiert aber fehlenden Kontext und "Halluzinationen".
   - `chunk_overlap` stabilisiert Querverweise und Satzgrenzen, zu viel Overlap erhöht Redundanz und Speicherbedarf.
   - `k` (Anzahl der ähnlichen Chunks) balanciert Kontextbreite vs. Rauschen; hier `k=6`, je nach Prompt-Komplexität sind `k=3–8` sinnvoll.
- Alternative Ansätze:
   - Tokenbasierte Splitter (z. B. nach Token-Länge statt Zeichen), um LLM-Kontextfenster besser auszunutzen.
   - Struktur-/Format-basierte Splitter (Markdown-/Header-Splitter) für dokumentengetreue Segmente.
   - Semantisches Chunking (Clustering/Sliding Window mit Ähnlichkeitsgrenzen), um inhaltlich ähnlicher Chunks zu bilden.
- Empfehlung: Für technische Anweisungen und Tabellen oft kleinere Chunks (200–400) mit moderatem Overlap (30–80). Für Fließtexte ggf. größere Chunks (600–1200) und geringeres Overlap.

### Empfehlung für industrielle Nutzung (Chatbot zur Steuerung eines Industrieroboters)

- Vorteile:
   - Natürliche Sprachschnittstelle reduziert Bedienhürden und Schulungsaufwand.
   - Flexibel bei Zieldefinition und Rückfragen; gute Integration mit Tool-Calls (ROS-Tools), siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/ROS_tools.py](ros2_ws/src/turtlebot3_full_bringup/scripts/ROS_tools.py).
   - RAG ermöglicht kontextbezogene Aktionen und erklärt Entscheidungen.
- Nachteile/Risiken:
   - Halluzinationen bei LLMs; Fehlinterpretationen sind möglich.
   - Latenz, Verfügbarkeit und API-Abhängigkeiten; Datenschutz bei Cloud-LLMs.
   - Nachvollziehbarkeit komplex; Maschinenrichtlinie, ISO/IEC erfordern harte Sicherheitsmechanismen.
- Empfehlung:
   - Chatbot als Assistenz: Der LLM schlägt Ziele vor, die Ausführung erfolgt über geprüfte ROS-Tools mit Grenzwerten, Interlocks und Zustandsprüfungen.
   - Kommandosprache und Bestätigung: Zwei-Schritt-Freigaben für Bewegungsbefehle, klare Formate (x, y, theta) und Validierungen gegen Karten.
   - Monitoring & Logging: Vollständige Protokollierung von Prompts, Tool-Calls und Statuscodes.
   - Safety by Design: Geofence, Kollisionsvermeidung, Not-Halt, und Testszenarien für Fehlbedienungen.

Hinweis: Die aktuelle Implementierung unterstützt Mistral und Google Gemini zur Laufzeitwahl des LLMs, siehe [ros2_ws/src/turtlebot3_full_bringup/scripts/chatbot.py](ros2_ws/src/turtlebot3_full_bringup/scripts/chatbot.py#L14-L43). Keine starken Unterschiede zwischen den LLMs, Gemini performt etwas besser hat aber entscheidend weniger Quota.
