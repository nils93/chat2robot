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
Change folder (TODO:change the path in Chatbot.py for Angabe.tex from relatv to absolut see we don't need to change folder, also rename chatbot.py)
```bash
cd src/turtlebot3_full_bringup/scripts
python3 Chatbot.py
```
Choose your Model.
Wait...
Tell the AI where you want your robot to drive!



old README:

#Auswirkung unterschiedlicher Sprache (z.B.: Deutsch, Englisch)

#Auswirkung unterschiedlicher Chunking Hyperparameter bzw. Chunking Ansätzen
 
#Ihre Empfehlung f¨ur industrielle Nutzung. W¨are ein Chatbot zur Steuerung eines Industrieroboters aus Ihrer Sicht empfehlenswert? Was w¨aren die Vorteile/Nachteile.

#(optional, empfohlen) Die Auswirkung von unterschiedlichen LLM.



#-> in src-folder: turtlebot_full_bringup is without ROS-interface, turtlebot3_full_bringup (with the 3) is with an ROS-interface &an GUI

