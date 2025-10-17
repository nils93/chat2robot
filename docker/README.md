# ROS2 TurtleBot3 Docker-Setup

## Schnellstart: So startest du den Docker-Container

1. **Wechsle ins Docker-Verzeichnis:**
   ```bash
   cd docker
   ```
2. **Setze eine neue Umgebungsvariable:**
   ```bash
   export GOOGLE_API_KEY="ABCDFEGH12345678"
   source ~/.bashrc
   ```
3. **Führe das bash-Skript aus:**
   ```bash
   ./setup_and_run.sh
   ```
4. **Öffne eine Bash-Shell im laufenden Container:**
   ```bash
   docker exec -it ros2_turtlebot3_gpu bash
   ```
   *(Der Name `ros2_turtlebot3_gpu` steht in der `docker-compose.yaml` unter `container_name`.)*

**Hinweise:**

- **Image bauen:**
   ```bash
   docker compose build
   ```
- **Container starten:**
   ```bash
   docker compose up
   ```
- **Container stoppen:**  
  Drücke `Strg+C` im Terminal mit `docker compose up` oder führe aus:
  ```bash
  docker compose down
  ```
  Danach ggf. erneut bauen mit `docker compose build` (z.B. nach Änderungen an Dockerfile oder docker-compose.yaml).

---


## Wo füge ich zusätzliche ROS/Ubuntu-Packages hinzu?
Dieses Repository enthält ein Docker-Setup für ROS2 (Humble), TurtleBot3, Gazebo, MoveIt und weitere Tools.

Im [Dockerfile](Dockerfile) gibt es einen markierten Bereich, in dem du weitere Ubuntu- oder ROS-Packages installieren kannst.  
**Füge deine Pakete nur zwischen den folgenden Kommentaren hinzu:**

```dockerfile
# === BEGIN: Eigene Ubuntu/ROS-Packages ===
# Hier kannst du weitere Pakete hinzufügen, z.B.:
# sudo apt install -y <dein-paket>
# === END: Eigene Ubuntu/ROS-Packages ===
```

**Beispiel:**
```dockerfile
RUN sudo apt update && sudo apt install -y \
    # ...bestehende Pakete...
    # === BEGIN: Eigene Ubuntu/ROS-Packages ===
    ros-humble-my-extra-pkg \
    # === END: Eigene Ubuntu/ROS-Packages ===
    && sudo apt autoremove -y && sudo apt clean
```

## Wo installiere ich zusätzliche Python-Pakete?

Im [Dockerfile](Dockerfile) gibt es einen markierten Bereich für Python-Pakete.  
**Füge deine Python-Pakete nur zwischen den folgenden Kommentaren hinzu:**

```dockerfile
# === BEGIN: Eigene Python-Pakete ===
# Hier kannst du weitere pip-Installationen hinzufügen, z.B.:
# pip install <dein-python-paket>
# === END: Eigene Python-Pakete ===
```

**Beispiel:**
```dockerfile
RUN pip install --upgrade pip && \
    pip install colcon-clean torch transformers protobuf \
    # === BEGIN: Eigene Python-Pakete ===
    numpy matplotlib \
    # === END: Eigene Python-Pakete ===
```

## Wie kann ich ein Laufwerk oder einen Ordner im Container einbinden (mounten)?

Im [`docker-compose.yaml`](docker-compose.yaml) kannst du mit dem Schlüsselwort `volumes` lokale Ordner oder Laufwerke in den Container einbinden.  
Das Format ist:

```yaml
volumes:
  - <lokaler_pfad>:<container_pfad>[:<optionen>]
```

**Beispiel:**  
Um den lokalen Ordner `../ros2_ws` in den Container unter `/home/ubuntu/ros2_ws` einzubinden, verwende:

```yaml
services:
  ros2:
    # ...weitere Einstellungen...
    volumes:
      - ../ros2_ws:/home/ubuntu/ros2_ws
```

**Weitere Beispiele:**
- Einen Ordner nur lesbar einbinden:
  ```yaml
  - ./daten:/home/ubuntu/daten:ro
  ```
- Ein X11-Socket für GUI-Anwendungen einbinden:
  ```yaml
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
  ```

## Hinweise 
- **Bitte ändere nur die Bereiche zwischen den markierten Kommentaren!**  
  So bleibt das geteilte Repository konsistent und Updates können leichter übernommen werden.
- Wenn du weitere Anpassungen benötigst, erstelle am besten einen eigenen Branch oder Fork.

- Alle unter `volumes:` gelisteten Einträge werden beim Start des Containers automatisch gemountet.  
  Passe die Pfade nach deinen Bedürfnissen an.
---
