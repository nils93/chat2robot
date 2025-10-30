#!/bin/bash
set -e

if [[ "$1" == "--gpu" ]]; then
  echo "[Hinweis] GPU-Unterstützung ist auf macOS (Apple Silicon) nicht verfügbar."
  echo "[Info] Das Skript läuft automatisch im CPU-Modus weiter."
fi

echo "[Projektsetup] Starte Installation für macOS..."

# 1. Docker Desktop prüfen
if ! command -v docker &>/dev/null; then
  echo "Docker Desktop ist nicht installiert. Das Skript beendet sich."
  exit 1
fi

if ! docker info &>/dev/null; then
  echo "Docker läuft nicht. Bitte Docker Desktop starten und erneut ausführen."
  exit 1
fi
echo "Docker läuft."

# 2. Plattform automatisch setzen (ARM oder Intel)
ARCH=$(uname -m)
if [[ "$ARCH" == "arm64" || "$ARCH" == "aarch64" ]]; then
  PLATFORM="linux/arm64/v8"
else
  PLATFORM="linux/amd64"
fi
echo "[Info] Verwende Plattform: $PLATFORM"

TMP_COMPOSE="docker-compose.temp.yaml"

# 3. Temporäre Compose-Datei erzeugen:
#    - Plattform einfügen
#    - vollständige GPU-/deploy-Blöcke entfernen
awk -v platform="$PLATFORM" '
  BEGIN {skip=0}
  # Wenn deploy-Block beginnt, überspringe bis zum nächsten Service oder Top-Level
  /^ {4}deploy:/ {skip=1; next}
  skip==1 {
    if ($0 ~ /^ {2}[^[:space:]]/ || $0 ~ /^[^[:space:]]/) {skip=0} else {next}
  }
  # Entferne einzelne GPU-Zeilen
  /capabilities: \[gpu\]/ || /device_requests:/ || /runtime: nvidia/ || /nvidia/ {next}
  # Plattform-Zeile einfügen
  /^services:/ {print; next}
  /^  ros2_turtlebot3_gpu:/ {
    print; getline;
    if ($0 ~ /^    platform:/) {print "    platform: " platform; next}
    else {print "    platform: " platform; print $0; next}
  }
  {print}
' docker-compose.yaml > "$TMP_COMPOSE"

# 4. GOOGLE_API_KEY abfragen
if [[ -z "$GOOGLE_API_KEY" ]]; then
  read -rp "Bitte gib deinen GOOGLE_API_KEY ein: " key
  export GOOGLE_API_KEY="$key"
fi

# 5. X11 Variablen setzen (optional)
if [[ -z "$DISPLAY" ]]; then
  export DISPLAY=host.docker.internal:0
fi
if [[ -z "$XAUTHORITY" ]]; then
  export XAUTHORITY=/tmp/.Xauthority
fi

echo "[Info] GPU-Unterstützung wird auf macOS nicht unterstützt. Starte im CPU-Modus."

# 6. Docker Compose Build
echo "[Docker] Baue Container-Image..."
docker compose -f "$TMP_COMPOSE" build

# 7. Container starten
echo "[Docker] Starte Container..."
GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose -f "$TMP_COMPOSE" up --detach

# 8. Aufräumen
rm -f "$TMP_COMPOSE"

echo "[Projektsetup] Installation abgeschlossen. Viel Erfolg!"
echo "Starte nun ein neues Terminal und führe den Befehl 'docker exec -it ros2_turtlebot3_gpu bash' aus, um in den Container zu gelangen."