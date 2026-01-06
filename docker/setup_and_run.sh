#!/bin/bash
set -e

echo "[Projektsetup] Starte Installation für Ubuntu 22.04..."

# 1. Docker aktivieren
echo "[Setup] Docker aktivieren..."
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"

# 2. GPU erkennen
USE_GPU=false
if command -v nvidia-smi &> /dev/null && nvidia-smi -L &> /dev/null; then
  echo "[Info] NVIDIA GPU erkannt."

  if ! dpkg -l | grep -q "^ii  nvidia-container-toolkit "; then
    echo "[Setup] Installiere NVIDIA Container Toolkit..."
    distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | \
      sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
    sudo apt update
    sudo apt install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
  fi

  USE_GPU=true
else
  echo "[Info] Keine GPU erkannt – CPU-Modus."
fi

# 3. API_KEYs abfragen

echo ""
echo "[API Keys: Mindestens einen Key angeben (Enter = Überspringen)"

#GOOGLE_API_KEY abfragen
if [[ -z "$GOOGLE_API_KEY" ]]; then
  read -rsp "Bitte gib deinen GOOGLE_API_KEY ein: " google_key
  echo
  export GOOGLE_API_KEY="$google_key"
fi

if [[ -z "$MISTRAL_API_KEY" ]]; then
  read -rsp "Bitte gib deinen MISTRAL_API_KEY ein: " mistral_key
  echo
  export MISTRAL_API_KEY="$mistral_key"
fi

# 4. Build
echo "[Docker] Baue Image..."
docker compose build

# 5. Start
echo "[Docker] Starte Container..."
if [ "$USE_GPU" = true ]; then
  docker compose --profile gpu up -d
else
  docker compose up -d
fi

echo "[Fertig] Setup abgeschlossen."
echo "Container betreten mit:"
echo "  docker exec -it ros2_turtlebot3 bash"