#!/bin/bash
set -e  # stop on first real error

echo "[Projektsetup] Starte Installation für Ubuntu 22.04..."

# 1. Docker aktivieren
echo "Docker aktivieren..."
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"

# 2. GPU automatisch erkennen
if command -v nvidia-smi &> /dev/null && nvidia-smi -L &> /dev/null; then
  echo "[Info] NVIDIA GPU erkannt – Toolkit wird geprüft/installiert."

  if ! dpkg -l | grep -q "^ii  nvidia-container-toolkit "; then
    echo "Installiere NVIDIA Container Toolkit..."
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
  else
    echo "[Info] NVIDIA Container Toolkit bereits installiert."
  fi

  GPU_FLAG="--gpus all"
else
  echo "[Info] Keine GPU erkannt – starte im CPU-Modus."
  GPU_FLAG=""
fi

# 3. GOOGLE_API_KEY abfragen (falls nicht gesetzt)
if [[ -z "$GOOGLE_API_KEY" ]]; then
  read -rp "Bitte gib deinen GOOGLE_API_KEY ein: " key
  export GOOGLE_API_KEY="$key"
fi

# 4. Docker Compose Build
echo "[Docker] Baue Container-Image..."
docker compose build

# 5. Container starten (mit oder ohne GPU)
echo "[Docker] Starte Container..."
GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach $GPU_FLAG

echo "[Projektsetup] Installation abgeschlossen. Viel Erfolg!"
echo "Starte nun ein neues Terminal und führe den Befehl 'docker exec -it ros2_turtlebot3_gpu bash' aus, um in den Container zu gelangen."