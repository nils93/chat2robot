#!/bin/bash
set -e  # stop on first real error

echo "[Projektsetup] Starte Installation für Ubuntu 22.04..."

# 1. Docker aktivieren
echo "Docker aktivieren..."
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"

# 2. Optionaler GPU-Support
if [[ "$1" == "--gpu" ]]; then
  echo "GPU-Modus aktiviert – prüfe containerd und Toolkit..."

  if dpkg -l | grep -q "^ii  containerd "; then
    echo "Dein System verwendet 'containerd' (durch docker.io)."
    echo "NVIDIA Toolkit benötigt 'containerd.io', das kollidiert."
    echo "GPU-Unterstützung wird daher übersprungen."
  elif dpkg -l | grep -q "^ii  nvidia-container-toolkit "; then
    echo "NVIDIA Container Toolkit bereits installiert – wird übersprungen."
  else
    echo "NVIDIA Toolkit wird installiert..."

    distribution=$(. /etc/os-release; echo $ID$VERSION_ID)

    if [[ ! -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg ]]; then
      curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    else
      echo "NVIDIA-Keyring bereits vorhanden – wird übersprungen."
    fi

    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

    sudo apt update
    sudo apt install -y nvidia-container-toolkit
    echo "NVIDIA Container Toolkit installiert."
    sudo systemctl restart docker
  fi
else
  echo "Kein GPU-Modus aktiviert – NVIDIA-Toolkit wird übersprungen."
fi

# 3. GOOGLE_API_KEY abfragen
if [[ -z "$GOOGLE_API_KEY" ]]; then
  read -rp "Bitte gib deinen GOOGLE_API_KEY ein: " key
  export GOOGLE_API_KEY="$key"
fi

# 4. Docker Compose Build
echo "Baue Container-Image mit docker compose build..."
docker compose build

# 5. Container starten
echo "Starte Container mit docker compose..."
GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach
echo "[Projektsetup] Installation abgeschlossen. Viel Erfolg!"

