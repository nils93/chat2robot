# chat2robot

## Getting Started
### 1. Load installer for conda 25.7.0
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-py310_25.7.0-0-Linux-x86_64.sh
```

### 2. Start the installer
```bash
bash Miniconda3-py310_25.7.0-0-Linux-x86_64.sh
```

### 3. Activate Shell-Integration
```bash
source ~/miniconda3/etc/profile.d/conda.sh
echo "source ~/miniconda3/etc/profile.d/conda.sh" >> ~/.bashrc
source ~/.bashrc
```

### 4. Check installation
```bash
conda --version
```

### 5. Create Conda environment 

```bash
conda env create -f conda/environment.yml
```
Paste your google api key
```bash
conda env config vars set GOOGLE_API_KEY="ABCDEFG1234567" -n chat2robot
```
Activate your conda environment and check whether you correctly added the key to the environment variable GOOGLE_API_KEY
```bash
conda activate chat2robot
echo $GOOGLE_API_KEY
```
In case you changed anything in the environment.yaml, you can update your existing conda environment
```bash
conda env update -f environment.yml --prune
```