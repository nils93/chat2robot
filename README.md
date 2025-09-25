# chat2robot

## Getting started
```shell
export GOOGLE_API_KEY="ABCDEFG1234567"
echo $GOOGLE_API_KEY
```

## Conda 
```shell
conda env create -f environment.yml
conda env config vars set GOOGLE_API_KEY="ABCDEFG1234567" -n chat2robot
conda activate chat2robot
conda env update -f environment.yml --prune
```