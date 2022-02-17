@echo off

set env_name=GEO5017A1_Group5

echo [*] Starting Installation
echo [*] Creating env: %env_name%...

call conda env create -f conda_env.yml

echo [*] Success
echo [*] Installing pyoctree from source

call conda activate %env_name%

echo [*] Activated env

cd pyoctree
python setup.py install
cd ..

echo [*] Success

echo [*] Installation Complete
echo [*] Run "python main.py" to execute
pause