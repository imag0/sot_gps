@echo off

:: Run main.py in a new command prompt window
start cmd /k "python main.py"

:: Wait for a moment to ensure main.py has finished
timeout /t 5 /nobreak

:: Run pipi.py in a new command prompt window
start cmd /k "python pipi.py"

:: Exit the script
exit
