@echo off
echo Installing dependencies from requirements.txt...
pip install --upgrade pip
pip install -r requirements.txt
if %ERRORLEVEL% neq 0 (
    echo Error occurred while installing dependencies.
    pause
    exit /b %ERRORLEVEL%
)
echo All dependencies installed successfully.
pause