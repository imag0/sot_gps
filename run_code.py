import subprocess

# Run main.py in the background
subprocess.Popen(['python', 'main.py'])

# Run pipi.py, which will wait until the marker file appears
subprocess.run(['python', 'pipi.py'])