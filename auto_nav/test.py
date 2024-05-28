import glob
import os

# Debugging file paths and packages
launch_files = glob(os.path.join("launch", "*.launch.py"))
config_files = glob(os.path.join("config", "*.yaml"))

print("Launch files found:", launch_files)
print("Config files found:", config_files)

# Check directory presence
print("Current working directory:", os.getcwd())
print("Directories and files in current directory:", os.listdir())
