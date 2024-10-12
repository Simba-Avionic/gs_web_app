#!/bin/bash

command_exists() {
  command -v "$1" &> /dev/null
}

get_python_version() {
  python3 --version 2>&1 | awk '{print $2}' | cut -d'.' -f1,2
}

# Check if Python 3 is installed and if the version is 3.11 or higher
if command_exists python3; then
  current_version=$(get_python_version)
  if (( $(echo "$current_version < 3.11" | bc -l) )); then
    echo "Python version is below 3.11. Current version: $current_version"
  else
    echo "Python version is 3.11 or higher. Current version: $current_version"
  fi
else
  echo "Python 3 is not installed."
fi

# Docker installation
if command_exists docker; then
  echo "Docker is already installed."
else
  echo "Docker is not installed. Installing Docker..."
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg lsb-release

  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

  sudo systemctl start docker
  sudo systemctl enable docker

  echo "Docker installation complete."
fi

# ROS2 Humble installation
if command_exists ros2; then
  ros_version=$(printenv ROS_DISTRO 2>&1)
  if [[ $ros_version == *"humble"* ]]; then
    echo "ROS2 Humble is already installed."
  else
    echo "ROS2 is installed, but not the Humble version."
  fi
else
  echo "ROS2 Humble is not installed. Installing ROS2 Humble..."

  sudo apt update && sudo apt install -y software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install -y curl gnupg2 lsb-release

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

  sudo apt update
  sudo apt install -y ros-humble-desktop

  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc

  echo "ROS2 Humble installation complete."
fi

# Check if the 'venv' directory exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment in 'venv' directory..."
    python3 -m venv venv
else
    echo "'venv' directory already exists. No action needed."
fi

# Activate the virtual environment
echo "Activating the virtual environment..."
source venv/bin/activate

if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "Virtual environment activated!"

    # Check if requirements.txt exists and install packages
    if [ -f "requirements.txt" ]; then
        echo "Installing packages from requirements.txt..."
        pip install -r requirements.txt
        echo "Packages installed successfully!"
    else
        echo "No requirements.txt found. Skipping package installation."
    fi
else
    echo "Failed to activate the virtual environment. Skipping package installation."
fi

install_npm() {
    echo "npm is not installed. Installing npm..."
    sudo apt update
    sudo apt install -y npm
    echo "npm installed, installing node..."

    sudo npm cache clean -f
    sudo npm install -g n
    sudo n stable
    echo "node installed successfully!"
}

if command -v npm &> /dev/null; then
    echo "npm is already installed at: $(which npm)"

    if grep -qEi "(microsoft|wsl)" /proc/version &> /dev/null; then
        echo "Detected WSL environment. Installing npm using apt regardless."
        install_npm
    fi
else
    echo "npm is not installed. Installing npm..."
    install_npm
fi

# Install frontend (node_modules)
if [ -d "app" ]; then
    echo "Navigating to 'app' directory..."
    cd app

    # Check if package.json exists before running npm install
    if [ -f "package.json" ]; then
        echo "Installing npm packages..."
        npm install
        echo "npm packages installed successfully!"
    else
        echo "No package.json found in 'app' directory. Skipping npm installation."
    fi
else
    echo "'app' directory does not exist. Skipping npm installation."
fi

cd ..

if [ -d "gs_interfaces" ]; then
    cd gs_interfaces
    echo "Running generate_ros2_messages.py script"
    python3 generate_ros2_messages.py
fi

install_colcon() {
    echo "Installing colcon using apt..."
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions
    echo "colcon installed successfully!"
}

if command -v colcon &> /dev/null; then
    echo "Running colcon build for the 'gs_interfaces' package..."
    if ! colcon build --packages-select gs_interfaces; then
        echo "colcon build failed. Attempting to install colcon and retry the build."
        install_colcon
        echo "Retrying colcon build..."
        colcon build --packages-select gs_interfaces
    else
        echo "colcon build completed successfully!"
    fi
else
    echo "colcon is not installed. Installing colcon using apt..."
    install_colcon

    echo "Retrying colcon build for the 'gs_interfaces' package..."
    colcon build --packages-select gs_interfaces
fi