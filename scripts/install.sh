#!/bin/bash

command_exists() {
  command -v "$1" &> /dev/null
}

get_python_version() {
  python3 --version 2>&1 | awk '{print $2}' | cut -d'.' -f1,2
}

# Check if Python 3 is installed and if the version is 3.10 or higher
if command_exists python3; then
  current_version=$(get_python_version)
  if (( $(echo "$current_version == 3.10" | bc -l) )); then
    echo "Python version is 3.10. No further action needed."
  else
    echo "Python version is: $current_version. The app was not tested on this distro."
  fi
else
  echo "Python 3.10 is not installed."
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
    sudo apt install python3.10-venv
    python3 -m venv venv
    echo "source $HOME/gs_web_app/venv/bin/activate" >> ~/.bashrc
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
  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
  sudo apt-get install -y nodejs
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
if [ -d "frontend" ]; then
    echo "Navigating to 'frontend' directory..."
    cd frontend

    # Check if package.json exists before running npm install
    if [ -f "package.json" ]; then
        echo "Installing npm packages..."
        npm install
        echo "npm packages installed successfully!"
    else
        echo "No package.json found in 'frontend' directory. Skipping npm installation."
    fi
else
    echo "'frontend' directory does not exist. Skipping npm installation."
fi

cd ..

install_colcon() {
    echo "Installing colcon using apt..."
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions
    sudo touch "$HOME/gs_web_app/venv/COLCON_IGNORE"
    echo "colcon installed successfully!"
}

if command -v colcon &> /dev/null || [ -d "$HOME/gs_web_app/build" ]; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

    echo "Running colcon build for the 'gs_interfaces' package..."
    if ! colcon --log-base build/log build --packages-select gs_interfaces --build-base build/build --install-base build/install; then
        echo "colcon build failed. Attempting to install colcon and retry the build."
        install_colcon
        echo "Retrying colcon build..."
        source /opt/ros/humble/setup.bash
        colcon --log-base build/log build --packages-select gs_interfaces --build-base build/build --install-base build/install
    else
        echo "colcon build completed successfully!"
    fi
else
    source /opt/ros/humble/setup.bash
    echo "colcon is not installed. Installing colcon using apt..."
    install_colcon

    echo "Retrying colcon build for the 'gs_interfaces' package..."
    colcon --log-base build/log build --packages-select gs_interfaces --build-base build/build --install-base build/install
fi

# Setup Mavlink dialect
./mavlink/setup.sh

# Add permissions for docker and serial ports
sudo usermod -aG dialout $USER
sudo usermod -aG docker $USER

sudo apt install chromium-browser ffmpeg i2c-tools python3-smbus raspi-config

# Enable app to automatically run when system is booting
sudo cp $HOME/gs_web_app/services/simba-app.service /etc/systemd/system/
sudo systemctl daemon-reexec
sudo systemctl enable simba-app.service
sudo systemctl start simba-app.service

sudo cp $HOME/gs_web_app/services/oled-display.service /etc/systemd/system/
sudo systemctl daemon-reexec
sudo systemctl enable oled-display.service
sudo systemctl start oled-display.service

echo "Application installed successfully, please reboot the system."