# ==========================================
# Stage 1: Build the Frontend
# ==========================================
FROM node:18 AS frontend-builder
WORKDIR /app/frontend

COPY frontend/package*.json ./
RUN npm install
COPY frontend/ ./
RUN npm run build


# ==========================================
# Stage 2: Build the ROS/Python Backend
# ==========================================
FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    xmlstarlet \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Setup Python Virtual Environment
RUN python3 -m venv /app/venv

# --- LAYER CACHING: Install dependencies FIRST ---
# This makes subsequent builds incredibly fast if you only change code
COPY requirements.txt /app/
RUN /app/venv/bin/pip install --upgrade pip && \
    /app/venv/bin/pip install -r /app/requirements.txt

# --- Build MAVLink ---
COPY mavlink/ /app/mavlink/
RUN cd mavlink && chmod +x setup.sh && ./setup.sh simba simba_mavlink/simba.xml

# --- Build ROS 2 Messages ---
# Only copy the src folder where your ROS 2 interfaces live
COPY gs_interfaces/ /app/gs_interfaces/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select gs_interfaces --build-base build/build --install-base build/install"

# --- COPY THE REST OF YOUR CODE ---
COPY . /app/

# Copy the built frontend from Stage 1
COPY --from=frontend-builder /app/frontend/dist /app/frontend/dist

# Build MAVLink definitions & ROS 2 custom messages
RUN cd mavlink && chmod +x setup.sh && ./setup.sh simba simba_mavlink/simba.xml
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select gs_interfaces --build-base build/build --install-base build/install"

# Setup entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]