FROM ros:humble

RUN mkdir -p /simba_ws/src
COPY gs_interfaces /simba_ws/src/gs_interfaces

WORKDIR /simba_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select gs_interfaces

RUN apt-get update && apt-get install -y \
    python3-pip \
    gcc \
    ffmpeg \
    iputils-ping \
 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

COPY backend .
COPY mavlink ./mavlink
COPY shared ./shared
COPY config.json /simba_ws

EXPOSE 2137
CMD bash -c "\
source /opt/ros/humble/setup.bash && \
source /simba_ws/install/setup.bash && \
uvicorn main:app --host 0.0.0.0 --port 2137"