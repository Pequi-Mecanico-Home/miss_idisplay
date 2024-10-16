FROM ros:humble-perception-jammy

WORKDIR /idisplay_ws

RUN apt update && apt install python3-pip gunicorn -y \ 
    && pip3 install --upgrade pip \
    && pip3 install flask flask-cors flask_socketio\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

