## How to run the Docker container

### Building the Image:

* In the miss_idisplay directory, run this code to build the image:

```sh
./docker/build.sh

```

### Running the docker container:

* To start the container, run the `up` command.
**Note:** If you are running this locally on your PC for development, use the `docker-compose.local.yaml` file to ensure the web server runs correctly in your environment.

```sh
# To run on the robot (Production):
docker compose -f docker/docker-compose.yaml up

# To run locally (Development):
docker compose -f docker/docker-compose.local.yaml up

```

* To stop and remove the container, run the `down` command:

```sh
# Stop the production container:
docker compose -f docker/docker-compose.yaml down

# Stop the local development container:
docker compose -f docker/docker-compose.local.yaml down

```

* To see the container logs in real time:

```sh
# Production container:
docker logs -f miss_idisplay

# Local development container:
docker logs -f miss_idisplay_dev

```

* To restart the container (useful after a `git pull` with changes to the files):

```sh
# Restart production:
docker compose -f docker/docker-compose.yaml restart

# Restart local development:
docker compose -f docker/docker-compose.local.yaml restart

```

---

## How to run the Flask server and change the HTML pages inside the container?

### Running the Flask server:

* Inside the `flaskServer` folder, run the Flask python code with:

```sh
gunicorn --threads 5 --workers 1 --bind 0.0.0.0:8080 app:app

```

---

### Available Test Cases (State vs HTML)

You can interact with the interface in two ways: by triggering a **State** (which includes animated borders, text, and the corresponding face) or by directly changing the **HTML** (which only loads the specific screen/face).

---

#### 1. Testing Locally (on PC)
Test video (currently on V2): https://www.youtube.com/watch?v=e-3FMok3maE

To test the interface on your PC without ROS2 or Docker, you can run the local test server.
1. Install requirements: `pip install flask flask-socketio`
2. Run the local server: `python3 flaskServer/app_local.py`
3. Open your browser at `http://localhost:8050`

_Note:_ End process: `fuser -k 8050/tcp`

Open a new terminal and use these `curl` commands to test the states:

```sh
# Normal/Idle (Removes borders)
curl -X POST http://localhost:8050/post_trigger_state -d "state=idle"

# Emotions & Interactions
curl -X POST http://localhost:8050/post_trigger_state -d "state=listening"
curl -X POST http://localhost:8050/post_trigger_state -d "state=thinking"
curl -X POST http://localhost:8050/post_trigger_state -d "state=talking"
curl -X POST http://localhost:8050/post_trigger_state -d "state=ready"

# Actions
curl -X POST http://localhost:8050/post_trigger_state -d "state=grabbing"
curl -X POST http://localhost:8050/post_trigger_state -d "state=placing"

# Task "Personal Recognition"
curl -X POST http://localhost:8050/post_trigger_state -d "state=scanning"
curl -X POST http://localhost:8050/post_trigger_state -d "state=pointing"

# Task "Navigation and Follow Me"
curl -X POST http://localhost:8050/post_trigger_state -d "state=navigating"
curl -X POST http://localhost:8050/post_trigger_state -d "state=following"

# System Alerts
curl -X POST http://localhost:8050/post_trigger_state -d "state=help"
curl -X POST http://localhost:8050/post_trigger_state -d "state=error"

# Sending text (Miss Piggy or Operator)
curl -X POST http://localhost:8050/post_subtitle_human -d "text=Hey Miss Piggy, please go to the kitchen."
curl -X POST http://localhost:8050/post_subtitle_robot -d "text=I'm going to the kitchen right now."
```

To test **direct HTML loading** (without the borders/state UI) locally:
```sh
curl -X POST http://localhost:8050/post_trigger_state -d "state=idle"

# Idle then others
curl -X POST http://localhost:8050/post_trigger_html_change -d "data=navigation"
curl -X POST http://localhost:8050/post_trigger_html_change -d "data=following"
# You can replace "data=..." with any HTML filename (without .html)
```

---

#### 2. Testing with Docker Compose Local (on PC)

To test the interface inside a Docker container on your PC, without the robot hardware. It runs `app.py` on port `8080`.
1. Build the Docker image (only needed once): `./docker/build.sh`
2. Start the container: `docker compose -f docker/docker-compose.local.yaml up`
3. Open your browser at `http://localhost:8080`

_Note:_ Stop the container: `docker compose -f docker/docker-compose.local.yaml down`

Open a new terminal and use these `curl` commands to test the states and HTML screens:

```sh
# Normal/Idle (Removes borders)
curl -X POST http://localhost:8080/post_trigger_state -d "state=idle"

# Emotions & Interactions
curl -X POST http://localhost:8080/post_trigger_state -d "state=listening"
curl -X POST http://localhost:8080/post_trigger_state -d "state=thinking"
curl -X POST http://localhost:8080/post_trigger_state -d "state=talking"
curl -X POST http://localhost:8080/post_trigger_state -d "state=ready"

# Actions
curl -X POST http://localhost:8080/post_trigger_state -d "state=grabbing"
curl -X POST http://localhost:8080/post_trigger_state -d "state=placing"

# Task "Personal Recognition"
curl -X POST http://localhost:8080/post_trigger_state -d "state=scanning"
curl -X POST http://localhost:8080/post_trigger_state -d "state=pointing"

# Task "Navigation and Follow Me"
curl -X POST http://localhost:8080/post_trigger_state -d "state=navigating"
curl -X POST http://localhost:8080/post_trigger_state -d "state=following"

# System Alerts
curl -X POST http://localhost:8080/post_trigger_state -d "state=help"
curl -X POST http://localhost:8080/post_trigger_state -d "state=error"
```

To test **direct HTML loading** (without the borders/state UI):
```sh
# Idle then others
curl -X POST http://localhost:8080/post_trigger_html_change -d "data=normal"
curl -X POST http://localhost:8080/post_trigger_html_change -d "data=following"
# You can replace "data=..." with any HTML filename (without .html)
```

To test **subtitles**, you can publish to the ROS2 topics directly inside the container (publishing 3 times to ensure delivery):

```sh
# Publish text as the human operator (ASR output)
docker exec miss_idisplay_dev bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -t 3 /asr_output std_msgs/String \"data: 'Hey Piggy, go to the kitchen'\""

# Publish text as the robot speaking (TTS)
docker exec miss_idisplay_dev bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -t 3 /text_to_speech std_msgs/String \"data: 'Going to the kitchen now'\""
```

---

#### 3. Testing on the Robot (Production)

To deploy on the robot (Raspberry Pi), the server runs with `gunicorn` on port `8080` with ROS2 active.
1. Build the Docker image (only needed once): `./docker/build.sh`
2. Start the container: `docker compose -f docker/docker-compose.yaml up -d`
3. Open your browser at `http://localhost:8080`

_Note:_ Replace `localhost` with the Raspberry Pi's IP if testing from another machine. Find it with: `hostname -I`

_Note:_ Stop the container: `docker compose -f docker/docker-compose.yaml down`

Open a new terminal and use these `curl` commands to test the states and HTML screens (replace `0.0.0.0` with the Raspberry Pi's IP if testing remotely):

```sh
# Normal/Idle (Removes borders)
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=idle"

# Emotions & Interactions
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=listening"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=thinking"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=talking"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=ready"

# Actions
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=grabbing"
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=grabbing"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=placing"

# Task "Personal Recognition"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=scanning"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=pointing"

# Task "Navigation and Follow Me"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=navigating"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=following"

# System Alerts
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=help"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=error"
```

To test **direct HTML loading** (without the borders/state UI):
```sh
# Format: curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data={html_file}"

# Actions
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=grabbing"
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=placing"
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=manipulator"

# Utilities / System Screens
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=cam"
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=map"
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=video"
```

To test **subtitles**, you can publish to the ROS2 topics directly inside the container (publishing 3 times to ensure delivery):

```sh
# Publish text as the human operator (ASR output)
docker exec miss_idisplay bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -t 3 /asr_output std_msgs/String \"data: 'Hey Piggy, go to the kitchen'\""

# Publish text as the robot speaking (TTS)
docker exec miss_idisplay bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -t 3 /text_to_speech std_msgs/String \"data: 'Going to the kitchen now'\""
```
