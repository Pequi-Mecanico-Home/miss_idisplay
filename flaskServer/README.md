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

#### 2. Testing on the Robot (Production)
When running the real `app.py` server (via Docker) on the robot or a ROS2 environment, the server runs on port `8080`. 
*(Note: Replace `0.0.0.0` with the Raspberry Pi's IP if testing from another machine).*

**Triggering States:**
```sh
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=listening"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=thinking"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=navigating"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=following"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=scanning"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=pointing"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=help"
curl -X POST http://0.0.0.0:8080/post_trigger_state -d "state=error"
```

**Direct HTML Loading:**
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

---

## Sending subtitles:

There are two ways to send subtitles to the display.

### 1. Sending a subtitle file:

*If you're using raspi:*

```sh
curl -F "file=@/path/to/subtitle.txt" http://0.0.0.0:8080/upload

```

*If you're using another machine:*

```sh
curl -F "file=@/path/to/subtitle.txt" http://{rasp ip}:8080/upload

```

### 2. Sending live text (Miss Piggy or Operator):

You can send text directly to the interface using POST requests. *Note: Ensure you have implemented the corresponding endpoints in your `app.py`.*

**Miss Piggy (Robot):**

```sh
curl -X POST http://localhost:8080/post_subtitle_robot -d "text=I'm going to the kitchen right now."

```

**Operator (Human):**

```sh
curl -X POST http://localhost:8080/post_subtitle_human -d "text=Hey Miss Piggy, please go to the kitchen."

```
