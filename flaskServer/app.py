# Imports for OpenCV operations
import cv2
from cv_bridge import CvBridge

# Imports for ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Imports for threading operations
import signal, sys
from threading import Thread, Event, Lock
from threading import Thread, Event, Lock
import urllib.request
import urllib.parse

from flask import Flask, render_template, request, Response
from flask_socketio import SocketIO

from collections import deque
import time
async_mode = 'threading'

frame = None # Global variable frame (the holy image)

bridge = CvBridge()
event = Event()

# Global variable to store concatenated messages
concatenated_text = ""

def on_text_human(msg):
    # Callback function for text subscription
    text_data = msg.data  # Assuming the message has a 'data' attribute containing the text
    try:
        data = urllib.parse.urlencode({'text': text_data}).encode()
        urllib.request.urlopen('http://127.0.0.1:8080/post_subtitle_human', data=data, timeout=1)
    except Exception as e:
        print(f"Error posting human text: {e}")

# def on_text_robot(msg):
#     global concatenated_text
#     text_data = msg.data
#     words = text_data.split()
    
#     if not hasattr(on_text_robot, 'word_queue'):
#         on_text_robot.word_queue = deque(maxlen=10)
    
#     on_text_robot.word_queue.extend(words)
    
#     concatenated_text = ' '.join(on_text_robot.word_queue)
#     socketio.emit('subtitle_robot', concatenated_text)
    
def on_text_robot(msg):
    global concatenated_text
    print(f"CALLBACK FIRED: {msg.data}", flush=True)
    text_data = msg.data
    words = text_data.split()
    
    if not hasattr(on_text_robot, 'word_queue'):
        on_text_robot.word_queue = deque(maxlen=8)
        on_text_robot.last_message_time = time.time()
    
    on_text_robot.word_queue.extend(words)
    on_text_robot.last_message_time = time.time()
    
    concatenated_text = ' '.join(on_text_robot.word_queue)
    try:
        data = urllib.parse.urlencode({'text': concatenated_text}).encode()
        urllib.request.urlopen('http://127.0.0.1:8080/post_subtitle_robot', data=data, timeout=1)
    except Exception as e:
        print(f"Error posting robot text: {e}")
    
    # Clear the queue after 3 seconds without messages
    def clear_queue():
        while True:
            if time.time() - on_text_robot.last_message_time > 3:
                on_text_robot.word_queue.clear()
            time.sleep(1)
    
    if not hasattr(on_text_robot, 'clear_thread'):
        on_text_robot.clear_thread = Thread(target=clear_queue)
        on_text_robot.clear_thread.daemon = True
        on_text_robot.clear_thread.start()



rclpy.init(args=None)

node_subtitle = rclpy.create_node('Show_subtitle_python')

Thread(target=lambda: rclpy.spin(node_subtitle)).start()  # Starting the Thread with a target in the node

subscription_subtitle_human = node_subtitle.create_subscription(String, "/asr_output", on_text_human, 10)  # Creating the Subscribe node for text messages
subscription_subtitle_robot = node_subtitle.create_subscription(String, '/text_to_speech', on_text_robot,10)


def on_image(msg):
    global frame
    
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
    cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    frame = cv2.imencode(".jpg",cv_image)[1].tobytes()
    event.set()

node = rclpy.create_node('Show_image_python')

Thread(target=lambda:node).start() # Starting the Thread with a target in the node

subscription = node.create_subscription(Image,"/camera/color/image_raw", on_image, 10) # Creating the Subscribe node

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)
thread = None
thread_lock = Lock()

## Function that runs 'spin_once' and waits for a new frame
def get_frame():
    rclpy.spin_once(node,timeout_sec=1.0)
    event.wait()
    event.clear()
    return frame

## Function that prepare the frame to be loaded into template
def gen():
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

## Function that finish the actual context
def signal_handler(signal, frame):
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler) # Calls the 'signal_handler' and finish the actual signal (like a Ctrl + C)

# To set a endpoint in the web page, we use @app.route, each endpoint has an utility
@app.route('/')
# With this function I can render the html page I send and set the async_mode to be useful with socketIO and its events
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

@app.route('/post_trigger_html_change', methods=['POST'])
def post_trigger_html_change():
# declares data and retrievesthe value associated with form data
    data = request.form.get('data')
# After providing some data the socket.emit send tht html_updated to the index.html with the correct data
    if data:
        socketio.emit('html_updated', {'data': data})
        return "html change event emitted with data"
    return "No data provided", 400

## Function that insert the new frame in template
@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/post_trigger_state', methods=['POST'])
def post_trigger_state():
    state = request.form.get('state')
    if state:
        socketio.emit('state_changed', {'state': state})
        return f"state changed to {state}"
    return "No state provided", 400

@app.route('/upload', methods=['POST'])
def upload():
    if 'file' not in request.files:
        return 'No file part', 400
    file = request.files['file']
    if file.filename == '':
        return 'No selected file', 400
    if file:
        content = file.read().decode('utf-8')
        for line in content.splitlines():
            socketio.emit('subtitle_robot', line)
            socketio.sleep(1)  # Wait for 3 seconds before sending the next line
    return 'File uploaded successfully \n', 200

# Robot subtitle
@app.route('/post_subtitle_robot', methods=['POST'])
def post_subtitle_robot():
    text = request.form.get('text', '')
    socketio.emit('subtitle_robot', text)
    return "Robot subtitle sent", 200

# Human subtitle
@app.route('/post_subtitle_human', methods=['POST'])
def post_subtitle_human():
    text = request.form.get('text', '')
    socketio.emit('subtitle_human', text)
    return "Human subtitle sent", 200
 
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True)
