from threading import Lock
from flask import Flask, render_template, session, send_file, request
from flask_socketio import SocketIO, emit
import os
import logging

async_mode = None

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)
thread = None
thread_lock = Lock()


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

@app.route('/image')
def get_image():
    data = request.form.get('data')
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(f'Received data: {data}')
    if data:
        image_path = f'/home/batman/interaction_miss/project2/static/{data}.gif' 
        print("image path", image_path) # Update this path to your image
        return send_file(image_path, mimetype='image/gif')
    return "No data provided", 501
    
@app.route('/trigger_image_change')
def trigger_image_change():
    socketio.emit('change_image')
    return "Image change event emitted"

@socketio.event
def my_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']})

@socketio.on('test_message')
def handle_message(data):
    print('received message: ' + str(data))
    emit('test_response', {'data': 'Test response sent'})

@socketio.on('change_image')
def change_image():
    print('Image change requested')
    emit('image_updated', {'data': 'Image has been updated'}, broadcast=True)

@app.route('/post_trigger_image_change', methods=['POST'])
def post_trigger_image_change():
    data = request.form.get('data')
    if data:
        socketio.emit('image_updated', {'data': data})
        return "Image change event emitted with data"
    return "No data provided", 400

if __name__ == '__main__':
    socketio.run(app)