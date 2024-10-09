# This module sets up a Flask web server with SocketIO integration to handle real-time
# websocket communication and HTTP requests.
# Routes:
# - /: Renders the index.html template.
# - /html: Receives data from a form, logs the data, and sends an HTML file based on the data.
# - /trigger_html_change: Emits a 'change_html' event via SocketIO.
# - /post_trigger_html_change: Receives data from a POST request and emits an 'html_updated' event with the data.
# SocketIO Events:
# - my_event: Handles a custom event, increments a session counter, and emits a response.
# - test_message: Handles 'test_message' events, prints the received message, and emits a test response.
# - change_html: Handles 'change_html' events, prints a message, and broadcasts an 'html_updated' event.
# Global Variables:
# - app: The Flask application instance.
# - socketio: The SocketIO instance for the Flask app.
# - thread: A placeholder for a background thread (currently unused).
# - thread_lock: A threading lock to manage access to the background thread.


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

# To set a endpoint in the web page, we use @app.route, each endpoint has an utility
@app.route('/')
# With this function the code render the html page choosed and set the async_mode to be useful with socketIO and its events
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

@app.route('/html')
def get_html():
    data = request.form.get('data')
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(f'Received data: {data}')
    if data:
        html_path = f'/static/html-css/{data}.html' 
        print("html path", html_path) 
        return send_file(html_path, mimetype='text/html')
    return "No data provided", 501
    
@app.route('/trigger_html_change')
def trigger_html_change():
    socketio.emit('change_html')
    return "html change event emitted"

@socketio.event
def my_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']})

@socketio.on('test_message')
def handle_message(data):
    print('received message: ' + str(data))
    emit('test_response', {'data': 'Test response sent'})

@socketio.on('change_html')
def change_html():
    print('html change requested')
    emit('html_updated', {'data': 'html has been updated'}, broadcast=True)

@app.route('/post_trigger_html_change', methods=['POST'])
def post_trigger_html_change():
# declares data and retrieves the value associated with form data
    data = request.form.get('data')
# After providing some data the socket.emit send tht html_updated to the index.html with the correct data
    if data:
        socketio.emit('html_updated', {'data': data})
        return "html change event emitted with data"
# If no data is sent, returns an error
    return "No data provided", 400
# Ensuring that the server starts only when the script is executed directly
if __name__ == '__main__':
    socketio.run(app)