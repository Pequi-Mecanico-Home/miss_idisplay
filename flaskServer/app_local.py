"""
Versão local do servidor Flask para testar a interface sem ROS2.
Roda direto no PC sem precisar de Docker nem de dependências ROS.

Uso:
    cd flaskServer
    pip install flask flask-socketio
    python app_local.py

Acesse: http://localhost:8050
"""

from threading import Lock
from flask import Flask, render_template, request
from flask_socketio import SocketIO

async_mode = None

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)
thread_lock = Lock()

@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

@app.route('/post_trigger_html_change', methods=['POST'])
def post_trigger_html_change():
    data = request.form.get('data')
    if data:
        socketio.emit('html_updated', {'data': data})
        return f"html changed to: {data}"
    return "No data provided", 400

@app.route('/post_trigger_state', methods=['POST'])
def post_trigger_state():
    state = request.form.get('state')
    if state:
        socketio.emit('state_changed', {'state': state})
        return f"state changed to: {state}"
    return "No state provided", 400

@app.route('/post_subtitle_robot', methods=['POST'])
def post_subtitle_robot():
    text = request.form.get('text', '')
    socketio.emit('subtitle_robot', text)
    return "Robot subtitle sent", 200

@app.route('/post_subtitle_human', methods=['POST'])
def post_subtitle_human():
    text = request.form.get('text', '')
    socketio.emit('subtitle_human', text)
    return "Human subtitle sent", 200

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
            socketio.sleep(1)
    return 'File uploaded successfully\n', 200

if __name__ == '__main__':
    print("\n  Miss Piggy Interface - Local Test Server")
    print("  http://localhost:8050\n")
    socketio.run(app, host='0.0.0.0', port=8050)
